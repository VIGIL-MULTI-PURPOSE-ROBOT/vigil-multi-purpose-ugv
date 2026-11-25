#!/usr/bin/python3
"""Footprint-aware A* planner + navigation state machine (no ROS imports).

Robot-size logic:
  * edge hazards   = OBSTACLE / CLIFF (geometric edges the body must not overlap)
  * hard forbidden = distance to an edge hazard < robot_width/2          (body overlaps it)
  * soft forbidden = distance to an edge hazard < robot_width/2 + margin (inflated region)
  * STEEP cells are forbidden for the robot CENTRE only: their slope is already a plane
    fit over the robot's own 0.9 m contact patch, so inflating them again by the body
    width would count the footprint twice
  * proximity cost ramps from the inscribed to the circumscribed radius, so paths
    keep away from edges when there is room
  * every candidate path is swept with the oriented footprint rectangle
    (length x width + margin); corners that would cross an edge reject it
  * rotating in place is only allowed when the circumscribed circle is clear
"""
import heapq
import math
from collections import deque

import cv2
import numpy as np

from terrain_core import (UNKNOWN, SAFE, UNEVEN, POSSIBLE_DROP, CLASS_NAMES, CLIFF, OBSTACLE, STEEP, CLIMBABLE)

S2 = math.sqrt(2.0)
NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, S2), (-1, 1, S2), (1, -1, S2), (1, 1, S2)]


class Planner:
    def __init__(self, tp, nav):
        self.tp = tp
        self.nav = nav
        self.res = tp.resolution
        self.f = max(1, int(round(nav.get('planning_resolution', 0.2) / tp.resolution)))
        self.pres = self.res * self.f
        self.classes = None
        self.lethal = None
        self.penalty = None      # plan-grid extra cost (tilt-limit / stuck spots)
        # Proven ground (maintained by NavigatorCore.mark_proven): cells the robot has physically
        # stood on while level. proven_seen: touched by the footprint (UNKNOWN there -> SAFE, so
        # the robot may reverse back along its own track even where the forward camera never
        # looked, e.g. the spawn area). proven_drop: deep inside the footprint (proven_drop_inset
        # from every side); a CLIFF / POSSIBLE_DROP cell there cannot be a real drop because the
        # robot is standing level on it - it is a mapping artefact and is cleared for navigation.
        self.proven_seen = None
        self.proven_drop = None

    # -------------------------------------------------------------- costmap
    def build(self, classes, slope=None, rough=None):
        """Cost map. Only CLIFF / OBSTACLE are inflated hard obstacles. Hills are cost:
        CLIMBABLE and slope/roughness add cost; STEEP (beyond the climbing limit) blocks the
        robot centre in normal mode and is only very expensive in climb mode."""
        tp, nav = self.tp, self.nav
        self.raw = (classes, slope, rough)
        if self.proven_seen is not None and self.proven_seen.shape == classes.shape:
            classes = classes.copy()
            classes[self.proven_seen & (classes == UNKNOWN)] = SAFE
            classes[self.proven_drop & np.isin(classes, (CLIFF, POSSIBLE_DROP))] = SAFE
        self.classes = classes
        lethal = np.isin(classes, (OBSTACLE, CLIFF))
        steep = classes == STEEP
        self.lethal = lethal
        self.steep = steep
        # unknown / possible-drop terrain within unknown_near_cliff m of a mapped drop: treated as a
        # hazard for motions that sweep where the forward camera has not looked (turns, reversing)
        drops = np.isin(classes, (CLIFF, POSSIBLE_DROP)).astype(np.uint8)
        r = max(1, int(round(nav.get('unknown_near_cliff', 1.0) / tp.resolution)))
        near = cv2.dilate(drops, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))) > 0
        self.risky_unknown = ((classes == UNKNOWN) & near) | (classes == POSSIBLE_DROP)
        free = (~lethal).astype(np.uint8)
        if lethal.any():
            dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        else:
            dist = np.full(classes.shape, 1e3, np.float32)
        self.dist = dist
        # distance (m, cell centres) to the nearest CLIFF / OBSTACLE / POSSIBLE_DROP cell, for the
        # continuous clearance used by the motion shield inside the safety band
        haz = lethal | (classes == POSSIBLE_DROP)
        if haz.any():
            self.dist_haz = cv2.distanceTransform((~haz).astype(np.uint8), cv2.DIST_L2, 5) * self.res
        else:
            self.dist_haz = None
        hard = dist < tp.half_width
        soft = dist < tp.inscribed_radius
        r_in, r_out = tp.inscribed_radius, tp.circumscribed_radius
        prox = np.clip((r_out - dist) / max(1e-6, r_out - r_in), 0, 1)
        cost = (1.0 + nav.get('unknown_cost', 1.5) * (classes == UNKNOWN)
                + nav.get('uneven_cost', 2.0) * (classes == UNEVEN)
                + nav.get('possible_drop_cost', 25.0) * (classes == POSSIBLE_DROP)
                + nav.get('proximity_cost', 6.0) * prox
                + nav.get('climbable_cost', 0.5) * (classes == CLIMBABLE)).astype(np.float32)
        if slope is not None:
            # slope as a smooth cost: 0 on flat ground, slope_cost at the climbing limit
            cost += nav.get('slope_cost', 1.0) * np.clip(slope / tp.max_climb_slope_deg, 0, 1.5) ** 2
        if rough is not None:
            cost += nav.get('roughness_cost', 1.0) * np.clip(rough / max(tp.uneven_roughness, 1e-3), 0, 3)
        self.slope = slope
        f, n = self.f, classes.shape[0]
        m = (n + f - 1) // f
        pad = m * f - n

        def pool(a, fill):
            a = np.pad(a, ((0, pad), (0, pad)), constant_values=fill)
            return a.reshape(m, f, m, f).max(axis=(1, 3))
        self.p_hard = pool(hard, True)
        self.p_soft = pool(soft, True)
        self.p_steep = pool(steep, True)
        self.p_cost = pool(cost, 1.0)
        self.m = m
        if self.penalty is None or self.penalty.shape != (m, m):
            self.penalty = np.zeros((m, m), np.float32)

    def add_penalty(self, x, y, radius, value):
        """Remember a place that failed (tilt limit, stuck) so replans try elsewhere."""
        if self.penalty is None:
            return
        c = self.tp.map_origin_x + (np.arange(self.m) + 0.5) * self.pres
        X, Y = np.meshgrid(c, self.tp.map_origin_y + (np.arange(self.m) + 0.5) * self.pres)
        self.penalty[np.hypot(X - x, Y - y) <= radius] += value

    def to_pcell(self, x, y):
        return (int(math.floor((y - self.tp.map_origin_y) / self.pres)),
                int(math.floor((x - self.tp.map_origin_x) / self.pres)))

    def pcell_center(self, i, j):
        return (self.tp.map_origin_x + (j + 0.5) * self.pres, self.tp.map_origin_y + (i + 0.5) * self.pres)

    def _in(self, i, j):
        return 0 <= i < self.m and 0 <= j < self.m

    # ------------------------------------------------------------------ A*
    def corridor(self, a, b):
        """Plan-grid mask of cells within `planning_corridor` m of segment a-b."""
        margin = self.nav.get('planning_corridor', 6.0)
        c = self.tp.map_origin_x + (np.arange(self.m) + 0.5) * self.pres
        X, Y = np.meshgrid(c, self.tp.map_origin_y + (np.arange(self.m) + 0.5) * self.pres)
        ax, ay = a
        dx, dy = b[0] - ax, b[1] - ay
        L2 = dx * dx + dy * dy or 1e-9
        t = np.clip(((X - ax) * dx + (Y - ay) * dy) / L2, 0, 1)
        return np.hypot(X - ax - t * dx, Y - ay - t * dy) <= margin

    def _snap_goal(self, gi, gj, hard, radius=3.0):
        """If B itself is inside a hazard band, aim for the closest usable cell near it."""
        if not hard[gi, gj]:
            return gi, gj
        r = int(radius / self.pres)
        best = None
        for a in range(max(0, gi - r), min(self.m, gi + r + 1)):
            for b in range(max(0, gj - r), min(self.m, gj + r + 1)):
                if not hard[a, b] and not self.p_soft[a, b]:
                    d = (a - gi) ** 2 + (b - gj) ** 2
                    if best is None or d < best[0]:
                        best = (d, a, b)
        return (best[1], best[2]) if best else (gi, gj)

    def plan(self, start, goal, escape_radius=1.3, area=None, mode='normal'):
        """Return (path Nx2 world, info) or (None, info).
        mode 'normal': inside the A-B corridor, STEEP blocked.
        mode 'wide'  : whole map, STEEP blocked.
        mode 'climb' : whole map, STEEP allowed at high cost (hill-climbing attempt).
        CLIFF / OBSTACLE (+ footprint inflation) are hard in every mode."""
        if self.classes is None:
            return None, dict(reason='no map')
        si, sj = self.to_pcell(*start)
        gi, gj = self.to_pcell(*goal)
        if not (self._in(si, sj) and self._in(gi, gj)):
            return None, dict(reason='start/goal outside map')
        steep_cost = 0.0
        if mode == 'climb':
            steep_cost = self.nav.get('steep_climb_cost', 15.0)
            hard, soft = self.p_hard, self.p_soft
        else:
            hard, soft = self.p_hard | self.p_steep, self.p_soft | self.p_steep
            area = area if mode == 'normal' else None
        gi, gj = self._snap_goal(gi, gj, hard)
        goal = self.pcell_center(gi, gj) if (gi, gj) != self.to_pcell(*goal) else goal
        return self._astar(start, goal, si, sj, gi, gj, hard, soft, steep_cost, escape_radius, area, mode)

    def _astar(self, start, goal, si, sj, gi, gj, hard, soft, steep_cost, escape_radius, area, mode):
        esc = escape_radius / self.pres
        g = np.full((self.m, self.m), np.inf, np.float32)
        parent = np.full((self.m, self.m, 2), -1, np.int32)
        closed = np.zeros((self.m, self.m), bool)
        g[si, sj] = 0.0
        heap = [(0.0, 0.0, si, sj)]
        cost = self.p_cost + self.penalty + steep_cost * self.p_steep
        allowed = self.corridor(*area) if area is not None else np.ones((self.m, self.m), bool)
        allowed[si, sj] = True
        self._cur_soft, self._cur_cost = soft, cost   # used by the shortcut smoother
        found = False
        while heap:
            _, gc, i, j = heapq.heappop(heap)
            if closed[i, j]:
                continue
            closed[i, j] = True
            if i == gi and j == gj:
                found = True
                break
            for di, dj, dl in NEIGHBOURS:
                a, b = i + di, j + dj
                if a < 0 or b < 0 or a >= self.m or b >= self.m or closed[a, b] or not allowed[a, b]:
                    continue
                near_start = (a - si) ** 2 + (b - sj) ** 2 <= esc * esc
                near_goal = (a - gi) ** 2 + (b - gj) ** 2 <= esc * esc
                if soft[a, b]:
                    # The safety-margin band may only be entered right next to the
                    # robot (e.g. after an emergency stop) or right at B; never the
                    # hard band where the body itself would overlap a hazard.
                    if not (near_start or near_goal) or (hard[a, b] and not (near_start and hard[si, sj])):
                        continue
                    step = dl * (cost[a, b] + 20.0)
                else:
                    step = dl * cost[a, b]
                ng = gc + step
                if ng < g[a, b]:
                    g[a, b] = ng
                    parent[a, b] = (i, j)
                    h = math.hypot(a - gi, b - gj)
                    heapq.heappush(heap, (ng + h, ng, a, b))
        if not found:
            return None, dict(reason='no safe route with current map', expanded=int(closed.sum()))
        cells = [(gi, gj)]
        while cells[-1] != (si, sj):
            i, j = cells[-1]
            cells.append(tuple(parent[i, j]))
        cells.reverse()
        uses_steep = any(self.p_steep[i, j] for i, j in cells)
        cells = self._shortcut(cells, si, sj, esc)
        pts = [start] + [self.pcell_center(i, j) for i, j in cells[1:-1]] + [goal]
        path = densify(np.array(pts, float), 0.1)
        return path, dict(reason='ok', cost=float(g[gi, gj]), expanded=int(closed.sum()), mode=mode,
                          uses_steep=bool(uses_steep))

    def _segment_ok(self, a, b, si, sj, esc, limit):
        (i0, j0), (i1, j1) = a, b
        n = int(max(abs(i1 - i0), abs(j1 - j0)) * 2) + 1
        for t in np.linspace(0, 1, n):
            i, j = int(round(i0 + (i1 - i0) * t)), int(round(j0 + (j1 - j0) * t))
            if self._cur_soft[i, j] and (i - si) ** 2 + (j - sj) ** 2 > esc * esc:
                return False
            if self._cur_cost[i, j] > limit:
                return False
        return True

    def _shortcut(self, cells, si, sj, esc):
        out = [cells[0]]
        k = 0
        while k < len(cells) - 1:
            best = k + 1
            limit = self._cur_cost[cells[k + 1]] if k + 1 < len(cells) else 1
            for m in range(min(len(cells) - 1, k + 40), k + 1, -1):
                seg_max = max(self._cur_cost[c] for c in cells[k:m + 1])
                if self._segment_ok(cells[k], cells[m], si, sj, esc, max(limit, seg_max) + 1e-3):
                    best = m
                    break
            out.append(cells[best])
            k = best
        return out

    # ------------------------------------------------------ path evaluation
    def path_cost(self, path):
        c = 0.0
        for k in range(1, len(path)):
            i, j = self.to_pcell(*path[k])
            if not self._in(i, j):
                return math.inf
            c += math.hypot(*(path[k] - path[k - 1])) / self.pres * float(self.p_cost[i, j])
        return c

    def footprint_points(self, x, y, yaw, margin=None, length_extra=0.0):
        tp = self.tp
        m = tp.safety_margin if margin is None else margin
        hl, hw = tp.robot_length / 2 + m, tp.robot_width / 2 + m
        xs = np.arange(-hl, hl + length_extra + 1e-6, self.res / 2) + tp.footprint_center_x
        ys = np.arange(-hw, hw + 1e-6, self.res / 2)
        X, Y = np.meshgrid(xs, ys)
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * X - s * Y, y + s * X + c * Y

    def set_body_cells(self, x, y, yaw):
        """Hazard cells already under the robot's own (margin-0) footprint cannot be avoided by any
        motion (typically a spurious detection on ground the robot stands on). They are excluded
        from motion/route checks until the body leaves them, so moves are judged only on NEW hazard
        cells they would bring under the footprint: backing off is allowed, moving further over a
        real edge still touches new cells and stays rejected. Returns the number of such cells."""
        self.body_ignore = None
        self.body_mask = None
        if self.lethal is None:
            return 0
        wx, wy = self.footprint_points(x, y, yaw, margin=0.0)
        i = np.floor((wy - self.tp.map_origin_y) / self.res).astype(int).ravel()
        j = np.floor((wx - self.tp.map_origin_x) / self.res).astype(int).ravel()
        n = self.classes.shape[0]
        ok = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        i, j = i[ok], j[ok]
        self.body_mask = np.zeros_like(self.lethal)   # cells the robot is standing on right now
        self.body_mask[i, j] = True
        hit = self.lethal[i, j]
        if not hit.any():
            return 0
        self.body_ignore = np.zeros_like(self.lethal)
        self.body_ignore[i[hit], j[hit]] = True
        return int(self.body_ignore.sum())

    def lethal_at(self, wx, wy, strict=0, drops=False):
        """drops=True: unconfirmed drops (POSSIBLE_DROP) also block - used by the motion shield so the
        footprint (+margin) never moves onto a drop the mapper has not been able to confirm or clear
        (driving ALONGSIDE an edge never steepens the grazing angle enough to confirm it).
        strict=1: unknown terrain NEAR a mapped drop (risky_unknown) also blocks (turning, where the
        corners sweep the sides/rear that the camera has not looked at); strict=2: ALL unknown and
        POSSIBLE_DROP cells block (reversing). Cells the robot is standing on right now never block
        for these strict checks."""
        i = np.floor((wy - self.tp.map_origin_y) / self.res).astype(int)
        j = np.floor((wx - self.tp.map_origin_x) / self.res).astype(int)
        n = self.classes.shape[0]
        ok = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        out = np.zeros(wx.shape, bool)
        out[~ok] = True  # leaving the map is unsafe
        out[ok] = self.lethal[i[ok], j[ok]]
        ign = getattr(self, 'body_ignore', None)
        if ign is not None:
            out[ok] &= ~ign[i[ok], j[ok]]
        if drops:
            out[ok] |= self.classes[i[ok], j[ok]] == POSSIBLE_DROP
        if strict:
            c = self.classes[i[ok], j[ok]]
            if strict >= 2:
                unseen = (c == UNKNOWN) | (c == POSSIBLE_DROP)
            else:
                unseen = self.risky_unknown[i[ok], j[ok]]
            body = getattr(self, 'body_mask', None)
            if body is not None:
                unseen &= ~body[i[ok], j[ok]]
            out[ok] |= unseen
        cls = np.full(wx.shape, CLIFF, np.uint8)
        cls[ok] = self.classes[i[ok], j[ok]]
        return out, cls

    def start_clearance(self, x, y, yaw, drops=False):
        """Largest margin level (safety_margin, 0.10, 0.05, 0.0 m) at which the footprint at this
        pose touches no CLIFF/OBSTACLE cell (drops=True: nor POSSIBLE_DROP); -1.0 if the body itself
        already overlaps one."""
        for m in (self.tp.safety_margin, 0.10, 0.05, 0.0):
            if m > self.tp.safety_margin:
                continue
            wx, wy = self.footprint_points(x, y, yaw, margin=m)
            hit, _ = self.lethal_at(wx, wy, drops=drops)
            if not hit.any():
                return m
        return -1.0

    def clearance_cont(self, x, y, yaw):
        """Approximately continuous clearance (m) of the footprint rectangle to the nearest CLIFF /
        OBSTACLE / POSSIBLE_DROP cell: the distance field bilinearly interpolated along the footprint
        outline. Cells under the body (body_ignore) are excluded. Used to compare poses (does a
        motion bring the body closer?), not as an absolute distance. inf if no hazard is mapped."""
        d = getattr(self, 'dist_haz', None)
        if d is None:
            return math.inf
        ign = getattr(self, 'body_ignore', None)
        if ign is not None and ign.any():
            haz = (np.isin(self.classes, (OBSTACLE, CLIFF)) | (self.classes == POSSIBLE_DROP)) & ~ign
            if not haz.any():
                return math.inf
            d = cv2.distanceTransform((~haz).astype(np.uint8), cv2.DIST_L2, 5) * self.res
        tp = self.tp
        hl, hw = tp.robot_length / 2, tp.robot_width / 2
        step = self.res / 2
        u = np.arange(-hl, hl + 1e-6, step) + tp.footprint_center_x
        v = np.arange(-hw, hw + 1e-6, step)
        lx = np.concatenate([u, u, np.full(v.size, u[0]), np.full(v.size, u[-1])])
        ly = np.concatenate([np.full(u.size, -hw), np.full(u.size, hw), v, v])
        c, s = math.cos(yaw), math.sin(yaw)
        wx, wy = x + c * lx - s * ly, y + s * lx + c * ly
        fj = (wx - tp.map_origin_x) / self.res - 0.5
        fi = (wy - tp.map_origin_y) / self.res - 0.5
        n = d.shape[0]
        j0 = np.clip(np.floor(fj).astype(int), 0, n - 2); i0 = np.clip(np.floor(fi).astype(int), 0, n - 2)
        a = np.clip(fj - j0, 0, 1); b = np.clip(fi - i0, 0, 1)
        val = (d[i0, j0] * (1 - a) * (1 - b) + d[i0, j0 + 1] * a * (1 - b)
               + d[i0 + 1, j0] * (1 - a) * b + d[i0 + 1, j0 + 1] * a * b)
        return float(val.min())

    def check_path(self, path, start_index=0, horizon=5.0, skip=0.3, margin=None, allow_steep=False,
                   near_margin=None, near_radius=0.0):
        """Sweep the oriented footprint along the path. Return (index, class) of the
        first collision within `horizon` metres, or None.

        near_margin / near_radius: within `near_radius` m of path[start_index] the footprint is
        swept with `near_margin` instead of the full safety margin. This mirrors the A* rule that
        the safety band may only be entered right next to the robot (escape_radius): a robot that
        is already closer than the margin to an edge may leave without getting closer, but the
        body itself (margin 0) must never overlap a hazard. Callers pass the robot's CURRENT
        clearance level (start_clearance), so a route can never reduce clearance below
        min(safety_margin, current clearance)."""
        if path is None or len(path) < 2:
            return None
        travelled = 0.0
        last = start_index
        p0 = path[start_index]
        for k in range(start_index + 1, len(path)):
            travelled += math.hypot(*(path[k] - path[k - 1]))
            if travelled > horizon:
                break
            if travelled < skip or (travelled - (k - last) * 0 < 0):
                continue
            if k != len(path) - 1 and math.hypot(*(path[k] - path[last])) < 0.2:
                continue
            last = k
            k2 = min(len(path) - 1, k + 2)
            k1 = max(0, k - 2)
            yaw = math.atan2(path[k2][1] - path[k1][1], path[k2][0] - path[k1][0])
            ci = int(math.floor((path[k][1] - self.tp.map_origin_y) / self.res))
            cj = int(math.floor((path[k][0] - self.tp.map_origin_x) / self.res))
            if not allow_steep and 0 <= ci < self.steep.shape[0] and 0 <= cj < self.steep.shape[1] \
                    and self.steep[ci, cj] \
                    and math.hypot(*(path[k] - path[-1])) > 0.5:
                return k, STEEP
            m = margin
            if near_margin is not None and math.hypot(*(path[k] - p0)) <= near_radius:
                m = max(0.0, near_margin)
            wx, wy = self.footprint_points(path[k][0], path[k][1], yaw, margin=m)
            hit, cls = self.lethal_at(wx, wy)
            if hit.any():
                return k, int(cls[hit].max())
        return None

    def rotation_clear(self, x, y, yaw):
        tp = self.tp
        cx = x + math.cos(yaw) * tp.footprint_center_x
        cy = y + math.sin(yaw) * tp.footprint_center_x
        r = tp.circumscribed_radius
        a = np.arange(-r, r + 1e-6, self.res / 2)
        X, Y = np.meshgrid(a, a)
        m = X * X + Y * Y <= r * r
        hit, _ = self.lethal_at(cx + X[m], cy + Y[m], strict=1)
        return not hit.any()

    def forward_clear(self, x, y, yaw, distance):
        tp = self.tp
        front = tp.footprint_center_x + tp.robot_length / 2
        xs = np.arange(front, front + distance + 1e-6, self.res / 2)
        ys = np.arange(-tp.robot_width / 2, tp.robot_width / 2 + 1e-6, self.res / 2)
        X, Y = np.meshgrid(xs, ys)
        c, s = math.cos(yaw), math.sin(yaw)
        hit, cls = self.lethal_at(x + c * X - s * Y, y + s * X + c * Y)
        return (not hit.any()), (int(cls[hit].max()) if hit.any() else None)

    def rear_clear(self, x, y, yaw, distance):
        """True if reversing `distance` m keeps the footprint on OBSERVED, non-hazard ground (the
        camera looks forward, so unknown terrain behind the robot is treated as blocked)."""
        tp = self.tp
        rear = tp.footprint_center_x - tp.robot_length / 2
        xs = np.arange(rear - distance, rear + 1e-6, self.res / 2)
        ys = np.arange(-tp.robot_width / 2, tp.robot_width / 2 + 1e-6, self.res / 2)
        X, Y = np.meshgrid(xs, ys)
        c, s = math.cos(yaw), math.sin(yaw)
        hit, _ = self.lethal_at(x + c * X - s * Y, y + s * X + c * Y, strict=2)
        return not hit.any()

    def slope_at(self, x, y):
        if self.slope is None:
            return 0.0
        i = int(math.floor((y - self.tp.map_origin_y) / self.res))
        j = int(math.floor((x - self.tp.map_origin_x) / self.res))
        n = self.slope.shape[0]
        return float(self.slope[i, j]) if 0 <= i < n and 0 <= j < n else 0.0


def densify(pts, step):
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        L = math.hypot(*(b - a))
        n = max(1, int(math.ceil(L / step)))
        for t in np.arange(1, n + 1) / n:
            out.append(a + (b - a) * t)
    return np.array(out)


def path_length(path):
    return float(np.sum(np.hypot(*np.diff(path, axis=0).T))) if path is not None and len(path) > 1 else 0.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))




class NavigatorCore:
    """Goal-directed A-to-B state machine. The mission ends only at B.

    States shown on the dashboard:
      WAITING          settling after spawn
      NAVIGATING       following a normal path
      REPLANNING       (transient) new terrain blocked the path: HILL AHEAD / CLIFF AHEAD / PATH BLOCKED
      CLIMBING         on a hill (tilt or slope ahead above climb_display_deg) or on a climb-mode path
      CLIFF_AVOIDANCE  no route through known terrain: goal-seeking direction search along the hazard
      RECOVERY         reversing / turning after stuck or tilt limit, then replans
      GOAL_REACHED

    Escalation when the current path is blocked:
      1. local replan inside the A-B corridor          (mode 'normal')
      2. replan over the whole map                     (mode 'wide')
      3. hill-climbing replan: STEEP cells allowed at high cost, cliffs still hard (mode 'climb')
      4. CLIFF_AVOIDANCE: best short heading scored by progress to B, slope, revisits
      5. RECOVERY manoeuvre, then back to 1.

    Bounded failure handling (terminal states; the robot is commanded to zero velocity and stays
    stopped until a new goal arrives on /navigation/goal or /goal_pose):
      GOAL_UNREACHABLE        no route to B even through unexplored terrain (confirmed over time and
                              several map updates), or the route search / progress budget ran out
      RECOVERY_LIMIT_REACHED  more than max_recoveries recovery manoeuvres for this goal
      SENSOR_TIMEOUT          no terrain-map update for sensor_timeout s (HOLD at zero before that)
      LOCALIZATION_LOST       no valid pose for localization_timeout s (HOLD at zero before that)
    A terminal failure never reports GOAL_REACHED; GOAL_REACHED is decided only by distance to B.
    """

    TERMINAL = ('GOAL_REACHED', 'GOAL_UNREACHABLE', 'RECOVERY_LIMIT_REACHED', 'SENSOR_TIMEOUT',
                'LOCALIZATION_LOST')
    FAILURES = TERMINAL[1:]

    def __init__(self, tp, nav):
        self.tp, self.nav = tp, nav
        self.planner = Planner(tp, nav)
        self.goal = (nav.get('goal_x', 0.0), nav.get('goal_y', 0.0))
        self.start = None
        self.path = None
        self.previous_path = None
        self.progress = 0
        self.mode = 'normal'           # normal | climb | explore
        self.state = 'WAITING'
        self.display = ('WAITING', -1.0)
        self.events = deque(maxlen=30)
        self.t0 = None
        self.t = 0.0
        self.last_plan_t = -1e9
        self.map_dirty = False
        self.replan_reason = None      # (kind, label)
        self.reason = None             # last human-readable reason
        self.best_goal_dist = math.inf
        self.last_progress_t = 0.0
        self.replans = 0
        self.hazard_replans = 0
        self.path_version = 0
        self.last_block = None
        self.cmd = (0.0, 0.0)
        self.recovery = None
        self.move_ref = None           # (t, x, y) for the stuck detector
        self.tilt_flag = False
        self.climbing = False
        self.blocked_since = None
        self.avoid_until = -1.0        # show CLIFF_AVOIDANCE while on a cliff detour
        # --- replanning hysteresis / bounded failure bookkeeping ---
        self.map_version = 0           # incremented on every terrain-map update
        self.plan_map_version = -1     # map version the current plan was made on
        self.last_map_t = None         # time of the last terrain-map update (None = never)
        self.recoveries = 0            # recovery manoeuvres for the current goal
        self.explore_attempts = 0      # CLIFF_AVOIDANCE headings tried for the current goal
        self.no_route_since = None     # first time no route to B was found (cleared by a route)
        self.closed_since = None       # first time no route existed even through unknown terrain
        self.closed_map_version = None
        self.failure_reason = None
        self.goal_t = None             # time the current goal became active
        self.hold_reason = None
        self.first_pose_t = None       # time the first valid pose was seen
        self.fail_best_dist = None     # best distance to B for the no-progress failure budget
        self.best_progress_t = None
        # --- stopping distance (added after the Gazebo regression, see TECHNICAL_CHANGELOG) ---
        self.vel_est = (0.0, 0.0)      # estimated ACTUAL (v, w): drive.py ramps towards each command
        self.last_tick_t = None
        self.null_recoveries = 0       # consecutive recovery manoeuvres that could not move at all
        self.proven_cells = 0
        vres = 0.5
        self.vres = vres
        self.visits = np.zeros((int(tp.map_size / vres) + 1,) * 2, np.float32)

    # ----------------------------------------------------------- interface
    def set_goal(self, x, y, t=None):
        self.goal = (float(x), float(y))
        self.path = None
        self.mode = 'normal'
        self.best_goal_dist = math.inf
        self.last_progress_t = self.t
        self.hazard_replans = 0
        self.recovery = None
        self.visits[:] = 0
        if self.planner.penalty is not None:
            self.planner.penalty[:] = 0
        self.start = None  # new A = where the robot is now
        # a new goal restarts navigation after any terminal state, with fresh failure budgets
        self.recoveries = 0
        self.explore_attempts = 0
        self.no_route_since = None
        self.closed_since = None
        self.closed_map_version = None
        self.failure_reason = None
        self.hold_reason = None
        self.blocked_since = None
        self.move_ref = None
        self.replan_reason = None
        self.goal_t = self.t if t is None else t
        self.last_progress_t = self.goal_t
        self.fail_best_dist = None
        self.best_progress_t = self.goal_t
        if self.state in self.TERMINAL:
            self.state = 'NAVIGATING'
        self._event('NEW GOAL', f'B = ({x:.2f}, {y:.2f})')

    def update_map(self, classes, slope=None, rough=None, t=None):
        """t = time the map was produced/received (same clock as tick). None = 'now' (offline use)."""
        self.planner.build(classes, slope, rough)
        self.map_dirty = True
        self.map_version += 1
        self.last_map_t = t if t is not None else self.t

    def _fail(self, state, reason):
        """Enter a terminal failure state: zero velocity until a new goal is given."""
        if self.state == state:
            return 0.0, 0.0
        self.state = state
        self.failure_reason = reason
        self.recovery = None
        self.hold_reason = None
        self._event(state, reason, hold=0)
        return 0.0, 0.0

    def _event(self, state, msg, hold=2.0):
        self.events.appendleft(dict(t=round(self.t, 1), state=state, msg=msg))
        self.display = (state, self.t + hold)

    def display_state(self):
        if self.display[1] > self.t:
            return self.display[0]
        return self.state

    @staticmethod
    def _reason_for(cls):
        if cls in (CLIFF, POSSIBLE_DROP):
            return 'CLIFF AHEAD'
        if cls == STEEP:
            return 'HILL AHEAD'
        return 'PATH BLOCKED'

    # ---------------------------------------------------------------- tick
    def tick(self, t, pose, pose_t=None):
        """pose_t = time the pose was received (same clock as t); None = fresh (offline use).
        pose may be None if no valid pose has ever been received."""
        self.t = t
        if self.last_tick_t is not None:
            # the wheels move towards the previous command at most drive_*_accel (scripts/drive.py)
            self.vel_est = self._ramp(self.vel_est, self.cmd, max(0.0, min(0.5, t - self.last_tick_t)))
        self.last_tick_t = t
        v, w = self._tick(t, pose, pose_t)
        if not (math.isfinite(v) and math.isfinite(w)):
            v, w = 0.0, 0.0
        self.cmd = (v, w)
        return v, w

    def _ramp(self, cur, target, dt):
        nav = self.nav
        a, b = nav.get('drive_linear_accel', 0.5) * dt, nav.get('drive_angular_accel', 0.5) * dt
        return (cur[0] + max(-a, min(a, target[0] - cur[0])), cur[1] + max(-b, min(b, target[1] - cur[1])))

    def mark_proven(self, pose):
        """Record ground the robot is standing on (only while level: tilt < proven_max_tilt).
        Returns True if a hazard/unknown cell in the current map changed class (map rebuilt)."""
        pl, nav = self.planner, self.nav
        if pl.classes is None or pose.tilt_deg >= nav.get('proven_max_tilt', 10.0):
            return False
        shape = pl.raw[0].shape
        if pl.proven_seen is None or pl.proven_seen.shape != shape:
            pl.proven_seen = np.zeros(shape, bool)
            pl.proven_drop = np.zeros(shape, bool)
        n = shape[0]
        changed = False
        for mask, inset in ((pl.proven_seen, 0.0), (pl.proven_drop, nav.get('proven_drop_inset', 0.30))):
            wx, wy = pl.footprint_points(pose.x, pose.y, pose.yaw, margin=-inset)
            i = np.floor((wy - self.tp.map_origin_y) / pl.res).astype(int).ravel()
            j = np.floor((wx - self.tp.map_origin_x) / pl.res).astype(int).ravel()
            ok = (i >= 0) & (i < n) & (j >= 0) & (j < n)
            i, j = i[ok], j[ok]
            new = ~mask[i, j]
            if new.any():
                mask[i[new], j[new]] = True
                raw = pl.raw[0][i[new], j[new]]
                bad = (raw == UNKNOWN) if inset == 0.0 else np.isin(raw, (CLIFF, POSSIBLE_DROP))
                changed |= bool(bad.any())
        self.proven_cells = int(pl.proven_seen.sum())
        if changed:
            pl.build(*pl.raw)
        return changed

    def _stale_inputs(self, t, pose, pose_t):
        """Return (terminal_state, reason) or ('HOLD', reason) or None. Never moves the robot on
        stale or invalid inputs."""
        nav = self.nav
        pose_ok = pose is not None and all(math.isfinite(v) for v in (pose.x, pose.y, pose.z))
        if pose_ok and self.first_pose_t is None:
            self.first_pose_t = t
        if self.first_pose_t is None:
            # no pose yet (simulation / bridge still starting): wait, but not forever
            if t - self.t0 > nav.get('startup_timeout', 30.0):
                return 'LOCALIZATION_LOST', (f'no valid pose received {t - self.t0:.0f} s after start '
                                             f'(limit {nav.get("startup_timeout", 30.0):.0f} s) - stopped')
            return 'HOLD', 'WAITING FOR POSE - holding at zero velocity'
        pose_age = math.inf if not pose_ok else (0.0 if pose_t is None else t - pose_t)
        if pose_age > nav.get('pose_stale_time', 0.5):
            if pose_age > nav.get('localization_timeout', 3.0) and self._since_active(t) > \
                    nav.get('localization_timeout', 3.0):
                age = pose_age if math.isfinite(pose_age) else self._since_active(t)
                return 'LOCALIZATION_LOST', (f'no valid pose for {age:.1f} s '
                                             f'(limit {nav.get("localization_timeout", 3.0):.1f} s) - stopped')
            return 'HOLD', 'POSE STALE - holding at zero velocity'
        if self.last_map_t is not None:
            map_age = t - self.last_map_t
            if map_age > nav.get('map_stale_time', 1.0):
                if map_age > nav.get('sensor_timeout', 5.0):
                    return 'SENSOR_TIMEOUT', (f'no terrain-map update for {map_age:.1f} s '
                                              f'(limit {nav.get("sensor_timeout", 5.0):.1f} s) - stopped')
                return 'HOLD', f'TERRAIN DATA STALE ({map_age:.1f} s) - holding at zero velocity'
        return None

    def _since_active(self, t):
        ref = self.goal_t if self.goal_t is not None else self.t0
        return math.inf if ref is None else t - ref

    def _tick(self, t, pose, pose_t=None):
        nav, tp, pl = self.nav, self.tp, self.planner
        if self.t0 is None:
            self.t0 = t
        if self.goal_t is None:
            self.goal_t = t
        if self.first_pose_t is None and pose is not None and \
                all(math.isfinite(v) for v in (pose.x, pose.y, pose.z)):
            self.first_pose_t = t
        if self.state in self.TERMINAL:
            return 0.0, 0.0
        delay = nav.get('start_delay', 6.0)
        if t - self.t0 >= delay:
            stale = self._stale_inputs(t, pose, pose_t)
            if stale is not None:
                state, why = stale
                if state != 'HOLD':
                    return self._fail(state, why)
                if self.hold_reason != why.split(' (')[0]:
                    self._event('HOLD', why, hold=0.5)
                self.hold_reason = why.split(' (')[0]
                self.state = 'HOLD'
                self.blocked_since = None
                self.move_ref = None
                return 0.0, 0.0
            if self.hold_reason is not None:
                self._event('NAVIGATING', f'inputs fresh again after: {self.hold_reason}', hold=1.0)
                self.hold_reason = None
                self.last_progress_t = t   # time spent holding is not lack of progress
        elif pose is None:
            self.state = 'WAITING'
            return 0.0, 0.0
        if self.start is None:
            self.start = (pose.x, pose.y)
        if t - self.t0 < delay or pl.classes is None:
            if pl.classes is None and self.first_pose_t is not None and \
                    t - self.first_pose_t > nav.get('startup_timeout', 30.0):
                return self._fail('SENSOR_TIMEOUT', f'no terrain map received {t - self.first_pose_t:.0f} s after '
                                                    f'the first pose (limit {nav.get("startup_timeout", 30.0):.0f} s) '
                                                    f'- stopped')
            self.state = 'WAITING'
            return 0.0, 0.0
        gx, gy = self.goal
        dgoal = math.hypot(gx - pose.x, gy - pose.y)
        if dgoal < nav.get('goal_tolerance', 0.3):
            self.state = 'GOAL_REACHED'
            self._event('GOAL_REACHED', f'reached B, {dgoal:.2f} m from goal', hold=0)
            return 0.0, 0.0
        # B right next to a cliff/obstacle: the footprint can never sit exactly on B without
        # overhanging the hazard, so the closest safe point within goal_hazard_tolerance counts.
        near = self._goal_hazard_distance()
        if near is not None and near < tp.circumscribed_radius and dgoal < nav.get('goal_hazard_tolerance', 1.0):
            self.state = 'GOAL_REACHED'
            self._event('GOAL_REACHED', f'reached B ({dgoal:.2f} m): B is {near:.2f} m from a hazard edge, '
                                        f'stopped at the closest safe point', hold=0)
            return 0.0, 0.0
        self._visit(pose)
        self.mark_proven(pose)
        if pl.set_body_cells(pose.x, pose.y, pose.yaw):
            self.body_overlap_ticks = getattr(self, 'body_overlap_ticks', 0) + 1
            if not getattr(self, '_overlap_flag', False):
                self._overlap_flag = True
                self._event('REPLANNING', 'mapped hazard cell under the robot body (moving off it; new hazard '
                                          'cells stay forbidden)', hold=1.0)
        else:
            self._overlap_flag = False

        # meaningful progress for the failure budget: a net gain of progress_min_step m towards B
        if self.fail_best_dist is None or dgoal < self.fail_best_dist - nav.get('progress_min_step', 0.5):
            self.fail_best_dist = dgoal
            self.best_progress_t = t
        # bounded effort: no meaningful progress (best distance to B not improved) for too long
        ref = self.best_progress_t
        ref = self.goal_t if ref is None or (self.goal_t is not None and ref < self.goal_t) else ref
        limit = nav.get('no_progress_failure_time', 75.0)
        if limit > 0 and ref is not None and t - ref > limit:
            return self._fail('GOAL_UNREACHABLE', f'no meaningful progress towards B (>= '
                                                  f'{nav.get("progress_min_step", 0.5):.1f} m) for {t - ref:.0f} s '
                                                  f'(limit {limit:.0f} s, best {self.best_goal_dist:.1f} m from B)')

        # 0. recovery manoeuvre in progress
        if self.recovery is not None:
            return self._recover(t, pose)

        # 1. IMU / pose monitor while climbing
        tilt = pose.tilt_deg
        fx, fy = pose.x + math.cos(pose.yaw) * 1.0, pose.y + math.sin(pose.yaw) * 1.0
        if tilt >= tp.max_tilt_deg:
            pl.add_penalty(fx, fy, 0.9, 60.0)
            self._start_recovery(t, pose, f'TILT LIMIT {tilt:.0f} deg >= {tp.max_tilt_deg:.0f} - backing off')
            return self._recover(t, pose)
        warn = tp.max_tilt_deg - nav.get('tilt_margin_deg', 5.0)
        if tilt >= warn and not self.tilt_flag:
            self.tilt_flag = True
            pl.add_penalty(fx, fy, 0.8, 25.0)
            self.replan_reason = ('tilt', 'TILT LIMIT')
            self._event('REPLANNING', f'TILT LIMIT: pitch/roll {tilt:.0f} deg near limit - trying another climb line',
                        hold=1.5)
        elif tilt < warn - 3.0:
            self.tilt_flag = False

        # 2. new terrain: is the path ahead still traversable? (same clearance rule as planning)
        if self.map_dirty and self.path is not None:
            self.map_dirty = False
            block = pl.check_path(self.path, self.progress, horizon=nav.get('replanning_distance', 5.0),
                                  allow_steep=self.mode != 'normal', **self._near(pose))
            if block is not None:
                k, cls = block
                bx, by = self.path[k]
                d = path_length(self.path[self.progress:k + 1])
                label = self._reason_for(cls)
                self.last_block = (float(bx), float(by), CLASS_NAMES[cls])
                self._event('REPLANNING', f'{label}: {CLASS_NAMES[cls]} on path {d:.1f} m ahead '
                                          f'at ({bx:.1f}, {by:.1f})', hold=1.5)
                self.replan_reason = ('hazard', label)

        # 3. (re)plan - periodic replans also pull the robot back out of fallback modes.
        # Hysteresis: a hazard/blocked replan is only run when there is NEW terrain data since the
        # last plan (re-running A* on an unchanged map returns the same answer) or after
        # min_replan_interval. The pending reason is kept, so the replan happens on the next map.
        period = nav.get('replan_period', 4.0)
        explore_done = self.mode == 'explore' and self.path is not None and self.progress >= len(self.path) - 3
        fresh_map = self.map_version != self.plan_map_version
        reason_due = self.replan_reason is not None and (
            fresh_map or t - self.last_plan_t >= nav.get('min_replan_interval', 1.0)
            or self.replan_reason[0] in ('stuck', 'tilt'))
        if self.path is None or reason_due or explore_done or t - self.last_plan_t > period:
            self._plan(t, pose)
            if self.state in self.TERMINAL:
                return 0.0, 0.0
        if self.recovery is not None:
            return self._recover(t, pose)
        if self.path is None:  # should not happen (explore/recovery always produce motion)
            self._start_recovery(t, pose, 'PATH BLOCKED - no direction available')
            return self._recover(t, pose)

        # 4. hard hazard right in front: never drive into it, turn / recover instead
        ok, cls = pl.forward_clear(pose.x, pose.y, pose.yaw, nav.get('stop_distance', 0.45))
        if not ok:
            if self.replan_reason is None:
                self.replan_reason = ('hazard', self._reason_for(cls))
            if self.blocked_since is None:
                self.blocked_since = t
            if t - self.blocked_since > nav.get('blocked_timeout', 4.0):
                self.blocked_since = None
                pl.add_penalty(fx, fy, 0.8, 40.0)
                self._start_recovery(t, pose, f'{self._reason_for(cls)} directly in front - recovering')
                return self._recover(t, pose)
            v, w = self._shield(t, pose, *self._follow(pose, allow_forward=False))
            if self.state in self.TERMINAL or self.recovery is not None:
                return (0.0, 0.0) if self.recovery is None else self._recover(t, pose)
            return v, w
        self.blocked_since = None

        # 5. follow (speed reduced on hills and near hazard edges), through the motion shield
        v, w = self._shield(t, pose, *self._follow(pose))
        if self.state in self.TERMINAL:
            return 0.0, 0.0
        if self.recovery is not None:
            return self._recover(t, pose)

        # 6. stuck detector (commanded forward but not moving)
        if self.move_ref is None or math.hypot(pose.x - self.move_ref[1], pose.y - self.move_ref[2]) > 0.25:
            self.move_ref = (t, pose.x, pose.y)
        elif v > 0.05 and t - self.move_ref[0] > nav.get('stuck_timeout', 10.0):
            self.move_ref = None
            pl.add_penalty(fx, fy, 0.8, 40.0)
            self._start_recovery(t, pose, 'STUCK: wheels turning but no motion - recovering')
            return self._recover(t, pose)

        # 7. no progress towards B for a long time: new route, avoid revisiting
        if dgoal < self.best_goal_dist - 0.05:
            self.best_goal_dist = dgoal
            self.last_progress_t = t
        if t - self.last_progress_t > nav.get('progress_timeout', 40.0):
            self.last_progress_t = t
            self.replan_reason = ('stuck', 'PATH BLOCKED')
            self._event('REPLANNING', 'PATH BLOCKED: no progress towards B - trying another route')

        self._update_state(pose)
        return v, w

    def _motion_clear(self, pose, v, w, margin):
        """True if the robot can execute (v, w) for shield_horizon s AND then brake to a standstill
        with the oriented footprint (+margin) never touching a CLIFF/OBSTACLE cell (or leaving the
        map). The robot does not follow commands instantly (scripts/drive.py ramps the wheels at
        drive_linear_accel / drive_angular_accel): the prediction starts from the estimated actual
        speed, keeps the previous command for command_latency s, ramps to (v, w), holds it, then
        ramps to zero. Reversing / turning sweeps use the strict unknown-terrain rules
        (Planner.lethal_at)."""
        pl, nav = self.planner, self.nav
        # Inside the safety band (margin below shield_margin) the robot may only move so that its
        # continuous clearance does not shrink: the margin levels are 0.05 m apart, and a slow
        # motion could otherwise creep closer within one level, then within the next (ratchet).
        # Gazebo 25 Nov 20:54, too_narrow t1: a slow recovery reverse reached 0.000 m this way.
        c_now = None
        if margin < nav.get('shield_margin', 0.10) - 1e-9:
            c_now = pl.clearance_cont(pose.x, pose.y, pose.yaw)
            if not math.isfinite(c_now):
                c_now = None
        tol = nav.get('band_clearance_tolerance', 0.005)
        dt = 0.1
        hold = nav.get('shield_horizon', 0.6)
        lat = nav.get('command_latency', 0.15)
        x, y, yaw = pose.x, pose.y, pose.yaw
        cur = self.vel_est
        t, last_check = 0.0, -1.0
        while t < 6.0:
            target = self.cmd if t < lat else ((v, w) if t < lat + hold else (0.0, 0.0))
            cur = self._ramp(cur, target, dt)
            x += cur[0] * math.cos(yaw) * dt
            y += cur[0] * math.sin(yaw) * dt
            yaw += cur[1] * dt
            t += dt
            wx, wy = pl.footprint_points(x, y, yaw, margin=margin)
            strict = 2 if cur[0] < -1e-6 else (1 if abs(cur[0]) < 0.1 and abs(cur[1]) > 0.05 else 0)
            hit, _ = pl.lethal_at(wx, wy, strict=strict, drops=True)
            if hit.any():
                return False
            if c_now is not None and pl.clearance_cont(x, y, yaw) < c_now - tol:
                return False
            if t >= lat + hold and abs(cur[0]) < 1e-6 and abs(cur[1]) < 1e-6:
                break
        return True

    def _shield(self, t, pose, v, w):
        """Last-resort motion check applied to every path-following command (including turns in
        place). The body must keep at least min(shield_margin, current clearance) from any mapped
        CLIFF/OBSTACLE cell over the next shield_horizon s; otherwise try turning only, then stop.
        A veto that persists longer than blocked_timeout starts a (bounded) recovery."""
        nav, pl = self.nav, self.planner
        if (abs(v) < 1e-6 and abs(w) < 1e-6) or pl.lethal is None or \
                not (pl.lethal.any() or (pl.classes == POSSIBLE_DROP).any()):
            self.shield_since = None
            return v, w
        now_c = pl.start_clearance(pose.x, pose.y, pose.yaw, drops=True)
        margin = max(0.0, min(nav.get('shield_margin', 0.10), now_c))
        if self._motion_clear(pose, v, w, margin):
            self.shield_since = None
            return v, w
        self.shield_vetoes = getattr(self, 'shield_vetoes', 0) + 1
        out = (0.0, 0.0)
        # slower versions of the same command (shorter stopping distance), then turning only
        cands = [(v * f, w) for f in (0.5, 0.25) if abs(v) > 0.05] + ([(0.0, w)] if abs(w) > 1e-6 else [])
        for cv, cw in cands:
            if self._motion_clear(pose, cv, cw, margin):
                out = (cv, cw)
                break
        if out != (0.0, 0.0) and abs(out[0]) > 1e-6:
            self.shield_since = None      # still moving along the path, only slower
            return out
        # no safe motion: once the robot has actually come to rest and a NEW map still allows nothing,
        # waiting longer changes nothing - back off (bounded recovery) instead of standing there
        if out == (0.0, 0.0) and abs(self.vel_est[0]) < 0.02 and abs(self.vel_est[1]) < 0.02:
            if getattr(self, 'rest_since', None) is None:
                self.rest_since, self.rest_map = t, self.map_version
            elif t - self.rest_since >= nav.get('shield_rest_timeout', 0.5) and self.map_version > self.rest_map:
                self.rest_since = None
                self.shield_since = None
                self._penalise_here(pose)
                self._start_recovery(t, pose, 'SAFETY STOP: no safe motion along the path - recovering')
                return out
        else:
            self.rest_since = None
        if getattr(self, 'shield_since', None) is None:
            self.shield_since = t
            self._event('SAFETY STOP', f'motion shield: command ({v:+.2f} m/s, {w:+.2f} rad/s) would bring the '
                                       f'footprint within {margin:.2f} m of a hazard', hold=1.0)
            if self.replan_reason is None:
                self.replan_reason = ('hazard', 'PATH BLOCKED')
        elif t - self.shield_since > nav.get('blocked_timeout', 4.0) and out == (0.0, 0.0):
            self.shield_since = None
            self._penalise_here(pose)
            self._start_recovery(t, pose, 'SAFETY STOP: no safe motion along the path - recovering')
        return out

    def _penalise_here(self, pose):
        """The shield found no safe motion here: make the planner avoid routing through this spot
        again (same penalty as a hazard right in front), otherwise the route leads back to the same
        deadlock after every recovery until max_recoveries is used up."""
        fx, fy = pose.x + math.cos(pose.yaw) * 0.5, pose.y + math.sin(pose.yaw) * 0.5
        self.planner.add_penalty(fx, fy, 0.8, 40.0)

    def _near(self, pose):
        """check_path arguments for the robot's current clearance (see Planner.check_path)."""
        c = self.planner.start_clearance(pose.x, pose.y, pose.yaw)
        return dict(near_margin=max(0.0, c), near_radius=self.nav.get('escape_radius', 1.3))

    # ------------------------------------------------------------ planning
    def _plan(self, t, pose):
        nav, pl = self.nav, self.planner
        reason = self.replan_reason
        self.replan_reason = None
        self.last_plan_t = t
        self.plan_map_version = self.map_version
        start = (pose.x, pose.y)
        new, info, used = None, {}, None
        near = self._near(pose)
        astar_any = False
        for mode in ('normal', 'wide', 'climb'):
            new, info = pl.plan(start, self.goal, area=(self.start, self.goal), mode=mode,
                                escape_radius=near['near_radius'])
            astar_any |= new is not None
            if new is not None and pl.check_path(new, 0, horizon=nav.get('replanning_distance', 5.0),
                                                 allow_steep=mode == 'climb', **near) is not None:
                # the footprint sweep already rejects this route: do not adopt it (avoids a
                # replan loop on the same blocked path) - escalate to the next fallback instead
                new = None
            if new is not None:
                used = 'climb' if (mode == 'climb' and info.get('uses_steep')) else 'normal'
                break
        if new is None:
            if self._check_unreachable(t, astar_any):
                return
            self._explore_plan(t, pose, reason)
            return
        self.no_route_since = None
        self.closed_since = None
        old_mode = self.mode
        adopt = self.path is None or reason is not None or old_mode != used or old_mode == 'explore'
        if not adopt:
            rem = self.path[self.progress:]
            blocked = pl.check_path(self.path, self.progress, horizon=nav.get('replanning_distance', 5.0),
                                    allow_steep=self.mode != 'normal', **near)
            old_cost = pl.path_cost(rem) if len(rem) > 1 else math.inf
            adopt = blocked is not None or info['cost'] < 0.85 * old_cost
            if adopt and blocked is None:
                reason = ('improve', 'shorter/safer route with new terrain data')
        if not adopt:
            return
        first = self.path is None and self.replans == 0
        if self.path is not None and reason is not None and reason[0] in ('hazard', 'stuck', 'tilt'):
            self.previous_path = self.path.copy()
            self.hazard_replans += 1
        self.path, self.progress, self.mode = new, 0, used
        self.path_version += 1
        self.replans += 1
        L = path_length(new)
        if first:
            self._event('NAVIGATING', f'initial path to B: {L:.1f} m', hold=0)
        elif used == 'climb':
            self._event('CLIMBING', f'{(reason or ("", "HILL AHEAD"))[1]}: hill-climbing route over steep '
                                    f'terrain towards B, {L:.1f} m', hold=2.5)
        elif old_mode == 'explore':
            self._event('SAFE PATH FOUND', f'route to B found again: {L:.1f} m', hold=2.5)
        elif reason and reason[0] in ('hazard', 'tilt', 'stuck'):
            self._event('SAFE PATH FOUND', f'{reason[1]}: new route {L:.1f} m', hold=2.5)
        if reason and reason[1] == 'CLIFF AHEAD':
            self.avoid_until = t + self.nav.get('cliff_avoidance_display', 12.0)
        elif reason:
            self._event('PATH UPDATED', f'{reason[1]}: {L:.1f} m', hold=1.0)

    def _check_unreachable(self, t, astar_any):
        """Bounded route search. Returns True if a terminal failure was declared.

        * closed map: A* found NO route in any mode, even though UNKNOWN cells are traversable
          (optimistic). If that holds continuously for unreachable_confirm_time s AND across at
          least unreachable_confirm_maps new terrain maps, B is enclosed by hazards as far as the
          sensors can tell -> GOAL_UNREACHABLE. One failed attempt is never enough.
        * search budget: no adoptable route to B for route_search_timeout s of continuous search
          (heading search / recovery, while routes found by A* keep failing the footprint sweep)
          -> GOAL_UNREACHABLE."""
        nav = self.nav
        if self.no_route_since is None:
            self.no_route_since = t
        if not astar_any:
            if self.closed_since is None:
                self.closed_since, self.closed_map_version = t, self.map_version
            maps = self.map_version - self.closed_map_version
            if t - self.closed_since >= nav.get('unreachable_confirm_time', 10.0) and \
                    maps >= nav.get('unreachable_confirm_maps', 3):
                self._fail('GOAL_UNREACHABLE', f'no route to B through known or unexplored terrain for '
                                               f'{t - self.closed_since:.0f} s over {maps} map updates - stopped')
                return True
        else:
            self.closed_since = None
        limit = nav.get('route_search_timeout', 60.0)
        if limit > 0 and t - self.no_route_since >= limit:
            self._fail('GOAL_UNREACHABLE', f'no safe route to B found during {t - self.no_route_since:.0f} s of '
                                           f'searching (limit {limit:.0f} s) - stopped')
            return True
        return False

    def _explore_plan(self, t, pose, reason):
        """No route through the known map: pick the best short heading and go look."""
        seg, info = self._best_heading(pose)
        if seg is None:
            self._start_recovery(t, pose, 'PATH BLOCKED in every direction - recovering')
            return
        self.explore_attempts += 1
        if self.mode != 'explore' and self.path is not None:
            self.previous_path = self.path.copy()
        if self.mode != 'explore':
            self.hazard_replans += 1
        self.path, self.progress, self.mode = seg, 0, 'explore'
        self.path_version += 1
        self.replans += 1
        label = reason[1] if reason else 'PATH BLOCKED'
        self._event('CLIFF_AVOIDANCE', f'{label}: no route through known terrain - searching heading '
                                       f'{info["heading"]:.0f} deg (progress to B {info["progress"]:+.1f} m)',
                    hold=1.5)

    def _best_heading(self, pose):
        """Score 24 headings: strong weight on progress to B, then slope, revisits, turning."""
        nav, tp, pl = self.nav, self.tp, self.planner
        gx, gy = self.goal
        d0 = math.hypot(gx - pose.x, gy - pose.y)
        best, best_info = None, None
        near = self._near(pose)
        can_rotate = pl.rotation_clear(pose.x, pose.y, pose.yaw)
        for L in (nav.get('explore_step', 2.5), 1.2):
            for k in range(24):
                h = k * math.pi / 12
                c, s = math.cos(h), math.sin(h)
                # a heading that needs a turn in place is only usable where turning is clear
                if not can_rotate and abs(wrap(h - pose.yaw)) > nav.get('turn_in_place_error', 0.65):
                    continue
                ok = True
                slope_pen = 0.0
                for d in np.arange(0.4, L + 1e-6, 0.3):
                    x, y = pose.x + c * d, pose.y + s * d
                    i, j = pl.to_pcell(x, y)
                    if not pl._in(i, j) or pl.p_hard[i, j]:
                        ok = False
                        break
                    slope_pen += pl.slope_at(x, y) / tp.max_climb_slope_deg
                if not ok:
                    continue
                ex, ey = pose.x + c * L, pose.y + s * L
                seg = densify(np.array([(pose.x, pose.y), (ex, ey)], float), 0.1)
                # the whole segment is swept with the oriented footprint, using the SAME clearance
                # rule as A* routes (full margin, or the current clearance right next to the robot)
                if pl.check_path(seg, 0, horizon=L + 0.1, skip=0.0, **near) is not None:
                    continue
                progress = d0 - math.hypot(gx - ex, gy - ey)
                vi, vj = self._vcell(ex, ey)
                revisit = float(self.visits[vi, vj]) if 0 <= vi < self.visits.shape[0] and 0 <= vj < self.visits.shape[1] else 0.0
                i, j = pl.to_pcell(ex, ey)
                score = (nav.get('goal_progress_weight', 3.0) * progress - 0.5 * slope_pen
                         - nav.get('revisit_weight', 0.4) * revisit - float(pl.penalty[i, j]) * 0.05
                         - 0.3 * abs(wrap(h - pose.yaw)))
                if best is None or score > best[0]:
                    best = (score, ex, ey)
                    best_info = dict(heading=math.degrees(h), progress=progress)
            if best is not None:
                break
        if best is None:
            return None, {}
        return densify(np.array([(pose.x, pose.y), (best[1], best[2])], float), 0.1), best_info

    def _goal_hazard_distance(self):
        pl = self.planner
        if pl.lethal is None or not pl.lethal.any():
            return None
        i = int(math.floor((self.goal[1] - self.tp.map_origin_y) / pl.res))
        j = int(math.floor((self.goal[0] - self.tp.map_origin_x) / pl.res))
        n = pl.dist.shape[0]
        return float(pl.dist[i, j]) if 0 <= i < n and 0 <= j < n else None

    def _vcell(self, x, y):
        return (int((y - self.tp.map_origin_y) / self.vres), int((x - self.tp.map_origin_x) / self.vres))

    def _visit(self, pose):
        i, j = self._vcell(pose.x, pose.y)
        if 0 <= i < self.visits.shape[0] and 0 <= j < self.visits.shape[1]:
            self.visits[i, j] += 0.1  # seconds spent in this 0.5 m cell

    # ------------------------------------------------------------ recovery
    def _start_recovery(self, t, pose, why):
        pl = self.planner
        limit = self.nav.get('max_recoveries', 6)
        nl = self.nav.get('null_recovery_limit', 2)
        if nl > 0 and self.null_recoveries >= nl:
            self._fail('RECOVERY_LIMIT_REACHED', f'{why.split(" - ")[0]}; the last {self.null_recoveries} recovery '
                                                 f'manoeuvres could not move at all (no observed ground to reverse '
                                                 f'onto, no turn that keeps the safety margin) - stopped')
            return
        if limit >= 0 and self.recoveries >= limit:
            self._fail('RECOVERY_LIMIT_REACHED', f'{why.split(" - ")[0]}; {self.recoveries} recovery manoeuvres '
                                                 f'already used for this goal (limit {limit}) - stopped')
            return
        self.recoveries += 1
        seg, info = self._best_heading(pose)
        turn = 1.0
        if info:
            turn = 1.0 if wrap(math.radians(info['heading']) - pose.yaw) >= 0 else -1.0
        self.recovery = dict(phase='reverse', t0=t, x0=pose.x, y0=pose.y, yaw0=pose.yaw, turn=turn, why=why,
                             flipped=False)
        self.state = 'RECOVERY'
        self._event('RECOVERY', why, hold=1.0)

    def _recover(self, t, pose):
        r, nav, pl = self.recovery, self.nav, self.planner
        if r is None:                      # recovery refused (limit reached) -> terminal stop
            return 0.0, 0.0
        self.state = 'RECOVERY'
        # Recovery motions pass the same check as path following: the swept footprint incl. braking
        # must keep min(shield_margin, current clearance) from CLIFF / OBSTACLE / POSSIBLE_DROP cells
        # (Gazebo 25 Nov 20:54, too_narrow t1: a recovery turn allowed by the margin-free
        # rotation_clear swept a corner over the plateau edge).
        margin = max(0.0, min(nav.get('shield_margin', 0.10), pl.start_clearance(pose.x, pose.y, pose.yaw,
                                                                                drops=True)))
        if r['phase'] == 'reverse':
            moved = math.hypot(pose.x - r['x0'], pose.y - r['y0'])
            if moved < nav.get('recovery_reverse', 0.8) and t - r['t0'] < 8.0 \
                    and pl.rear_clear(pose.x, pose.y, pose.yaw, 0.3) \
                    and self._motion_clear(pose, -0.12, 0.0, margin):
                return -0.12, 0.0
            r['phase'], r['t0'] = 'turn', t
        if r['phase'] == 'turn' and t - r['t0'] < nav.get('recovery_turn_time', 2.5):
            for _ in range(2):
                w = 0.35 * r['turn']
                if self._motion_clear(pose, 0.0, w, margin):
                    return 0.0, w
                if r['flipped']:
                    break
                r['flipped'], r['turn'] = True, -r['turn']
        moved = math.hypot(pose.x - r['x0'], pose.y - r['y0'])
        turned = abs(wrap(pose.yaw - r['yaw0']))
        self.null_recoveries = self.null_recoveries + 1 if (moved < 0.05 and turned < 0.1) else 0
        self.recovery = None
        self.path = None
        self.mode = 'normal'
        self.last_progress_t = t
        self.replan_reason = ('stuck', 'RECOVERY DONE')
        return 0.0, 0.0

    # ----------------------------------------------------------- following
    def _follow(self, pose, allow_forward=True):
        nav, tp, pl = self.nav, self.tp, self.planner
        path = self.path
        lo, hi = self.progress, min(len(path), self.progress + 40)
        d = np.hypot(path[lo:hi, 0] - pose.x, path[lo:hi, 1] - pose.y)
        self.progress = lo + int(np.argmin(d))
        la = nav.get('lookahead', 0.8)
        k = self.progress
        acc = 0.0
        while k < len(path) - 1 and acc < la:
            acc += math.hypot(*(path[k + 1] - path[k]))
            k += 1
        tx, ty = path[k]
        self.target = (float(tx), float(ty))
        err = wrap(math.atan2(ty - pose.y, tx - pose.x) - pose.yaw)
        wmax = nav.get('max_angular', 0.35)
        w = max(-wmax, min(wmax, err))
        if abs(err) > nav.get('turn_in_place_error', 0.65):
            if pl.rotation_clear(pose.x, pose.y, pose.yaw):
                return 0.0, w
            ok, _ = pl.forward_clear(pose.x, pose.y, pose.yaw, 0.6)
            if ok and allow_forward:
                return 0.06, w
            return 0.0, w
        if not allow_forward:
            return 0.0, w
        # hill speed: slow down in proportion to current tilt / slope just ahead
        ahead = 0.0
        for q in path[self.progress:min(len(path), self.progress + 15)]:
            ahead = max(ahead, pl.slope_at(q[0], q[1]))
        self.climb_slope = max(pose.tilt_deg, ahead)
        lo_deg = nav.get('climb_display_deg', 15.0)
        f = 1.0
        if self.climb_slope > lo_deg:
            f = 1.0 - (self.climb_slope - lo_deg) / max(1.0, tp.max_climb_slope_deg - lo_deg) \
                * (1.0 - nav.get('climb_min_speed_factor', 0.45))
            f = max(nav.get('climb_min_speed_factor', 0.45), f)
        v = nav.get('cruise_speed', 0.18) * max(0.35, math.cos(err)) * f
        # near a CLIFF/OBSTACLE edge: slow down so the stopping distance plus camera/map latency
        # stays small compared with the remaining clearance (does not change any safety margin)
        band = nav.get('hazard_slow_band', 0.6)
        if band > 0 and pl.lethal is not None and pl.lethal.any():
            i = int(math.floor((pose.y - tp.map_origin_y) / pl.res))
            j = int(math.floor((pose.x - tp.map_origin_x) / pl.res))
            if 0 <= i < pl.dist.shape[0] and 0 <= j < pl.dist.shape[1]:
                d = float(pl.dist[i, j])
                lo, hi = tp.inscribed_radius, tp.circumscribed_radius + band
                fmin = nav.get('hazard_min_speed_factor', 0.5)
                self.hazard_speed_factor = float(np.clip(fmin + (1 - fmin) * (d - lo) / max(1e-6, hi - lo), fmin, 1.0))
                v *= self.hazard_speed_factor
        # do not outrun the map: the robot must be able to stop (perception + command latency, then
        # braking at drive_linear_accel) before its footprint reaches terrain that is not known to
        # be safe (unobserved, possible drop, cliff, obstacle). Never below creep_speed, so it can
        # start from a spawn pose whose surroundings the forward camera has not seen yet.
        cap = self._seen_speed_cap(pose)
        self.seen_speed_cap = cap
        v = min(v, cap)
        return v, w

    def _seen_speed_cap(self, pose):
        nav, tp, pl = self.nav, self.tp, self.planner
        a = nav.get('drive_linear_accel', 0.5)
        lat = nav.get('command_latency', 0.15) + nav.get('perception_latency', 0.3)
        vmax = nav.get('cruise_speed', 0.6)
        margin = nav.get('shield_margin', 0.10)
        reach = vmax * lat + vmax * vmax / (2 * a) + margin + 0.2
        front = tp.footprint_center_x + tp.robot_length / 2
        xs = np.arange(front, front + reach + 1e-6, pl.res / 2)
        hw = tp.robot_width / 2 + margin
        ys = np.arange(-hw, hw + 1e-6, pl.res / 2)
        X, Y = np.meshgrid(xs, ys)
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        wx, wy = pose.x + c * X - s * Y, pose.y + s * X + c * Y
        hit, _ = pl.lethal_at(wx, wy, strict=2)          # strict 2: unknown / possible drop block
        if not hit.any():
            return vmax
        d = float(X[hit].min()) - front - margin            # free, known-safe distance ahead
        if d <= 0:
            return nav.get('creep_speed', 0.15)
        # largest v with v * lat + v^2 / (2 a) <= d
        v = a * (-lat + math.sqrt(lat * lat + 2.0 * d / a))
        return max(nav.get('creep_speed', 0.15), min(vmax, v))

    def _update_state(self, pose):
        climb = getattr(self, 'climb_slope', 0.0) > self.nav.get('climb_display_deg', 15.0) or self.mode == 'climb'
        if climb and self.mode != 'explore':
            self.state = 'CLIMBING'
        elif self.mode == 'explore' or self.t < self.avoid_until:
            self.state = 'CLIFF_AVOIDANCE'
        else:
            self.state = 'NAVIGATING'

    def status(self, pose=None):
        d = dict(state=self.display_state(), raw_state=self.state, mode=self.mode,
                 goal=list(self.goal), start=list(self.start) if self.start else None,
                 replans=self.replans, hazard_replans=self.hazard_replans,
                 path_version=self.path_version,
                 path_length=round(path_length(self.path), 2),
                 cmd_v=round(self.cmd[0], 3), cmd_w=round(self.cmd[1], 3),
                 climb_slope=round(getattr(self, 'climb_slope', 0.0), 1),
                 target=getattr(self, 'target', None),
                 last_block=self.last_block, events=list(self.events)[:12],
                 terminal=self.state in self.TERMINAL, failure_reason=self.failure_reason,
                 recoveries=self.recoveries, explore_attempts=self.explore_attempts,
                 map_version=self.map_version,
                 map_age=None if self.last_map_t is None else round(self.t - self.last_map_t, 2),
                 no_route_for=None if self.no_route_since is None else round(self.t - self.no_route_since, 1),
                 no_progress_for=None if self.best_progress_t is None else round(self.t - self.best_progress_t, 1),
                 hold_reason=self.hold_reason, body_overlap_ticks=getattr(self, 'body_overlap_ticks', 0),
                 shield_vetoes=getattr(self, 'shield_vetoes', 0),
                 hazard_speed_factor=round(getattr(self, 'hazard_speed_factor', 1.0), 2),
                 vel_est=[round(self.vel_est[0], 3), round(self.vel_est[1], 3)],
                 null_recoveries=self.null_recoveries, proven_cells=self.proven_cells,
                 seen_speed_cap=round(getattr(self, 'seen_speed_cap', 0.0), 2))
        if self.events:
            d['reason'] = self.events[0]['msg']
        if self.failure_reason:
            d['reason'] = self.failure_reason
        if pose is not None:
            d['distance_to_goal'] = round(math.hypot(self.goal[0] - pose.x, self.goal[1] - pose.y), 2)
        return d
