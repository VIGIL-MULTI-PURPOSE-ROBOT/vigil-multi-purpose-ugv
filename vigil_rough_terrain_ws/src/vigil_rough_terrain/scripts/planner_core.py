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

from terrain_core import (UNKNOWN, UNEVEN, POSSIBLE_DROP, CLASS_NAMES, CLIFF, OBSTACLE, STEEP, CLIMBABLE)

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

    # -------------------------------------------------------------- costmap
    def build(self, classes, slope=None, rough=None):
        """Cost map. Only CLIFF / OBSTACLE are inflated hard obstacles. Hills are cost:
        CLIMBABLE and slope/roughness add cost; STEEP (beyond the climbing limit) blocks the
        robot centre in normal mode and is only very expensive in climb mode."""
        tp, nav = self.tp, self.nav
        self.classes = classes
        lethal = np.isin(classes, (OBSTACLE, CLIFF))
        steep = classes == STEEP
        self.lethal = lethal
        self.steep = steep
        free = (~lethal).astype(np.uint8)
        if lethal.any():
            dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        else:
            dist = np.full(classes.shape, 1e3, np.float32)
        self.dist = dist
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

    def lethal_at(self, wx, wy):
        i = np.floor((wy - self.tp.map_origin_y) / self.res).astype(int)
        j = np.floor((wx - self.tp.map_origin_x) / self.res).astype(int)
        n = self.classes.shape[0]
        ok = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        out = np.zeros(wx.shape, bool)
        out[~ok] = True  # leaving the map is unsafe
        out[ok] = self.lethal[i[ok], j[ok]]
        cls = np.full(wx.shape, CLIFF, np.uint8)
        cls[ok] = self.classes[i[ok], j[ok]]
        return out, cls

    def check_path(self, path, start_index=0, horizon=5.0, skip=0.3, margin=None, allow_steep=False):
        """Sweep the oriented footprint along the path. Return (index, class) of the
        first collision within `horizon` metres, or None."""
        if path is None or len(path) < 2:
            return None
        travelled = 0.0
        last = start_index
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
            wx, wy = self.footprint_points(path[k][0], path[k][1], yaw, margin=margin)
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
        hit, _ = self.lethal_at(cx + X[m], cy + Y[m])
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
        """True if reversing `distance` m keeps the footprint off CLIFF/OBSTACLE."""
        tp = self.tp
        rear = tp.footprint_center_x - tp.robot_length / 2
        xs = np.arange(rear - distance, rear + 1e-6, self.res / 2)
        ys = np.arange(-tp.robot_width / 2, tp.robot_width / 2 + 1e-6, self.res / 2)
        X, Y = np.meshgrid(xs, ys)
        c, s = math.cos(yaw), math.sin(yaw)
        hit, _ = self.lethal_at(x + c * X - s * Y, y + s * X + c * Y)
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

    Escalation when the current path is blocked (never "stop and give up"):
      1. local replan inside the A-B corridor          (mode 'normal')
      2. replan over the whole map                     (mode 'wide')
      3. hill-climbing replan: STEEP cells allowed at high cost, cliffs still hard (mode 'climb')
      4. CLIFF_AVOIDANCE: best short heading scored by progress to B, slope, revisits
      5. RECOVERY manoeuvre, then back to 1.
    """

    TERMINAL = ('GOAL_REACHED',)

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
        if self.state in self.TERMINAL:
            self.state = 'NAVIGATING'
        self._event('NEW GOAL', f'B = ({x:.2f}, {y:.2f})')

    def update_map(self, classes, slope=None, rough=None):
        self.planner.build(classes, slope, rough)
        self.map_dirty = True

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
    def tick(self, t, pose):
        self.t = t
        v, w = self._tick(t, pose)
        self.cmd = (v, w)
        return v, w

    def _tick(self, t, pose):
        nav, tp, pl = self.nav, self.tp, self.planner
        if self.t0 is None:
            self.t0 = t
        if self.start is None:
            self.start = (pose.x, pose.y)
        if t - self.t0 < nav.get('start_delay', 6.0) or pl.classes is None:
            self.state = 'WAITING'
            return 0.0, 0.0
        if self.state in self.TERMINAL:
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

        # 2. new terrain: is the path ahead still traversable?
        if self.map_dirty and self.path is not None:
            self.map_dirty = False
            block = pl.check_path(self.path, self.progress, horizon=nav.get('replanning_distance', 5.0),
                                  allow_steep=self.mode != 'normal')
            if block is not None:
                k, cls = block
                bx, by = self.path[k]
                d = path_length(self.path[self.progress:k + 1])
                label = self._reason_for(cls)
                self.last_block = (float(bx), float(by), CLASS_NAMES[cls])
                self._event('REPLANNING', f'{label}: {CLASS_NAMES[cls]} on path {d:.1f} m ahead '
                                          f'at ({bx:.1f}, {by:.1f})', hold=1.5)
                self.replan_reason = ('hazard', label)

        # 3. (re)plan - periodic replans also pull the robot back out of fallback modes
        period = nav.get('replan_period', 4.0)
        explore_done = self.mode == 'explore' and self.path is not None and self.progress >= len(self.path) - 3
        if self.path is None or self.replan_reason or explore_done or t - self.last_plan_t > period:
            self._plan(t, pose)
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
            return self._follow(pose, allow_forward=False)
        self.blocked_since = None

        # 5. follow (speed reduced on hills)
        v, w = self._follow(pose)

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
        elif t - self.last_progress_t > nav.get('progress_timeout', 40.0):
            self.last_progress_t = t
            self.replan_reason = ('stuck', 'PATH BLOCKED')
            self._event('REPLANNING', 'PATH BLOCKED: no progress towards B - trying another route')

        self._update_state(pose)
        return v, w

    # ------------------------------------------------------------ planning
    def _plan(self, t, pose):
        nav, pl = self.nav, self.planner
        reason = self.replan_reason
        self.replan_reason = None
        self.last_plan_t = t
        start = (pose.x, pose.y)
        new, info, used = None, {}, None
        for mode in ('normal', 'wide', 'climb'):
            new, info = pl.plan(start, self.goal, area=(self.start, self.goal), mode=mode)
            if new is not None and pl.check_path(new, 0, horizon=nav.get('replanning_distance', 5.0),
                                                 allow_steep=mode == 'climb') is not None:
                # the footprint sweep already rejects this route: do not adopt it (avoids a
                # replan loop on the same blocked path) - escalate to the next fallback instead
                new = None
            if new is not None:
                used = 'climb' if (mode == 'climb' and info.get('uses_steep')) else 'normal'
                break
        if new is None:
            self._explore_plan(t, pose, reason)
            return
        old_mode = self.mode
        adopt = self.path is None or reason is not None or old_mode != used or old_mode == 'explore'
        if not adopt:
            rem = self.path[self.progress:]
            blocked = pl.check_path(self.path, self.progress, horizon=nav.get('replanning_distance', 5.0),
                                    allow_steep=self.mode != 'normal')
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

    def _explore_plan(self, t, pose, reason):
        """No route through the known map: pick the best short heading and go look."""
        seg, info = self._best_heading(pose)
        if seg is None:
            self._start_recovery(t, pose, 'PATH BLOCKED in every direction - recovering')
            return
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
        for L in (nav.get('explore_step', 2.5), 1.2):
            for k in range(24):
                h = k * math.pi / 12
                c, s = math.cos(h), math.sin(h)
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
                wx, wy = pl.footprint_points(ex, ey, h, margin=0.0)
                hit, _ = pl.lethal_at(wx, wy)
                if hit.any():
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
        seg, info = self._best_heading(pose)
        turn = 1.0
        if info:
            turn = 1.0 if wrap(math.radians(info['heading']) - pose.yaw) >= 0 else -1.0
        self.recovery = dict(phase='reverse', t0=t, x0=pose.x, y0=pose.y, turn=turn, why=why)
        self.state = 'RECOVERY'
        self._event('RECOVERY', why, hold=1.0)

    def _recover(self, t, pose):
        r, nav, pl = self.recovery, self.nav, self.planner
        self.state = 'RECOVERY'
        if r['phase'] == 'reverse':
            moved = math.hypot(pose.x - r['x0'], pose.y - r['y0'])
            if moved < nav.get('recovery_reverse', 0.8) and t - r['t0'] < 8.0 \
                    and pl.rear_clear(pose.x, pose.y, pose.yaw, 0.3):
                return -0.12, 0.0
            r['phase'], r['t0'] = 'turn', t
        if r['phase'] == 'turn':
            if t - r['t0'] < nav.get('recovery_turn_time', 2.5) and pl.rotation_clear(pose.x, pose.y, pose.yaw):
                return 0.0, 0.35 * r['turn']
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
        return v, w

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
                 last_block=self.last_block, events=list(self.events)[:12])
        if self.events:
            d['reason'] = self.events[0]['msg']
        if pose is not None:
            d['distance_to_goal'] = round(math.hypot(self.goal[0] - pose.x, self.goal[1] - pose.y), 2)
        return d
