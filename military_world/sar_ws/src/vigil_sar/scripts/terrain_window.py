#!/usr/bin/python3
"""Windowed terrain analysis for large maps (vigil_sar addition).

terrain_core.TerrainMapper.analyze() re-classifies the WHOLE map every depth frame.
That is fine on the 32 m rough-terrain map (320 x 320 cells) but military_world needs
~170 m (1700 x 1700 cells): measured 1.1 s per frame, i.e. no 5 Hz loop.

New depth data, edge events and shadows can only change cells near the camera
(cliff_detection_distance + 2 m, shadows capped at 2 m behind an edge), so this
subclass runs the UNCHANGED terrain_core analysis on a window around the camera
(+ padding for the filter kernels) and writes back only the window interior.
The classification rules, thresholds and results are identical inside the window.
"""
import math


from terrain_core import TerrainMapper

_IN = ('hits', 'hsum', 'wsum', 'edge_drop', 'edge_theta', 'shadow_drop', 'shadow_theta', 'driven')
_OUT = ('height', 'drop', 'step', 'roughness', 'slope_deg', 'classes')


class _View:
    pass


class WindowedTerrainMapper(TerrainMapper):
    def __init__(self, p, window=12.0, pad=2.0):
        super().__init__(p)
        self.window = float(window)
        self.pad = float(pad)
        self._centre = None

    def process(self, depth, pose, cam, seg=None, rgb_shape=None):
        self._centre = (pose.x, pose.y)
        return super().process(depth, pose, cam, seg=seg, rgb_shape=rgb_shape)

    def analyze(self):
        if self._centre is None or self.window <= 0:
            return super().analyze()
        p = self.p
        res = p.resolution
        ci, cj = self.cell(self._centre[0], self._centre[1])
        r_in = int(math.ceil(self.window / res))
        r_pad = r_in + int(math.ceil(self.pad / res))
        n = self.n
        a0, a1 = max(0, ci - r_pad), min(n, ci + r_pad + 1)
        b0, b1 = max(0, cj - r_pad), min(n, cj + r_pad + 1)
        if a1 <= a0 or b1 <= b0:
            return self.classes
        v = _View()
        v.p = p
        for k in _IN:
            setattr(v, k, getattr(self, k)[a0:a1, b0:b1])
        TerrainMapper.analyze(v)            # unchanged rules on the window
        i0, i1 = max(0, ci - r_in), min(n, ci + r_in + 1)
        j0, j1 = max(0, cj - r_in), min(n, cj + r_in + 1)
        si, sj = slice(i0 - a0, i1 - a0), slice(j0 - b0, j1 - b0)
        for k in _OUT:
            getattr(self, k)[i0:i1, j0:j1] = getattr(v, k)[si, sj]
        return self.classes
