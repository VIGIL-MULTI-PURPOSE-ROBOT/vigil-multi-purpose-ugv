"""Localization error metrics for VIGIL GPS-denied evaluation (no ROS dependency).

Inputs are time-associated samples: for every ground-truth sample time t the estimate at the same
time (or None when no estimate was available). Poses are planar (x, y, yaw) plus z for reporting.

Definitions (see tools/vigil_eval/README.md):
  ATE_start   RMSE of position error after aligning the estimate to ground truth with the FIRST pair
              only (the robot starts at a known pose). This keeps the drift the navigator suffered.
  ATE_se2     RMSE after the best-fit SE(2) alignment of the whole trajectory (Umeyama, no scale).
              Reported for comparison with the literature; never used for acceptance.
  RPE(d)      translational error of relative motion over path segments of length d (metres of
              ground-truth travel), reported as RMSE, mean, median, p95 and max.
  drift_pct   final position error (start-aligned) / ground-truth path length x 100.
  yaw_err     heading error after start alignment, wrapped to [-180, 180) degrees: RMS and max.
  availability  fraction of ground-truth samples that had an estimate.
"""
import math

import numpy as np


def wrap(a):
    """Wrap angles (rad) to [-pi, pi)."""
    return (np.asarray(a, float) + math.pi) % (2 * math.pi) - math.pi


def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def to_mats(xyyaw):
    return [se2(*p) for p in xyyaw]


def path_length(xy):
    xy = np.asarray(xy, float)
    if len(xy) < 2:
        return 0.0
    return float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))


def start_align(gt, est):
    """Transform est so that its first pose equals gt's first pose. gt, est: (N, 3) x, y, yaw."""
    gt, est = np.asarray(gt, float), np.asarray(est, float)
    T = se2(*gt[0]) @ np.linalg.inv(se2(*est[0]))
    out = []
    for x, y, yaw in est:
        M = T @ se2(x, y, yaw)
        out.append((M[0, 2], M[1, 2], math.atan2(M[1, 0], M[0, 0])))
    return np.array(out)


def umeyama_se2(gt_xy, est_xy):
    """Best rigid 2D transform (R, t) minimising |gt - (R est + t)| (no scale)."""
    A, B = np.asarray(est_xy, float), np.asarray(gt_xy, float)
    ma, mb = A.mean(0), B.mean(0)
    H = (A - ma).T @ (B - mb)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(2)
    D[1, 1] = np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0
    R = Vt.T @ D @ U.T
    return R, mb - R @ ma


def stats(v):
    v = np.asarray(v, float)
    if v.size == 0:
        return None
    return dict(rmse=float(np.sqrt(np.mean(v ** 2))), mean=float(np.mean(v)), median=float(np.median(v)),
                p95=float(np.percentile(v, 95)), max=float(np.max(v)), n=int(v.size))


def rpe(gt, est, seg_m):
    """Relative pose error over ground-truth path segments of seg_m metres (translation, metres)."""
    gt, est = np.asarray(gt, float), np.asarray(est, float)
    if len(gt) < 2:
        return None
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(gt[:, :2], axis=0).T))])
    G, E = to_mats(gt), to_mats(est)
    errs, j = [], 0
    for i in range(len(gt)):
        if j < i:
            j = i
        while j < len(gt) and s[j] - s[i] < seg_m:
            j += 1
        if j >= len(gt):
            break
        dg = np.linalg.inv(G[i]) @ G[j]
        de = np.linalg.inv(E[i]) @ E[j]
        d = np.linalg.inv(dg) @ de
        errs.append(math.hypot(d[0, 2], d[1, 2]))
    return stats(errs)


def evaluate(samples, rpe_segments=(1.0, 2.0, 5.0)):
    """samples: list of dicts {t, gt: (x, y, z, yaw), est: (x, y, z, yaw) or None}, sorted by t.

    Returns a JSON-ready dict. Every metric that cannot be computed is None, never 0."""
    n = len(samples)
    out = dict(samples=n, with_estimate=0, availability=None, gt_path_length_m=None,
               gt_max_displacement_m=None, ate_start=None, ate_se2=None, rpe={}, drift_pct=None,
               final_error_m=None, yaw_err_deg=None, z_err=None, longest_gap_s=None)
    if n == 0:
        return out
    gt_all = np.array([s['gt'] for s in samples], float)
    out['gt_path_length_m'] = path_length(gt_all[:, :2])
    out['gt_max_displacement_m'] = float(np.max(np.hypot(*(gt_all[:, :2] - gt_all[0, :2]).T)))
    have = [s for s in samples if s.get('est') is not None]
    out['with_estimate'] = len(have)
    out['availability'] = len(have) / n
    # longest stretch of ground-truth time without an estimate
    gap, longest, last_t = 0.0, 0.0, None
    for s in samples:
        if last_t is not None and s.get('est') is None:
            gap += s['t'] - last_t
            longest = max(longest, gap)
        elif s.get('est') is not None:
            gap = 0.0
        last_t = s['t']
    out['longest_gap_s'] = longest
    if len(have) < 2:
        return out
    gt = np.array([(s['gt'][0], s['gt'][1], s['gt'][3]) for s in have], float)
    est = np.array([(s['est'][0], s['est'][1], s['est'][3]) for s in have], float)
    al = start_align(gt, est)
    e = np.hypot(*(al[:, :2] - gt[:, :2]).T)
    out['ate_start'] = stats(e)
    R, t = umeyama_se2(gt[:, :2], est[:, :2])
    e2 = np.hypot(*((est[:, :2] @ R.T + t) - gt[:, :2]).T)
    out['ate_se2'] = stats(e2)
    for d in rpe_segments:
        out['rpe'][f'{d:g}m'] = rpe(gt, est, d)
    L = path_length(gt[:, :2])
    out['final_error_m'] = float(e[-1])
    out['drift_pct'] = float(100.0 * e[-1] / L) if L > 0.5 else None
    yaw = np.degrees(np.abs(wrap(al[:, 2] - gt[:, 2])))
    out['yaw_err_deg'] = dict(rms=float(np.sqrt(np.mean(yaw ** 2))), max=float(np.max(yaw)))
    gz = np.array([s['gt'][2] for s in have]) - have[0]['gt'][2]
    ez = np.array([s['est'][2] for s in have]) - have[0]['est'][2]
    out['z_err'] = stats(np.abs(ez - gz))
    return out
