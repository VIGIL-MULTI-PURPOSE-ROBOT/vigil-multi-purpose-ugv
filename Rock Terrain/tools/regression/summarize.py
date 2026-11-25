#!/usr/bin/env python3
"""Summarise a vigil_regression.sh run directory into Markdown (stdout).

    python3 summarize.py <run dir> [baseline dir] [offline status string]

Every number comes from the run's result.json files; missing values are printed as 'n/a', never
estimated. The baseline (25 Nov 2026 run) used monitor v1: its path length is NOT comparable (it
over-counted) and recoveries were not recorded, so those columns are marked as such.
"""
import json
import statistics as stats
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics import verdict, MIN_CLEARANCE  # noqa: E402

ORDER = ['cliff_front', 'flat', 'hill_choice', 'narrow_route', 'too_narrow', 'no_route']


def load(p):
    try:
        return json.loads(Path(p).read_text())
    except (OSError, ValueError):
        return None


def fmt(v, nd=1):
    if v is None:
        return 'n/a'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def agg(vals, nd=1):
    v = [x for x in vals if isinstance(x, (int, float))]
    if not v:
        return 'n/a'
    if len(v) == 1:
        return fmt(float(v[0]), nd)
    return f'{stats.mean(v):.{nd}f} ({min(v):.{nd}f}-{max(v):.{nd}f})'


def main():
    run = Path(sys.argv[1])
    base = Path(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] else None
    offline = sys.argv[3] if len(sys.argv) > 3 else 'n/a'
    r = lambda f: (run / f).read_text().strip() if (run / f).exists() else ''  # noqa: E731
    print('# VIGIL Rock Terrain regression run\n')
    print(f'- Code under test: `{r("code_id.txt")}`')
    print(f'- Environment: {" | ".join(r("environment.txt").splitlines())}')
    print(f'- Offline tests (0 = pass): {offline}')
    build = [ln for ln in r('notes.txt').splitlines() if ln.startswith('build exit code')]
    print(f'- Build: {build[0] if build else "n/a"}')
    print('- Collision instrumentation: **none** (minimum clearance is geometric: ground-truth pose vs. '
          'scenario geometry). No collision-free claim is made.')
    print(f'- Verdict rule: the expected outcome AND, in the cliff scenarios, a footprint clearance of at '
          f'least {MIN_CLEARANCE:.2f} m to the true drop edge (0.000 = the footprint overhung the edge). '
          f'Verdicts are recomputed with this rule; runs recorded under the older rule are marked *.\n')

    runs = []
    for d in sorted((run / 'runs').glob('*')):
        res = load(d / 'result.json') or {}
        res.setdefault('scenario', d.name.rsplit('_t', 1)[0])
        res.setdefault('trial', d.name.rsplit('_t', 1)[-1])
        if (d / 'superseded_by_retry').exists():
            res['superseded'] = True
        if res.get('category') is not None:
            v = verdict(res.get('category'), res.get('expected'), res.get('min_footprint_clearance_m'))
            if v != res.get('verdict'):
                res['verdict_recorded'] = res.get('verdict')
                res['verdict'] = v
        runs.append(res)
    print('## Every run\n')
    print('| Scenario | Trial | Category | Verdict | Final state | Failure reason | Goal time (sim s) | Sim s | Wall s | RTF '
          '| Path m | Replans | Hazard replans | Recoveries | Min clearance m | Max roll ° | Max pitch ° | Longest no-progress s '
          '| Depth Hz (sim) | cmd_vel Hz (sim) | Runtime errors | Shutdown msgs | Screenshots |')
    print('|' + '---|' * 23)
    for x in runs:
        shots = x.get('screenshots') or []
        shot_ok = sum(1 for s in shots if s.startswith('OK'))
        rh = x.get('rates_hz_sim') or {}
        star = ('*' if x.get('verdict_recorded') else '') + (' (superseded: retried)' if x.get('superseded') else '')
        print(f"| {x.get('scenario')} | {x.get('trial')} | {x.get('category')} | **{x.get('verdict')}**{star} | {x.get('raw_state')} | "
              f"{(x.get('failure_reason') or '')[:80]} | {fmt(x.get('goal_time_sim_s'))} | {fmt(x.get('sim_elapsed_s'))} | "
              f"{fmt(x.get('wall_elapsed_s'))} | {fmt(x.get('real_time_factor'), 3)} | {fmt(x.get('path_length_m'), 2)} | "
              f"{fmt(x.get('replans'))} | {fmt(x.get('hazard_replans'))} | {fmt(x.get('recoveries'))} | "
              f"{fmt(x.get('min_footprint_clearance_m'), 3)} | {fmt(x.get('max_roll_deg'))} | {fmt(x.get('max_pitch_deg'))} | "
              f"{fmt(x.get('longest_no_progress_s'))} | {fmt(rh.get('depth'))} | {fmt(rh.get('cmd'))} | "
              f"{fmt(x.get('runtime_error_lines'))} | {fmt(x.get('shutdown_message_lines'))} | {shot_ok}/{len(shots)} ok |")

    print('\n## Per scenario (mean (min-max) over trials)\n')
    print('| Scenario | Trials | PASS | Categories | Goal time s | Path m | Replans | Hazard replans | Recoveries '
          '| Min clearance m (worst) | Max roll ° | Max pitch ° | RTF |')
    print('|' + '---|' * 13)
    sup = [x for x in runs if x.get('superseded')]
    if sup:
        print('Simulator start-up failures that were retried (excluded below, listed above): '
              + ', '.join(f"{x.get('scenario')} t{x.get('trial')} ({x.get('category')})" for x in sup) + '\n')
    by = {}
    for x in runs:
        if x.get('superseded'):
            continue
        by.setdefault(x.get('scenario'), []).append(x)
    for sc in [s for s in ORDER if s in by] + [s for s in by if s not in ORDER]:
        xs = by[sc]
        cl = [x.get('min_footprint_clearance_m') for x in xs if x.get('min_footprint_clearance_m') is not None]
        cats = sorted({x.get('category') for x in xs})
        print(f"| {sc} | {len(xs)} | {sum(x.get('verdict') == 'PASS' for x in xs)}/{len(xs)} | {'; '.join(map(str, cats))} | "
              f"{agg([x.get('goal_time_sim_s') for x in xs])} | {agg([x.get('path_length_m') for x in xs], 2)} | "
              f"{agg([x.get('replans') for x in xs], 0)} | {agg([x.get('hazard_replans') for x in xs], 0)} | "
              f"{agg([x.get('recoveries') for x in xs], 0)} | {fmt(min(cl), 3) if cl else 'n/a'} | "
              f"{agg([x.get('max_roll_deg') for x in xs])} | {agg([x.get('max_pitch_deg') for x in xs])} | "
              f"{agg([x.get('real_time_factor') for x in xs], 3)} |")

    if base and (base / 'scenarios').exists():
        print(f'\n## Comparison with the baseline (`{base}`, original code, monitor v1, 1 trial)\n')
        print('| Scenario | Baseline outcome | Baseline goal s | Improved goal s (mean) | Baseline replans / hazard '
              '| Improved replans / hazard (mean) | Baseline min clearance m | Improved worst clearance m |')
        print('|' + '---|' * 8)
        for sc in ORDER:
            b = load(base / 'scenarios' / sc / 'result.json')
            xs = by.get(sc, [])
            if b is None and not xs:
                continue
            b = b or {}
            bout = 'GOAL_REACHED' if b.get('goal_reached') else f"no terminal state ({b.get('final_state')})"
            cl = [x.get('min_footprint_clearance_m') for x in xs if x.get('min_footprint_clearance_m') is not None]
            print(f"| {sc} | {bout} | {fmt(b.get('goal_reached_sim_s'))} | {agg([x.get('goal_time_sim_s') for x in xs])} | "
                  f"{fmt(b.get('replans'))} / {fmt(b.get('hazard_replans'))} | "
                  f"{agg([x.get('replans') for x in xs], 0)} / {agg([x.get('hazard_replans') for x in xs], 0)} | "
                  f"{fmt(b.get('min_footprint_clearance_to_drop_m'), 3)} | {fmt(min(cl), 3) if cl else 'n/a'} |")
        print('\nBaseline path length is not shown: monitor v1 over-counted it (see tools/regression/metrics.py). '
              'Baseline recoveries were not recorded.')

    print('\n## Git verification\n')
    b, a = run / 'git_before', run / 'git_after'
    for k in ['branch.txt', 'head.txt', 'main.txt', 'branches.txt', 'stash.txt', 'status_porcelain.txt',
              'diff_tracked.sha256', 'diff_files.sha256', 'diff_summary.txt', 'untracked.sha256']:
        if not ((b / k).exists() and (a / k).exists()):
            print(f'- {k}: missing (snapshot not taken)')
            continue
        tb, ta = (b / k).read_text(), (a / k).read_text()
        print(f'- {k}: {"unchanged during the run" if tb == ta else "CHANGED during the run"}')
        if k in ('untracked.sha256', 'diff_files.sha256') and tb != ta:   # name the files that changed
            lb, la = set(tb.splitlines()), set(ta.splitlines())
            names = sorted({ln.split('  ', 1)[-1] for ln in lb ^ la})
            for n in names:
                print(f'  - changed/added/removed: `{n}`')
    rd = lambda f: (a / f).read_text() if (a / f).exists() else ''  # noqa: E731
    print('\n```\n' + rd('branch.txt') + rd('status.txt') + rd('diff_check.txt') + '```'
          if a.exists() else '\n(no after-snapshot)')
    notes = r('notes.txt')
    if notes:
        print('\n## Notes\n\n```\n' + notes + '\n```')


if __name__ == '__main__':
    main()
