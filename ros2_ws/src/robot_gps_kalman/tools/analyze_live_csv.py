from __future__ import annotations

import argparse
import csv
import math
from collections import Counter, defaultdict
from pathlib import Path

STATUS_CODE = {'ROBOT DRIVING': 0, 'MOVEMENT_ON_PLACE': 1, 'ROTATED': 2}
STATUS_NAMES = ['ROBOT DRIVING', 'MOVEMENT_ON_PLACE', 'ROTATED']


def number(s):
    try:
        n = float(s)
        return n if math.isfinite(n) else None
    except (TypeError, ValueError):
        return None


def seq(rows, key):
    return [number(row.get(key, '')) if number(row.get(key, '')) is not None else math.nan for row in rows]


def transitions_for(rows):
    return [(number(row['t_s']), row['robot_status']) for row in rows
            if row.get('status_changed') == '1' and row.get('robot_status')]


def _mark_transitions(ax, transitions, annotate=False):
    for t, status in transitions:
        if t is None:
            continue
        ax.axvline(t, color='#6750A4', lw=1, ls='--', alpha=.58)
        if annotate:
            ax.annotate(status, (t, 1), xycoords=('data', 'axes fraction'),
                        xytext=(4, -5), textcoords='offset points',
                        va='top', ha='left', fontsize=8, rotation=90, color='#503b75')


def _setup_plot_lib():
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError('Install Matplotlib: sudo apt install python3-matplotlib') from exc
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'axes.titlesize': 12,
                         'axes.labelsize': 10, 'figure.facecolor': 'white',
                         'axes.spines.top': False, 'axes.spines.right': False})
    return plt


def plot_one(scenario, segment, rows, out_dir, rotated_enter=110.,
             rotated_exit=95., stop_speed=.10, stop_disp=.25):
    plt = _setup_plot_lib()
    out_dir.mkdir(parents=True, exist_ok=True)
    name = ''.join(c if c.isalnum() or c in '-_' else '_' for c in scenario)
    prefix = out_dir / f'{name}_segment_{segment}'
    t = seq(rows, 't_s')
    tr = transitions_for(rows)
    fig, axs = plt.subplots(4, 1, figsize=(13, 11), sharex=True,
                            gridspec_kw={'height_ratios': [1.4, 1.4, 1.25, .75]})
    fig.suptitle(f'{scenario} · запуск/сегмент {segment}\n'
                 'Состояние, IMU, движение и решения ноды во времени',
                 fontweight='bold', fontsize=15)

    ax = axs[0]
    ax.plot(t, seq(rows, 'tilt_deg'), lw=1.7, color='#15803d', label='Наклон IMU')
    ax.axhline(rotated_enter, ls='--', lw=1.1, color='#dc2626', label=f'Переворот: {rotated_enter:g}°')
    ax.axhline(rotated_exit, ls=':', lw=1.1, color='#be123c', label=f'Возврат: {rotated_exit:g}°')
    ax.set_ylabel('Наклон, °'); ax.set_ylim(bottom=0); ax.legend(loc='upper left', ncol=3, fontsize=9)
    ax.grid(alpha=.22); _mark_transitions(ax, tr)

    ax = axs[1]
    ax.plot(t, seq(rows, 'filtered_speed_xy'), color='#2563eb', lw=1.5,
            label='Скорость Калмана XY')
    ax.plot(t, seq(rows, 'speed'), color='#9ca3af', lw=1, alpha=.9,
            label='Скалярная скорость (топик)')
    ax.axhline(stop_speed, color='#ea580c', ls='--', lw=1, label=f'Порог остановки: {stop_speed:g}')
    ax.set_ylabel('Скорость, ед./с'); ax.set_ylim(bottom=0)
    ax.legend(loc='upper right', ncol=3, fontsize=9); ax.grid(alpha=.22)
    _mark_transitions(ax, tr)

    ax = axs[2]
    ax.plot(t, seq(rows, 'filtered_displacement_xy_08s'), color='#7c3aed', lw=1.7,
            label='Смещение Калмана XY за 0,8 с')
    ax.axhline(stop_disp, color='#ea580c', ls='--', lw=1, label=f'Порог: {stop_disp:g}')
    ax.set_ylabel('Смещение, ед.'); ax.set_ylim(bottom=0)
    ax.legend(loc='upper right', fontsize=9); ax.grid(alpha=.22)
    _mark_transitions(ax, tr)

    ax = axs[3]
    states = [STATUS_CODE.get(r.get('robot_status', '')) for r in rows]
    ax.step(t, [v if v is not None else math.nan for v in states], where='post',
            color='#0f172a', lw=2)
    ax.set_yticks([0, 1, 2], STATUS_NAMES)
    ax.set_ylim(-.35, 2.35)
    ax.grid(axis='x', alpha=.25); _mark_transitions(ax, tr, annotate=True)
    ax.set_xlabel('Время от начала сегмента, с')
    fig.align_ylabels(axs)
    fig.tight_layout(rect=(0, 0, 1, .935))
    overview = Path(str(prefix) + '_overview.png')
    fig.savefig(overview, dpi=155); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(seq(rows, 'gps_x'), seq(rows, 'gps_y'),
            lw=.9, alpha=.48, label='Сырой GPS', color='#60a5fa')
    ax.plot(seq(rows, 'filtered_x'), seq(rows, 'filtered_y'),
            lw=2, label='Калман GPS + speed_vector', color='#ea580c')
    ax.set_xlabel('X, единицы GPS'); ax.set_ylabel('Y, единицы GPS')
    ax.set_title(f'{scenario} · сегмент {segment} · траектория XY')
    ax.set_aspect('equal', adjustable='datalim'); ax.margins(.09)
    ax.grid(alpha=.25); ax.legend()
    fig.tight_layout()
    trajectory = Path(str(prefix) + '_xy.png')
    fig.savefig(trajectory, dpi=155); plt.close(fig)

    print(f'  Saved: {overview.name}, {trajectory.name}')
    return overview, trajectory


def plot_scenario_comparison(groups, out_dir, rotated_enter=110., stop_speed=.10):
    """Side-by-side comparison of the largest run for each scenario."""
    plt = _setup_plot_lib()
    by_scenario = {}
    for (scenario, run_id, segment), rows in groups.items():
        if scenario not in by_scenario or len(rows) > len(by_scenario[scenario]):
            by_scenario[scenario] = rows
    if len(by_scenario) < 2:
        return None
    ordered = [s for s in ('rosbag2_езда', 'rosbag2_застрял', 'rosbag2_перевернулся')
               if s in by_scenario]
    ordered += [s for s in by_scenario if s not in ordered]
    n = len(ordered)
    fig, axes = plt.subplots(3, n, figsize=(5.1 * n, 9), sharex='col',
                             gridspec_kw={'height_ratios':[1.3,1.3,.8]},
                             squeeze=False)
    fig.suptitle('Сравнение сценариев: ориентация, скорость и опубликованный статус',
                 fontweight='bold', fontsize=14)
    for col, scenario in enumerate(ordered):
        rows = by_scenario[scenario]
        t = seq(rows, 't_s')
        transitions = transitions_for(rows)
        ax = axes[0, col]
        ax.plot(t, seq(rows, 'tilt_deg'), color='#15803d', lw=1.6)
        ax.axhline(rotated_enter, color='#dc2626', ls='--', lw=1)
        ax.set_ylim(bottom=0)
        ax.set_title(scenario, fontweight='bold')
        ax.grid(alpha=.25)
        if col == 0: ax.set_ylabel('IMU: наклон, °')
        _mark_transitions(ax, transitions)
        ax = axes[1, col]
        ax.plot(t, seq(rows, 'filtered_speed_xy'), color='#2563eb', lw=1.5)
        ax.axhline(stop_speed, color='#ea580c', ls='--', lw=1)
        ax.set_ylim(bottom=0); ax.grid(alpha=.25)
        if col == 0: ax.set_ylabel('Калман: скорость XY')
        _mark_transitions(ax, transitions)
        ax = axes[2, col]
        statuses = [STATUS_CODE.get(row.get('robot_status', '')) for row in rows]
        ax.step(t, [v if v is not None else math.nan for v in statuses],
                where='post', lw=1.8, color='#0f172a')
        ax.set_ylim(-.4, 2.4)
        ax.set_yticks([0,1,2], STATUS_NAMES)
        ax.grid(axis='x', alpha=.25)
        ax.set_xlabel('Время, с')
        _mark_transitions(ax, transitions)
    fig.tight_layout(rect=(0, 0, 1, .94))
    out_dir.mkdir(parents=True, exist_ok=True)
    result = out_dir/'all_scenarios_comparison.png'
    fig.savefig(result, dpi=150)
    plt.close(fig)
    print(f'Saved side-by-side plot: {result}')
    return result


def describe(path: Path, plots=False, out_dir=None, rotated_enter=110., rotated_exit=95.,
             stop_speed=.10, stop_disp=.25):
    with path.open(encoding='utf-8-sig', newline='') as f:
        reader = csv.DictReader(f)
        needed = {'scenario', 'segment_id', 't_s', 'robot_status', 'status_changed'}
        if not reader.fieldnames or not needed.issubset(reader.fieldnames):
            raise ValueError('This is not a live-node telemetry CSV; please check the input path.')
        records = list(reader)
    if not records:
        raise ValueError('CSV has no samples. Run ros2 bag play while the node records CSV.')
    groups = defaultdict(list)
    for r in records:
        groups[(r['scenario'], r.get('run_id', ''), r['segment_id'])].append(r)
    print(f'CSV: {path} | total rows: {len(records)} | scenario/segment groups: {len(groups)}')
    for (scenario, run_id, segment), rows in groups.items():
        times = [number(row['t_s']) for row in rows]
        ok_t = [t for t in times if t is not None]
        counts = Counter(row['robot_status'] or 'AWAITING_DATA' for row in rows)
        changes = transitions_for(rows)
        backwards = sum(a is not None and b is not None and b < a
                        for a, b in zip(times, times[1:]))
        missing_imu = sum(not row.get('imu_header_ns') for row in rows)
        missing_velocity = sum(not row.get('speed_vector_x') for row in rows)
        print(f'\n{scenario} | run={run_id or "unknown"} | segment={segment} | samples={len(rows)}')
        if ok_t:
            print(f'  Time: {min(ok_t):.3f} .. {max(ok_t):.3f} s')
        print(f'  Counts: {dict(counts)}')
        if changes:
            print('  Transitions: ' + '; '.join(f'{t:.3f}s → {status}' for t, status in changes))
        else:
            print('  Transitions: none')
        print(f'  Missing IMU/velocity snapshots: {missing_imu}/{missing_velocity}; '
              f'backward-timestamp rows: {backwards}')
        if plots:
            plot_one(scenario, f'{run_id + "_" if run_id else ""}{segment}', rows, out_dir or path.parent,
                     rotated_enter=rotated_enter, rotated_exit=rotated_exit,
                     stop_speed=stop_speed, stop_disp=stop_disp)
    if plots:
        plot_scenario_comparison(groups, out_dir or path.parent,
                                 rotated_enter=rotated_enter, stop_speed=stop_speed)
    return groups


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_path', type=Path)
    parser.add_argument('--plots', action='store_true', help='Save two PNGs per scenario/segment')
    parser.add_argument('--output-dir', type=Path, default=None)
    parser.add_argument('--rotated-enter', type=float, default=110.)
    parser.add_argument('--rotated-exit', type=float, default=95.)
    parser.add_argument('--stop-speed', type=float, default=.1)
    parser.add_argument('--stop-disp', type=float, default=.25)
    args = parser.parse_args()
    describe(args.csv_path, plots=args.plots, out_dir=args.output_dir,
             rotated_enter=args.rotated_enter, rotated_exit=args.rotated_exit,
             stop_speed=args.stop_speed, stop_disp=args.stop_disp)
