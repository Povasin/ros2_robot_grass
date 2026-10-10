#!/usr/bin/env python3
"""Scenario metrics and status transitions for actual ROS 2 live CSV data.
No models are executed or classifications invented: robot_status is read verbatim.
Requires Python 3.12 standard library only.
"""
from __future__ import annotations
import argparse
import csv
import math
from collections import Counter, defaultdict
from pathlib import Path

EXPECTED = {
    'rosbag2_езда': (1193, 'ROBOT DRIVING'),
    'rosbag2_застрял': (874, 'MOVEMENT_ON_PLACE'),
    'rosbag2_перевернулся': (563, 'ROTATED'),
}
STATES = ('ROBOT DRIVING', 'MOVEMENT_ON_PLACE', 'ROTATED')


def scenario_sanity(scenario, statuses, count, valid):
    """Shared capture rules for the report and validate_all.sh."""
    if scenario not in EXPECTED:
        return 'UNKNOWN'
    expected, target = EXPECTED[scenario]
    allowed = {'', 'ROBOT DRIVING', target}
    good = (count == expected and valid >= .95 * count
            and statuses.get(target, 0) > 0 and not (set(statuses) - allowed))
    return 'PASS' if good else 'REVIEW'

def num(raw):
    try:
        value = float(raw)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None

def pct(n, total):
    return round(100. * n / total, 2) if total else 0.

def first_time(rows, status):
    for row in rows:
        if row.get('robot_status') == status:
            return num(row.get('t_s'))
    return None

def first_sustained_speed(rows, threshold=0.1, duration=0.8):
    start = None
    for row in rows:
        t = num(row.get('t_s'))
        v = num(row.get('filtered_speed_xy'))
        if t is None or v is None or v >= threshold:
            start = None
            continue
        if start is None: start = t
        if t - start >= duration:
            return start
    return None

def p95_abs(rows, name):
    values = sorted(abs(v) for row in rows if (v := num(row.get(name))) is not None)
    if not values: return None
    return values[min(len(values) - 1, math.ceil(.95 * len(values)) - 1)]

def write_csv(path, cols, rows):
    with open(path,'w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=cols,extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)

def report(csv_file: Path, output_dir: Path):
    output_dir.mkdir(parents=True,exist_ok=True)
    with csv_file.open('r',newline='',encoding='utf-8-sig') as f:
        reader=csv.DictReader(f)
        columns=set(reader.fieldnames or ())
        necessary={'scenario','run_id','segment_id','t_s','robot_status',
                   'gps_x','filtered_x','filtered_y','tilt_deg','filtered_speed_xy',
                   'status_changed', 'gps_header_ns'}
        if not necessary <= columns:
            raise ValueError(f'This is not a live node CSV; missing: {sorted(necessary-columns)}')
        all_rows=list(reader)
    if not all_rows: raise ValueError('CSV is empty')

    groups=defaultdict(list)
    for row in all_rows:
        groups[(row['scenario'],row.get('run_id',''),row.get('segment_id','0'))].append(row)

    summary=[]; events=[]; warnings=[]
    for scenario in EXPECTED:
        count_total=sum(len(rs) for (s,_,_),rs in groups.items() if s==scenario)
        if not count_total:
            warnings.append(f'ОТСУТСТВУЕТ сценарий {scenario}')

    for (scenario,run_id,segment),rows in sorted(groups.items(),key=lambda p:(list(EXPECTED).index(p[0][0]) if p[0][0] in EXPECTED else 99,p[0][1],p[0][2])):
        # Keep capture order: sorting first would conceal timestamp regressions.
        expected,target=EXPECTED.get(scenario,(None,None))
        n=len(rows)
        times=[v for r in rows if (v:=num(r.get('t_s'))) is not None]
        duration=max(times)-min(times) if times else 0.
        status_cnt=Counter(r.get('robot_status') for r in rows)
        valid=sum(r.get('robot_status') in STATES for r in rows)
        first_target=first_time(rows,target) if target else None
        first110=next((v for r in rows if (v:=num(r.get('t_s'))) is not None and (a:=num(r.get('tilt_deg'))) is not None and a>=110),None)
        speed_start=first_sustained_speed(rows)
        max_tilt=max((v for r in rows if (v:=num(r.get('tilt_deg'))) is not None),default=None)
        low_tilt_rotated=sum(r.get('robot_status')=='ROTATED' and (v:=num(r.get('tilt_deg'))) is not None and v<95 for r in rows)
        missing_imu=sum(not r.get('q_w','') for r in rows)
        missing_vec=sum(not r.get('speed_vector_x','') for r in rows)
        fresh_imu=p95_abs(rows,'imu_time_diff_ms')
        fresh_v=p95_abs(rows,'speed_vector_time_diff_ms')
        gps_times=[int(r['gps_header_ns']) for r in rows]
        backward=sum(b<a for a,b in zip(gps_times,gps_times[1:]))
        duplicates=len(gps_times)-len(set(gps_times))
        unique_run_ids={r.get('run_id','') for r in rows}
        ratio=pct(n,expected) if expected else None
        sanity=scenario_sanity(scenario, status_cnt, n, valid)
        if expected and n != expected:
            sanity='REVIEW'
            warnings.append(f'{scenario}, run={run_id}, segment={segment}: GPS строк {n}, ожидалось {expected}')
        if valid==0:
            sanity='REVIEW'
            warnings.append(f'{scenario}: не опубликовано ни одного статуса')
        if backward:
            sanity='REVIEW'
            warnings.append(f'{scenario}: {backward} движений GPS header timestamp назад')
        if duplicates:
            sanity='REVIEW'
            warnings.append(f'{scenario}: повторных GPS timestamps {duplicates}')
        if sum(s == scenario for s, _, _ in groups) != 1:
            sanity='REVIEW'
        if status_cnt.get('ROTATED',0) and scenario!='rosbag2_перевернулся':
            warnings.append(f'{scenario}: неожиданные ROTATED: {status_cnt["ROTATED"]} GPS-отсчётов')
        if scenario=='rosbag2_езда' and status_cnt.get('MOVEMENT_ON_PLACE',0):
            warnings.append(f'{scenario}: неожиданные MOVEMENT_ON_PLACE: {status_cnt["MOVEMENT_ON_PLACE"]} GPS-отсчётов')
        if valid < .95*n:
            warnings.append(f'{scenario}: валидных статусов только {pct(valid,n):.1f}%')
        if len(unique_run_ids)!=1:
            warnings.append(f'{scenario}: внутри группы смешано несколько run_id')
        # One row per observed topic transition, preserve exact CSV timestamp.
        previous=None
        for r in rows:
            status=r.get('robot_status','')
            if status and status!=previous:
                t=num(r.get('t_s'))
                events.append({'scenario':scenario,'run_id':run_id,'segment_id':segment,
                              't_s':t if t is not None else '', 'robot_status':status,
                              'tilt_deg':r.get('tilt_deg',''),
                              'filtered_speed_xy':r.get('filtered_speed_xy',''),
                              'filtered_displacement_xy_08s':r.get('filtered_displacement_xy_08s','')})
            if status: previous=status
        summary.append(dict(scenario=scenario,run_id=run_id,segment_id=segment,gps_rows=n,
                            expected_gps_rows=expected or '',gps_coverage_pct=ratio if ratio is not None else '',
                            recorded_duration_sec=round(duration,3),valid_status_rows=valid,
                            status_valid_pct=pct(valid,n),driving_rows=status_cnt.get('ROBOT DRIVING',0),
                            stuck_rows=status_cnt.get('MOVEMENT_ON_PLACE',0),rotated_rows=status_cnt.get('ROTATED',0),
                            first_target_sec=first_target if first_target is not None else '',
                            imu_tilt_first_ge110_sec=first110 if first110 is not None else '',
                            rotation_detection_lag_sec=round(first_target-first110,3) if scenario=='rosbag2_перевернулся' and first_target is not None and first110 is not None else '',
                            stop_speed_sustained_start_sec=speed_start if speed_start is not None else '',
                            stop_detection_lag_sec=round(first_target-speed_start,3) if scenario=='rosbag2_застрял' and first_target is not None and speed_start is not None else '',
                            max_tilt_deg=round(max_tilt,3) if max_tilt is not None else '',
                            missing_imu_snapshots=missing_imu,missing_speed_vector_snapshots=missing_vec,
                            imu_age_abs_p95_ms=round(fresh_imu,2) if fresh_imu is not None else '',
                            speed_vector_age_abs_p95_ms=round(fresh_v,2) if fresh_v is not None else '',
                            backward_gps_header_timestamps=backward,
                            duplicate_gps_header_timestamps=duplicates,
                            rotated_with_tilt_lt95_rows=low_tilt_rotated,
                            scenario_sanity=sanity))
    for scenario in EXPECTED:
        count=sum(s==scenario for s,_,_ in groups)
        if count!=1:
            warnings.append(f'{scenario}: {count} сегментов/запусков, ожидался 1 — проверь --loop и процесс воспроизведения')

    write_csv(output_dir/'scenario_metrics.csv',list(summary[0].keys()),summary)
    write_csv(output_dir/'status_transitions.csv', ['scenario','run_id','segment_id','t_s','robot_status','tilt_deg','filtered_speed_xy','filtered_displacement_xy_08s'],events)
    print('Live CSV rows:',len(all_rows),'groups:',len(groups))
    for s in summary:
        print(f"  {s['scenario']}: {s['gps_rows']}/{s['expected_gps_rows']} GPS, valid={s['status_valid_pct']}%, "
              f"target={s['first_target_sec']}, result={s['scenario_sanity']}")
    print('Warnings:',len(warnings))
    for warning in warnings:
        print('  WARNING:',warning)
    print('Metrics:',output_dir/'scenario_metrics.csv')
    print('Events:',output_dir/'status_transitions.csv')
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('live_csv',type=Path)
    p.add_argument('--output-dir',type=Path,default=Path('validation_report'))
    a=p.parse_args()
    results=report(a.live_csv,a.output_dir)
    if (any(r['scenario_sanity'] != 'PASS' for r in results)
            or {r['scenario'] for r in results} != set(EXPECTED)):
        raise SystemExit(1)
