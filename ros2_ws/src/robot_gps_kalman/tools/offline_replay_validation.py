from __future__ import annotations

import argparse
import csv
import math
from collections import Counter
from pathlib import Path

from robot_gps_kalman.kalman_core import KalmanGPSVelocity
from robot_gps_kalman.status_core import RobotStatusEstimator


def finite_vec(row, names):
    try:
        out = [float(row[name]) for name in names]
    except (ValueError, TypeError, KeyError):
        return None
    return out if all(math.isfinite(v) for v in out) else None


def validate(csv_file, output_csv):
    rows = list(csv.DictReader(open(csv_file, encoding='utf-8-sig', newline='')))
    report = {}
    output = []
    scenarios = list(dict.fromkeys(r['scenario'] for r in rows))
    for scenario in scenarios:
        data = [r for r in rows if r['scenario'] == scenario]
        est = RobotStatusEstimator()
        kf = KalmanGPSVelocity()
        last_t = None
        transitions = []
        last_status = None
        counters = Counter()
        for r in data:
            t = float(r['t_s'])
            p = finite_vec(r, ('gps_x','gps_y','gps_z'))
            v = finite_vec(r, ('speed_vector_x','speed_vector_y','speed_vector_z'))
            q = finite_vec(r, ('q_x','q_y','q_z','q_w'))
            if p is None or q is None:
                continue
            if not kf.initialized:
                kf.initialize(p, v)
            else:
                if last_t is not None and t > last_t:
                    kf.predict(t-last_t)
                if v is not None:
                    kf.update_velocity(v)
                kf.update_gps(p)
            last_t = t
            est.update_imu(q, t)
            est.update_motion(kf.position, kf.velocity, t)
            state = est.current_status(t)
            counters[state or 'AWAITING_DATA'] += 1
            if state != last_status:
                transitions.append((round(t, 3), state or 'AWAITING_DATA'))
                last_status = state
            output.append({
                'scenario': scenario, 't_s': round(t,6),
                'tilt_deg': round(est.last_tilt_deg,3),
                'speed_xy': round(est.last_speed_xy,5),
                'displacement_xy_08s': '' if est.last_displacement_xy is None else round(est.last_displacement_xy,5),
                'filtered_x': round(float(kf.position[0]),5),
                'filtered_y': round(float(kf.position[1]),5),
                'filtered_z': round(float(kf.position[2]),5),
                'status': state or '',
            })
        report[scenario] = {'samples':len(data),'counts':dict(counters),'transitions':transitions}
    with open(output_csv, 'w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(output[0])); writer.writeheader(); writer.writerows(output)
    print('This is approximate OFFLINE replay (nearest-message synchronisation, not ROS integration).')
    for name, result in report.items():
        print(name)
        print('  samples:',result['samples'],'counts:',result['counts'])
        print('  transitions:',result['transitions'])
    return report

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('input_csv')
    parser.add_argument('--out', default='offline_status_timeline.csv')
    args=parser.parse_args()
    validate(args.input_csv, args.out)
