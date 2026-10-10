set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -d "$ROOT/ros2_ws" ]]; then
  echo "ERROR: put validate_all.sh in the root roboticks-testovoe directory (next to ros2_ws)." >&2
  exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash" >&2
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [[ ! -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  echo "ERROR: ROS workspace is not built. First run colcon build in ros2_ws." >&2
  exit 2
fi
set +u
source "$ROOT/ros2_ws/install/setup.bash"
set -u
if ! ros2 pkg executables robot_gps_kalman 2>/dev/null | grep -q '^robot_gps_kalman gps_filter$'; then
  echo "ERROR: executable robot_gps_kalman gps_filter not found. Rebuild the latest package." >&2
  exit 2
fi
PLOTTER="$ROOT/ros2_ws/src/robot_gps_kalman/tools/analyze_live_csv.py"
REPORTER="$ROOT/ros2_ws/report_full_validation.py"
for script in "$PLOTTER" "$REPORTER"; do
  if [[ ! -f "$script" ]]; then
    echo "ERROR: script not found: $script" >&2
    exit 2
  fi
done
RUN_LABEL="$(date +%Y%m%d_%H%M%S)_$BASHPID"
OUT="$ROOT/validation_runs/$RUN_LABEL"
mkdir -p "$OUT/logs" "$OUT/plots"
CSV="$OUT/robot_telemetry_live_all.csv"
RATE="${RATE:-0.5}"
NODE_PID=''
BAG_PID=''

stop_group() {
  local pid="${1:-}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    # The node flushes every CSV row, so data is preserved on shutdown.
    wait "$pid" 2>/dev/null || true
  fi
}
cleanup() {
  stop_group "$BAG_PID"
  stop_group "$NODE_PID"
}
trap cleanup EXIT
trap 'echo "Interrupted. Logs/CSV (if any): '"$OUT"'" >&2; exit 130' INT TERM

SCENARIOS=('rosbag2_езда' 'rosbag2_застрял' 'rosbag2_перевернулся')
for scenario in "${SCENARIOS[@]}"; do
  if [[ ! -d "$ROOT/$scenario" || ! -f "$ROOT/$scenario/metadata.yaml" ]]; then
    echo "ERROR: missing rosbag2 folder: $ROOT/$scenario" >&2
    exit 2
  fi
done

printf '\n=== LIVE VALIDATION: all 3 ROS 2 bags ===\n'
printf 'Root: %s\nOutput: %s\nBag playback rate: %s\n' "$ROOT" "$OUT" "$RATE"
printf 'Before starting, stop OTHER ros2 bag play and robot_gps_kalman nodes.\n\n'
if ros2 node list 2>/dev/null | grep -Fxq '/robot_gps_kalman'; then
  echo 'ERROR: another /robot_gps_kalman is already running. Stop it (Ctrl+C) before validation.' >&2
  exit 2
fi

for scenario in "${SCENARIOS[@]}"; do
  echo "----------------------------------------------------------"
  echo "RUN: $scenario"
  NODELOG="$OUT/logs/${scenario}_node.log"
  BAGLOG="$OUT/logs/${scenario}_bag.log"
  setsid ros2 run robot_gps_kalman gps_filter --ros-args \
    -p use_sim_time:=true \
    -p "scenario:=$scenario" \
    -p "csv_output:=$CSV" \
    -p csv_append:=true \
    > "$NODELOG" 2>&1 &
  NODE_PID=$!

  ready=0
  for _ in $(seq 1 40); do
    if [[ -f "$NODELOG" ]] && grep -q 'GPS Kalman online' "$NODELOG" && grep -q 'Live telemetry CSV:' "$NODELOG"; then ready=1; break; fi
    if ! kill -0 "$NODE_PID" 2>/dev/null; then break; fi
    sleep 0.25
  done
  if [[ "$ready" != 1 ]]; then
    echo "ERROR: ROS node did not initialize for $scenario. See $NODELOG" >&2
    cat "$NODELOG" >&2
    exit 1
  fi
  sleep 3

  setsid ros2 bag play "$ROOT/$scenario" --clock --rate "$RATE" \
      --topics /RoboMowerRealistic/gps \
               /RoboMowerRealistic/gps/speed_vector \
               /RoboMowerRealistic/gps/speed \
               /imu \
      > "$BAGLOG" 2>&1 &
  BAG_PID=$!
  if wait "$BAG_PID"; then
    BAG_PID=''
  else
    echo "ERROR: bag playback failed: $scenario. See $BAGLOG" >&2
    exit 1
  fi
  sleep 2
  stop_group "$NODE_PID"
  NODE_PID=''
  python3 - "$CSV" "$scenario" "$ROOT/ros2_ws" <<'PY'
import csv, sys
from collections import Counter
csv_path, scenario, report_dir = sys.argv[1:4]
sys.path.insert(0, report_dir)
from report_full_validation import EXPECTED, STATES, scenario_sanity
expected = EXPECTED[scenario][0]
with open(csv_path, encoding='utf-8-sig', newline='') as f:
    rows = [r for r in csv.DictReader(f) if r.get('scenario') == scenario]
statuses = Counter(r.get('robot_status', '') for r in rows)
segments = set(r.get('segment_id', '') for r in rows)
print(f"Captured {len(rows)}/{expected} GPS rows, "
      f"{sum(bool(r.get('robot_status')) for r in rows)} status rows, "
      f"segments={len(segments)} for {scenario}")
if len(rows) != expected:
    sys.exit(f"ERROR: incomplete {scenario}: {len(rows)}/{expected} GPS rows. "
             "Check node/bag logs; no plots will be generated for this run.")
if len(segments) != 1:
    sys.exit(f"ERROR: {scenario} contains {len(segments)} segments: "
             "time discontinuity or ROS time/GPS timestamp mismatch.")
valid = sum(statuses.get(status, 0) for status in STATES)
if scenario_sanity(scenario, statuses, len(rows), valid) != 'PASS':
    sys.exit(f"ERROR: unexpected/missing robot status in {scenario}: {dict(statuses)}")
PY
  echo "Done: $scenario"
done

printf '\n=== PLOTS ===\n'
python3 "$PLOTTER" "$CSV" --plots --output-dir "$OUT/plots" \
  | tee "$OUT/plot_log.txt"

printf '\n=== VALIDATION METRICS ===\n'
python3 "$REPORTER" "$CSV" --output-dir "$OUT" | tee "$OUT/report_log.txt"

printf '\nVALIDATION COMPLETE\n'
printf 'CSV:     %s\n' "$CSV"
printf 'Metrics: %s\n' "$OUT/scenario_metrics.csv"
printf 'Events:  %s\n' "$OUT/status_transitions.csv"
printf 'Plots:   %s\n' "$OUT/plots"
