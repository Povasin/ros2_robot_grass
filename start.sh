set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -d "$ROOT/ros2_ws" ]]; then
  echo "ERROR: put start.sh in the root roboticks-testovoe directory (next to ros2_ws)." >&2
  exit 2
fi
exec 9<"${BASH_SOURCE[0]}"
if ! flock -n 9; then
  echo "ERROR: another start.sh is already running." >&2
  exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash" >&2
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
missing_packages=()
for package in python3-numpy python3-matplotlib python3-colcon-common-extensions; do
  if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -qx 'install ok installed'; then
    missing_packages+=("$package")
  fi
done
if (( ${#missing_packages[@]} )); then
  sudo apt install -y "${missing_packages[@]}"
fi

printf '\n=== BUILD robot_gps_kalman ===\n'
cd "$ROOT/ros2_ws"
colcon build --symlink-install --packages-select robot_gps_kalman
cd "$ROOT"
if [[ ! -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  echo "ERROR: build did not create ros2_ws/install/setup.bash." >&2
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
  [[ -n "$pid" ]] || return 0
  kill -TERM -- "-$pid" 2>/dev/null || true
  for _ in {1..50}; do
    if ! kill -0 -- "-$pid" 2>/dev/null; then break; fi
    sleep 0.1
  done
  if kill -0 -- "-$pid" 2>/dev/null; then
    echo "WARNING: process group $pid did not stop within 5 seconds; forcing shutdown." >&2
    kill -KILL -- "-$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
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
python3 - "$RATE" <<'PY'
import math, sys, time
import rclpy
from rclpy.node import Node

try:
    rate = float(sys.argv[1])
except ValueError:
    sys.exit('ERROR: RATE must be a finite number greater than zero.')
if not math.isfinite(rate) or rate <= 0:
    sys.exit('ERROR: RATE must be a finite number greater than zero.')

rclpy.init(args=[])
node = Node('validation_preflight')
topics = ['/clock', '/imu', '/RoboMowerRealistic/gps',
          '/RoboMowerRealistic/gps/speed_vector', '/RoboMowerRealistic/gps/speed',
          '/robot/gps_filtered', '/robot/status']
try:
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        for topic in topics:
            publishers = node.get_publishers_info_by_topic(topic)
            if publishers:
                names = ', '.join(p.node_namespace.rstrip('/') + '/' + p.node_name
                                  for p in publishers)
                sys.exit(f'ERROR: {topic} already has publishers: {names}. Stop them first.')
        for name, namespace in node.get_node_names_and_namespaces():
            if name in {'robot_gps_kalman', 'rosbag2_player', 'validation_player'}:
                sys.exit(f'ERROR: another {namespace.rstrip("/")}/{name} is running. Stop it first.')
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

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
  setsid ros2 bag play "$ROOT/$scenario" --clock --rate "$RATE" \
      --start-paused --disable-keyboard-controls --wait-for-all-acked 10000 \
      --remap __node:=validation_player __ns:=/ \
      --topics /RoboMowerRealistic/gps \
               /RoboMowerRealistic/gps/speed_vector \
               /RoboMowerRealistic/gps/speed \
               /imu \
      > "$BAGLOG" 2>&1 &
  BAG_PID=$!
  python3 - "$NODE_PID" "$BAG_PID" <<'PY'
import os, sys, time
import rclpy
from rclpy.node import Node
from rosbag2_interfaces.srv import Resume

rclpy.init(args=[])
node = Node('validation_startup')
client = node.create_client(Resume, '/validation_player/resume')
topics = ['/clock', '/imu', '/RoboMowerRealistic/gps',
          '/RoboMowerRealistic/gps/speed_vector', '/RoboMowerRealistic/gps/speed']
deadline = time.monotonic() + 20
try:
    while True:
        for pid in sys.argv[1:3]:
            try:
                os.kill(int(pid), 0)
            except ProcessLookupError:
                sys.exit('ERROR: node/player exited during startup. Check node/bag logs.')
        ready = client.service_is_ready()
        for topic in topics:
            publishers = node.get_publishers_info_by_topic(topic)
            if any(p.node_name != 'validation_player' or p.node_namespace != '/'
                   for p in publishers) or len(publishers) > 1:
                sys.exit(f'ERROR: conflicting publishers on {topic}. Stop other playback.')
            subscribers = node.get_subscriptions_info_by_topic(topic)
            ready = ready and len(publishers) == 1 and any(
                s.node_name == 'robot_gps_kalman' and s.node_namespace == '/'
                for s in subscribers)
        if ready:
            break
        if time.monotonic() >= deadline:
            sys.exit('ERROR: ROS subscriptions/player did not become ready within 20 seconds.')
        rclpy.spin_once(node, timeout_sec=0.1)
    future = client.call_async(Resume.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done() or future.exception() is not None:
        sys.exit('ERROR: could not resume bag playback. Check bag log.')
    print('ROS subscriptions ready; playback resumed.')
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
  if wait "$BAG_PID"; then
    BAG_PID=''
  else
    echo "ERROR: bag playback failed: $scenario. See $BAGLOG" >&2
    exit 1
  fi
  python3 - "$CSV" "$scenario" "$ROOT/ros2_ws" "$NODE_PID" <<'PY'
import csv, os, sys, time
from collections import Counter
from pathlib import Path
csv_path, scenario, report_dir = sys.argv[1:4]
sys.path.insert(0, report_dir)
from report_full_validation import EXPECTED, STATES, scenario_sanity
expected = EXPECTED[scenario][0]
deadline = time.monotonic() + 10
rows = []
while True:
    try:
        os.kill(int(sys.argv[4]), 0)
    except ProcessLookupError:
        sys.exit(f'ERROR: ROS node exited before capture validation for {scenario}. Check node log.')
    if Path(csv_path).exists():
        with open(csv_path, encoding='utf-8-sig', newline='') as f:
            lines = f.readlines()
            if lines and not lines[-1].endswith('\n'):
                lines.pop()
            rows = [r for r in csv.DictReader(lines)
                    if r.get('scenario') == scenario and None not in r.values()]
    if len(rows) >= expected or time.monotonic() >= deadline:
        break
    time.sleep(0.1)
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
  stop_group "$NODE_PID"
  NODE_PID=''
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
