#!/usr/bin/env bash

# Measure runtime cadence and the growth of the RViz path stream. The script
# starts a real hover launch, samples ROS topics, captures process load, and
# shuts down the complete launch process group when finished.
set -Eeuo pipefail

readonly ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly TAG="${1:-smoothness}"
readonly USE_RVIZ="${2:-false}"
readonly OUTPUT_DIR="${ROOT}/artifacts/performance/${TAG}"
mkdir -p "${OUTPUT_DIR}"

set +u
# The profiler needs ROS CLI tools in its own shell. The launched simulation
# still goes through start_sim.sh and therefore exercises the real entry point.
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROOT}/install/setup.bash"
set -u

cd "${ROOT}"
setsid "${ROOT}/start_sim.sh" hover use_rviz:="${USE_RVIZ}" \
  >"${OUTPUT_DIR}/launch.log" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 5
ros2 topic info --verbose /drone/truth/odom >"${OUTPUT_DIR}/odom_publishers.txt"
ros2 topic info --verbose /tf >"${OUTPUT_DIR}/tf_publishers.txt"
{
  ros2 param get /quadrotor_dynamics_node simulation_frequency
  ros2 param get /quadrotor_dynamics_node state_publish_frequency
  ros2 param get /quadrotor_dynamics_node path_sample_frequency
  ros2 param get /quadrotor_dynamics_node path_publish_frequency
} >"${OUTPUT_DIR}/runtime_parameters.txt"
timeout 7 ros2 topic hz /drone/truth/odom >"${OUTPUT_DIR}/odom_hz.txt" 2>&1 &
odom_probe=$!
timeout 7 ros2 topic hz /tf >"${OUTPUT_DIR}/tf_hz.txt" 2>&1 &
tf_probe=$!
timeout 7 ros2 topic hz /drone/path >"${OUTPUT_DIR}/path_hz.txt" 2>&1 &
path_probe=$!
timeout 7 ros2 topic hz /drone/markers >"${OUTPUT_DIR}/marker_hz.txt" 2>&1 &
marker_probe=$!
timeout 7 ros2 topic bw /drone/path >"${OUTPUT_DIR}/path_bw_early.txt" 2>&1 &
bandwidth_probe=$!
wait "${odom_probe}" || true
wait "${tf_probe}" || true
wait "${path_probe}" || true
wait "${marker_probe}" || true
wait "${bandwidth_probe}" || true

sleep 15
timeout 7 ros2 topic bw /drone/path >"${OUTPUT_DIR}/path_bw_late.txt" 2>&1 || true
ps -eo pid,pcpu,pmem,rss,comm,args --sort=-pcpu \
  | grep -E 'quadrotor_dynamics|position_controller|sensor_simulator|drone_marker|rviz2' \
  | grep -v grep >"${OUTPUT_DIR}/processes.txt" || true

echo "Runtime profile: ${OUTPUT_DIR}"
for report in odom_hz tf_hz path_hz marker_hz path_bw_early path_bw_late; do
  echo "--- ${report} ---"
  tail -n 4 "${OUTPUT_DIR}/${report}.txt"
done
echo "--- processes ---"
cat "${OUTPUT_DIR}/processes.txt"
echo "--- runtime parameters ---"
cat "${OUTPUT_DIR}/runtime_parameters.txt"
echo "--- publisher counts ---"
grep -E 'Publisher count|Node name|Topic type' "${OUTPUT_DIR}/odom_publishers.txt" || true
grep -E 'Publisher count|Node name|Topic type' "${OUTPUT_DIR}/tf_publishers.txt" || true
