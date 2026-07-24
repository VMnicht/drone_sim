#!/usr/bin/env bash

# One entry point for WSL/Ubuntu. It loads ROS and the workspace internally, so
# the calling terminal never needs to run source manually.
if [ -z "${BASH_VERSION:-}" ]; then
    exec bash "$0" "$@"
fi
set -Eeuo pipefail

readonly ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly WS_SETUP="${ROOT}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "错误：未找到 ROS2 Humble：${ROS_SETUP}" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u

if [[ ! -f "${WS_SETUP}" ]]; then
    echo "工作区尚未构建，正在执行 colcon build --symlink-install ..."
    (cd "${ROOT}" && colcon build --symlink-install)
fi

set +u
# shellcheck disable=SC1091
source "${WS_SETUP}"
set -u

if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
    export DISPLAY="${DISPLAY:-:0}"
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
fi

cd "${ROOT}"
mode="${1:-hover}"
[[ $# -gt 0 ]] && shift

# Every direct mode script eventually enters here. Hold one workspace-wide
# advisory lock for the complete lifetime of the launched process tree so two
# dynamics nodes can never publish the same TF/topic concurrently. The Panel
# itself does not hold the lock; whichever mode it starts does.
if [[ "${DRONE_SIM_LOCK_HELD:-0}" != "1" && "${mode}" != "panel" && "${mode}" != "help" && "${mode}" != "-h" && "${mode}" != "--help" ]]; then
    runtime_lock_dir="${XDG_RUNTIME_DIR:-/tmp}"
    if [[ ! -d "${runtime_lock_dir}" || ! -w "${runtime_lock_dir}" ]]; then
        runtime_lock_dir=/tmp
    fi
    readonly RUNTIME_LOCK_FILE="${runtime_lock_dir}/drone_sim_ws_${UID}.lock"
    exec {RUNTIME_LOCK_FD}>>"${RUNTIME_LOCK_FILE}"
    if ! flock -n "${RUNTIME_LOCK_FD}"; then
        echo "错误：已有一个仿真/评测模式正在运行。请先停止它，再启动 ${mode}。" >&2
        echo "锁文件：${RUNTIME_LOCK_FILE}" >&2
        exit 3
    fi
    printf 'pid=%s mode=%s started=%s\n' "$$" "${mode}" "$(date --iso-8601=seconds)" \
        >"${RUNTIME_LOCK_FILE}"
    # Keep this outer Bash process alive as the lock owner. Python closes
    # inherited descriptors when ros2 launch starts, so simply exec'ing the
    # launch command would silently release the advisory lock.
    export DRONE_SIM_LOCK_HELD=1
    set +e
    "${BASH_SOURCE[0]}" "${mode}" "$@"
    child_status=$?
    set -e
    exit "${child_status}"
fi

case "${mode}" in
    hover)
        exec ros2 launch drone_bringup hover.launch.py use_rviz:=true "$@"
        ;;
    experiment)
        scenario="${1:-hover}"
        [[ $# -gt 0 ]] && shift
        use_rviz=false
        if [[ "${1:-}" == "--rviz" ]]; then
            use_rviz=true
            shift
        fi
        exec ros2 launch drone_bringup experiment.launch.py \
            scenario:="${scenario}" use_rviz:="${use_rviz}" "$@"
        ;;
    multi)
        exec ros2 launch drone_bringup multi_drone.launch.py use_rviz:=true "$@"
        ;;
    ground-station|ground_station)
        exec ros2 launch drone_bringup ground_station.launch.py "$@"
        ;;
    panel)
        exec python3 scripts/mode_panel.py "$@"
        ;;
    batch)
        exec python3 scripts/run_all_experiments.py "$@"
        ;;
    help|-h|--help)
        cat <<'EOF'
用法：
  ./start_sim.sh hover
  ./start_sim.sh experiment <场景> [--rviz] [duration:=秒] [output_dir:=目录]
  ./start_sim.sh multi
  ./start_sim.sh ground-station
  ./start_sim.sh panel
  ./start_sim.sh batch [--scenario 名称 ...]

场景：hover target square circle figure_eight wind_gust sensor_noise
      fault_motor five_obstacles narrow_passage perception_replan

Panel：http://127.0.0.1:8060，可启动/停止模式、查看日志并编辑 YAML。
EOF
        ;;
    *)
        echo "未知模式：${mode}；运行 ./start_sim.sh help 查看用法。" >&2
        exit 2
        ;;
esac
