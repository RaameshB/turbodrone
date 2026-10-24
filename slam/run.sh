#!/usr/bin/env bash
# Run monocular stella_vslam inside the `ros2` distrobox.
#   slam/run.sh --dir slam/data/tum_xyz_frames --config slam/config/tum_fr1_mono.yaml
#   slam/run.sh --url http://127.0.0.1:8000/mjpeg --config slam/config/drone.yaml
# Args before --config/--vocab are passed to image_source.py (--url/--dir/--rate/--calib/--gray).
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CONFIG="$HERE/config/drone.yaml"; VOCAB="$HERE/data/orb_vocab.fbow"; RVIZ="$HERE/config/slam.rviz"; SRC=(); MAPOUT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --config) CONFIG="$(realpath "$2")"; shift 2;;
    --vocab) VOCAB="$(realpath "$2")"; shift 2;;
    --map-out) MAPOUT="$(realpath -m "$2")"; shift 2;;
    --no-rviz) RVIZ=""; shift;;
    --rviz) shift;;
    --dir) SRC+=("$1" "$(realpath "$2")"); shift 2;;
    *) SRC+=("$1"); shift;;
  esac
done
cleanup_host() {
  trap - EXIT INT TERM HUP
  [ -n "${DB_PID:-}" ] && kill "$DB_PID" 2>/dev/null || true
  pkill -INT -x run_slam 2>/dev/null || true
  sleep 0.2
  pkill -9 -x run_slam 2>/dev/null || true
  pkill -9 -x rviz2 2>/dev/null || true
  pkill -9 -f "slam/image_source\.py" 2>/dev/null || true
}
cleanup_host
trap cleanup_host EXIT INT TERM HUP
distrobox enter ros2 -- env HERE="$HERE" CONFIG="$CONFIG" VOCAB="$VOCAB" MAPOUT="$MAPOUT" RVIZ="$RVIZ" SRCARGS="${SRC[*]}" bash -c '
  export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH
  source /opt/ros/jazzy/setup.bash; source $HOME/slam_ws/install/setup.bash
  if [ -n "$RVIZ" ] && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    rviz2 -d "$RVIZ" >/dev/null 2>&1 &
  fi
  python3 $HERE/image_source.py $SRCARGS --gray &
  exec $HOME/slam_ws/install/stella_vslam_ros/lib/stella_vslam_ros/run_slam -v $VOCAB -c $CONFIG ${MAPOUT:+--map-db-out $MAPOUT} --ros-args -p publish_tf:=false -r camera/image_raw:=/camera/image_raw
' &
DB_PID=$!
wait "$DB_PID" || true
