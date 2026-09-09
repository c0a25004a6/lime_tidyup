#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}\")/.." && pwd)"
world="$repo_root/yolo_ws/cube8_sim_empty.world"
evidence_dir="${1:-$repo_root/yolo_ws/cube8_sim_evidence}"
mkdir -p "$evidence_dir"

command -v ros2 >/dev/null
command -v gzserver >/dev/null
python3 -c 'import cv2, rclpy'

export PYTHONPATH="$repo_root/yolo_ws/src/barcode_detector${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-82}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11389}"
export GAZEBO_MODEL_DATABASE_URI=""

pids=()
cleanup() {
  if ((${#pids[@]})); then
    kill "${pids[@]}" 2>/dev/null || true
    local live
    for _ in {1..20}; do
      live=0
      for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
          live=1
          break
        fi
      done
      if ((live == 0)); then
        break
      fi
      sleep 0.1
    done
    kill -KILL "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}

dump_failure_evidence() {
  local file
  for file in \
    camera_info.yaml \
    pnp_result.yaml \
    roundtrip_status.yaml \
    pnp_probe.log \
    bridge.log \
    compare.log \
    setup.log \
    topics.txt \
    services.txt; do
    if [[ -f "$evidence_dir/$file" ]]; then
      echo "--- $file ---" >&2
      cat "$evidence_dir/$file" >&2
    fi
  done
  if [[ -f "$evidence_dir/gazebo.log" ]]; then
    echo "--- gazebo.log tail ---" >&2
    tail -n 120 "$evidence_dir/gazebo.log" >&2
  fi
}

on_exit() {
  local rc=$?
  if ((rc != 0)); then
    dump_failure_evidence
  fi
  cleanup
  return "$rc"
}
trap on_exit EXIT

wait_for_service() {
  local name="$1"
  local attempts="${2:-80}"
  for ((i=0; i<attempts; i++)); do
    if ros2 service list 2>/dev/null | grep -Fxq "$name"; then
      return 0
    fi
    sleep 0.5
  done
  echo "service did not appear: $name" >&2
  return 1
}

gzserver --verbose \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  "$world" \
  >"$evidence_dir/gazebo.log" 2>&1 &
pids+=("$!")

wait_for_service /spawn_entity
ros2 service list | sort >"$evidence_dir/services.txt"
wait_for_service /gazebo/set_entity_state

python3 -m barcode_detector.cube8_sim_setup \
  >"$evidence_dir/setup.log" 2>&1

ros2 topic list | sort >"$evidence_dir/topics.txt"
if ! grep -Fxq /cube8_sim/camera/camera_info "$evidence_dir/topics.txt"; then
  echo "expected camera topic did not appear: /cube8_sim/camera/camera_info" >&2
  echo "--- ROS topics ---" >&2
  cat "$evidence_dir/topics.txt" >&2
  echo "--- sim setup ---" >&2
  cat "$evidence_dir/setup.log" >&2
  echo "--- Gazebo tail ---" >&2
  tail -n 120 "$evidence_dir/gazebo.log" >&2
  exit 1
fi
timeout 20 ros2 topic echo --once --full-length /cube8_sim/camera/camera_info \
  >"$evidence_dir/camera_info.yaml"

python3 -m barcode_detector.cube8_sim_compare \
  >"$evidence_dir/compare.log" 2>&1 &
pids+=("$!")
python3 -m barcode_detector.cube8_sim_gazebo_bridge \
  >"$evidence_dir/bridge.log" 2>&1 &
pids+=("$!")
python3 -m barcode_detector.cube8_sim_pnp_probe \
  >"$evidence_dir/pnp_probe.log" 2>&1 &
pids+=("$!")

timeout 30 ros2 topic echo --once --full-length /cube8_pose_result \
  >"$evidence_dir/pnp_result.yaml"
timeout 30 ros2 topic echo --once --full-length /cube8_sim/roundtrip_status \
  >"$evidence_dir/roundtrip_status.yaml"

grep -Fq '"status": "accepted"' "$evidence_dir/pnp_result.yaml"
grep -Fq '"physical_robot_authority": false' "$evidence_dir/pnp_result.yaml"
grep -Fq '"status": "PASS"' "$evidence_dir/roundtrip_status.yaml"
grep -Fq '"physical_robot_authority": false' "$evidence_dir/roundtrip_status.yaml"

cat "$evidence_dir/roundtrip_status.yaml"
