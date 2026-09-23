#!/usr/bin/env bash
set -euo pipefail

sim_root="${1:?hl2_rack_sim_1_4 path is required}"
gazebo_gui="${2:-true}"
gazebo_paused="${3:-false}"
world_path="$sim_root/worlds/hl2_ward.world"

if [[ ! -f "$world_path" || ! -d "$sim_root/models" ]]; then
  printf 'Invalid HL2 simulation root: %s\n' "$sim_root" >&2
  exit 2
fi

export QT_X11_NO_MITSHM=1
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
export GAZEBO_MODEL_PATH="$sim_root/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:$sim_root${GAZEBO_RESOURCE_PATH:+:$GAZEBO_RESOURCE_PATH}"
export GAZEBO_MODEL_DATABASE_URI="file://$sim_root/config/gazebo_offline/"
export HL2_SIM_ROOT="$sim_root"
export GAZEBO_LOG_PATH="$sim_root/runtime/gazebo_logs"
mkdir -p "$GAZEBO_LOG_PATH"

# This Gazebo 11 plugin publishes the two simulated Mid-360 point clouds on
# ROS 2 without requiring the unavailable ROS 1 Kinetic installation.
ros2_plugin_path="$sim_root/ros2_mid360_plugin/build"
gazebo_ros2_plugin_path="/opt/ros/humble/lib"
export GAZEBO_PLUGIN_PATH="$gazebo_ros2_plugin_path${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
if [[ -f "$ros2_plugin_path/libhl2_mid360_sensor.so" || -f "$ros2_plugin_path/libhl2_assembly_motion_ros2.so" ]]; then
  export GAZEBO_PLUGIN_PATH="$ros2_plugin_path:$GAZEBO_PLUGIN_PATH"
fi

gazebo_args=(--verbose)
case "$gazebo_paused" in
  true|True|TRUE|1) gazebo_args+=(-u) ;;
  false|False|FALSE|0) ;;
  *) printf 'gazebo_paused must be true or false, got: %s\n' "$gazebo_paused" >&2; exit 2 ;;
esac
gazebo_args+=("$world_path")

case "$gazebo_gui" in
  true|True|TRUE|1) exec gazebo "${gazebo_args[@]}" ;;
  false|False|FALSE|0) exec gzserver "${gazebo_args[@]}" ;;
  *) printf 'gazebo_gui must be true or false, got: %s\n' "$gazebo_gui" >&2; exit 2 ;;
esac
