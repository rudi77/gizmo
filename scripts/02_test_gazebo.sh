#!/usr/bin/env bash
# Smoke-test the ROS 2 Jazzy + Gazebo Harmonic install under WSL2.
set -eo pipefail   # no -u: ROS setup scripts reference unbound vars

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy not found. Run 01_install_gazebo_ros2_wsl2.sh first." >&2
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

echo "==> ROS 2 distro"
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
ros2 --help >/dev/null && echo "ros2 CLI: OK"

echo "==> Gazebo version"
gz sim --version || { echo "gz sim missing"; exit 1; }

echo "==> WSLg display vars"
echo "DISPLAY=${DISPLAY:-<unset>}"
echo "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<unset>}"
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  cat >&2 <<'EOF'
WARN: No DISPLAY or WAYLAND_DISPLAY set. Under WSL2/WSLg these are usually
auto-set. From an elevated PowerShell run:
    wsl --update
    wsl --shutdown
then start a fresh WSL shell.
EOF
fi

echo "==> ros_gz bridge package present?"
ros2 pkg prefix ros_gz_bridge >/dev/null && echo "ros_gz_bridge: OK"

echo "==> gz_ros2_control package present?"
ros2 pkg prefix gz_ros2_control >/dev/null && echo "gz_ros2_control: OK"

cat <<'EOF'

All checks passed. To launch a demo world (Ctrl+C to quit):

    source /opt/ros/jazzy/setup.bash
    gz sim -v 4 shapes.sdf

Or headless (good for sanity checks without WSLg):

    gz sim -v 4 -s -r shapes.sdf
EOF
