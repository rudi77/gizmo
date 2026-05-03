#!/usr/bin/env bash
# Install ROS 2 Jazzy + Gazebo Harmonic stack for the Gizmo bot on Ubuntu 24.04 (WSL2).
#
# Combination per ROS docs:
#   Ubuntu 24.04 noble  ->  ROS 2 Jazzy  ->  Gazebo Harmonic (via ros_gz vendor pkgs)
#
# Refs:
#   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
#   https://gazebosim.org/docs/latest/ros2_gz_vendor_pkgs/
#   https://github.com/ros-controls/gz_ros2_control
set -euo pipefail

if [[ "$(. /etc/os-release; echo "$VERSION_CODENAME")" != "noble" ]]; then
  echo "This script targets Ubuntu 24.04 (noble). Detected: $(. /etc/os-release; echo "$PRETTY_NAME")" >&2
  exit 1
fi

echo "==> [1/6] Locale (UTF-8)"
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "==> [2/6] Prereqs + ROS 2 apt repository"
sudo apt-get install -y software-properties-common curl ca-certificates gnupg lsb-release
sudo add-apt-repository -y universe

ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -oP '"tag_name": "\K[^"]+' || echo '')"
if [[ -z "${ROS_APT_SOURCE_VERSION}" ]]; then
  echo "Falling back to manual ROS 2 apt source setup."
  sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
else
  curl -fsSLO "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
  sudo apt-get install -y "./ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
  rm -f "ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
fi

sudo apt-get update
sudo apt-get upgrade -y

echo "==> [3/6] ROS 2 Jazzy desktop + dev tools"
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "==> [4/6] Gazebo Harmonic + ros_gz bridge (vendor packages)"
sudo apt-get install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-image \
  ros-jazzy-ros-gz-interfaces

echo "==> [5/6] ros2_control + gz_ros2_control + controllers"
sudo apt-get install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui

echo "==> [6/6] Shell setup (~/.bashrc)"
SETUP_LINE='source /opt/ros/jazzy/setup.bash'
if ! grep -Fxq "$SETUP_LINE" "$HOME/.bashrc"; then
  echo "$SETUP_LINE" >> "$HOME/.bashrc"
  echo "Added ROS 2 sourcing to ~/.bashrc"
fi

# Gazebo resource path hint for custom worlds/models we'll add later
GZ_LINE='export GZ_SIM_RESOURCE_PATH="$HOME/gizmo_ws/install/gizmo_description/share:${GZ_SIM_RESOURCE_PATH:-}"'
if ! grep -Fxq "$GZ_LINE" "$HOME/.bashrc"; then
  echo "$GZ_LINE" >> "$HOME/.bashrc"
fi

cat <<'EOF'

================================================================
Done. Open a NEW shell (so ~/.bashrc is sourced), then verify:

  ros2 --help
  ros2 pkg list | grep -E 'ros_gz|gz_ros2_control|joint_trajectory'
  gz sim --version

If $DISPLAY / $WAYLAND_DISPLAY are empty under WSL2, run from
PowerShell:  wsl --update  &&  wsl --shutdown
================================================================
EOF
