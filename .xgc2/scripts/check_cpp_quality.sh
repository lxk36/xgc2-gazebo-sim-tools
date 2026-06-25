#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
ros_distro="${ROS_DISTRO:-noetic}"
workspace="${XGC2_GAZEBO_SIM_TOOLS_QUALITY_WS:-${repo_root}/.ci/cpp-quality}"

sources=(
  gazebo_sim_vrpn_bridge/include/gazebo_sim_vrpn_bridge/mocap_noise.h
  gazebo_sim_vrpn_bridge/src/gazebo_vrpn_server_node.cpp
  gazebo_sim_vrpn_bridge/test/mocap_noise_tests.cpp
)

for tool in catkin_make clang-format clang-tidy cppcheck; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "Missing required C++ quality tool: ${tool}" >&2
    exit 1
  fi
done

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "Missing ROS setup: /opt/ros/${ros_distro}/setup.bash" >&2
  exit 1
fi

cd "${repo_root}"

clang-format --dry-run --Werror "${sources[@]}"

rm -rf "${workspace}"
mkdir -p "${workspace}/src"
ln -s "${repo_root}/gazebo_sim_vrpn_bridge" "${workspace}/src/gazebo_sim_vrpn_bridge"

# shellcheck source=/dev/null
source "/opt/ros/${ros_distro}/setup.bash"

catkin_make -C "${workspace}" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual -Werror" \
  gazebo_vrpn_server_node \
  gazebo_sim_vrpn_bridge_mocap_noise_tests

clang_tidy_sources=(
  "${workspace}/src/gazebo_sim_vrpn_bridge/src/gazebo_vrpn_server_node.cpp"
  "${workspace}/src/gazebo_sim_vrpn_bridge/test/mocap_noise_tests.cpp"
)
clang-tidy --quiet -p "${workspace}/build" "${clang_tidy_sources[@]}" 2>&1 |
  sed -E '/^[0-9]+ warnings generated\.$/d'

cppcheck \
  --enable=warning,performance,portability,style \
  --error-exitcode=1 \
  --std=c++14 \
  --inline-suppr \
  -Igazebo_sim_vrpn_bridge/include \
  gazebo_sim_vrpn_bridge/include \
  gazebo_sim_vrpn_bridge/src \
  gazebo_sim_vrpn_bridge/test

echo "C++ quality checks passed."
