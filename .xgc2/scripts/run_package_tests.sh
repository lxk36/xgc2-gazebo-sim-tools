#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
ros_distro="${ROS_DISTRO:-noetic}"
workspace="${XGC2_GAZEBO_SIM_TOOLS_TEST_WS:-${repo_root}/.ci/package-tests}"

for tool in catkin_make; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "Missing required package test tool: ${tool}" >&2
    exit 1
  fi
done

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "Missing ROS setup: /opt/ros/${ros_distro}/setup.bash" >&2
  exit 1
fi

rm -rf "${workspace}"
mkdir -p "${workspace}/src"
ln -s "${repo_root}/gazebo_session_manager" "${workspace}/src/gazebo_session_manager"
ln -s "${repo_root}/gazebo_sim_examples" "${workspace}/src/gazebo_sim_examples"

# shellcheck source=/dev/null
source "/opt/ros/${ros_distro}/setup.bash"

catkin_make -C "${workspace}" -DCATKIN_ENABLE_TESTING=OFF
python3 "${repo_root}/gazebo_sim_examples/test/test_uav_auto_takeoff_track.py"

echo "Package tests passed."
