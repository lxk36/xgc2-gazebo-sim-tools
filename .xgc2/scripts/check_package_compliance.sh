#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"

cd "${repo_root}"

bash -n .xgc2/scripts/*.sh

search_files() {
  local pattern="$1"
  shift
  if command -v rg >/dev/null 2>&1; then
    rg -n --glob '!.xgc2/scripts/check_package_compliance.sh' "${pattern}" "$@"
    return
  fi
  find "$@" -type f \
    ! -path './.xgc2/scripts/check_package_compliance.sh' \
    ! -path '.xgc2/scripts/check_package_compliance.sh' \
    -exec grep -HnE "${pattern}" {} +
}

nested_git="$(
  find . \
    -path ./.git -prune -o \
    -path ./.ci -prune -o \
    -path './*/build' -prune -o \
    -path './*/devel' -prune -o \
    -path './*/install' -prune -o \
    -name .git -print
)"
if [[ -n "${nested_git}" ]]; then
  echo "Nested .git directory found." >&2
  echo "${nested_git}" >&2
  exit 1
fi

if git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.ci|\.work|debs)(/|$)' >/dev/null; then
  echo "Generated build artifacts are tracked." >&2
  git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.ci|\.work|debs)(/|$)' >&2
  exit 1
fi

required_files=(
  .clang-format
  .clang-tidy
  .github/workflows/build-debs.yml
  .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh
  .xgc2/scripts/check_cpp_quality.sh
  .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/check_version_bump.sh
  .xgc2/scripts/package_debs.sh
  .xgc2/scripts/publish_apt_repo.sh
  .xgc2/scripts/run_package_tests.sh
  gazebo_sim_vrpn_bridge/CMakeLists.txt
  gazebo_sim_vrpn_bridge/package.xml
  gazebo_sim_vrpn_bridge/include/gazebo_sim_vrpn_bridge/mocap_noise.h
  gazebo_sim_vrpn_bridge/src/gazebo_vrpn_server_node.cpp
  gazebo_sim_vrpn_bridge/test/mocap_noise_tests.cpp
  gazebo_sim_vrpn_bridge/test/vrpn_protocol_e2e.py
  gazebo_sim_vrpn_bridge/test/vrpn_protocol_e2e.test
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

if [[ -e gazebo_sim_vrpn_bridge/include/gazebo_sim_vrpn_bridge/butterworth_filter.h ]]; then
  echo "gazebo_sim_vrpn_bridge must use xgc2_math::SecondOrderButterworthLowPass directly." >&2
  exit 1
fi

if search_files 'ros-noetic-xgc2-controller' .github .xgc2 gazebo_sim_examples gazebo_sim_vrpn_bridge \
  >/tmp/xgc2-gazebo-sim-tools-controller-deps.txt; then
  echo "gazebo_sim_tools must depend on split controller packages directly." >&2
  cat /tmp/xgc2-gazebo-sim-tools-controller-deps.txt >&2
  exit 1
fi

grep -q "ros-noetic-xgc2-multirotor-controller (>= 1.0.7-1)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-ugv-controller (>= 1.0.0-1)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-multirotor-controller (>= 1.0.7-1)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-ugv-controller (>= 1.0.0-1)" .xgc2/scripts/package_debs.sh

grep -q '<xgc2_math/filter/butterworth_filter.hpp>' gazebo_sim_vrpn_bridge/src/gazebo_vrpn_server_node.cpp
grep -q 'xgc2_math::SecondOrderButterworthLowPass' gazebo_sim_vrpn_bridge/src/gazebo_vrpn_server_node.cpp

if ! grep -q 'catkin_add_gtest(gazebo_sim_vrpn_bridge_mocap_noise_tests' gazebo_sim_vrpn_bridge/CMakeLists.txt; then
  echo "gazebo_sim_vrpn_bridge unit test target is missing." >&2
  exit 1
fi

if ! grep -q 'add_rostest(test/vrpn_protocol_e2e.test' gazebo_sim_vrpn_bridge/CMakeLists.txt; then
  echo "gazebo_sim_vrpn_bridge VRPN protocol rostest is missing." >&2
  exit 1
fi

if ! grep -q 'install(DIRECTORY include/${PROJECT_NAME}/' gazebo_sim_vrpn_bridge/CMakeLists.txt; then
  echo "gazebo_sim_vrpn_bridge installed header export is missing." >&2
  exit 1
fi

if search_files 'name="robot_namespace"' gazebo_sim_examples/launch >/tmp/xgc2-gazebo-sim-tools-legacy-args.txt; then
  echo "gazebo_sim_examples uses legacy Scout spawn arg robot_namespace; use ns." >&2
  cat /tmp/xgc2-gazebo-sim-tools-legacy-args.txt >&2
  exit 1
fi

echo "Package compliance checks passed."
