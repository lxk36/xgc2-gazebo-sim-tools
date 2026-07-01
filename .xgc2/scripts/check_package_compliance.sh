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
  .github/workflows/ci.yml
  .github/workflows/release.yml
  .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh
  .xgc2/scripts/check_cpp_quality.sh
  .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/check_version_bump.sh
  .xgc2/scripts/package_debs.sh
  .xgc2/scripts/publish_apt_repo.sh
  .xgc2/scripts/run_package_tests.sh
  gazebo_session_manager/CMakeLists.txt
  gazebo_session_manager/package.xml
  gazebo_sim_examples/CMakeLists.txt
  gazebo_sim_examples/package.xml
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

for split_package in gazebo_sim_visualization gazebo_sim_vrpn_bridge; do
  if [[ -e "${split_package}" ]]; then
    echo "${split_package} is owned by its split repository and must not be tracked here." >&2
    exit 1
  fi
done

if search_files 'ros-noetic-xgc2-controller' .github .xgc2 gazebo_sim_examples gazebo_session_manager \
  >/tmp/xgc2-gazebo-sim-tools-controller-deps.txt; then
  echo "gazebo_sim_tools must depend on split controller packages directly." >&2
  cat /tmp/xgc2-gazebo-sim-tools-controller-deps.txt >&2
  exit 1
fi

grep -q "ros-noetic-xgc2-multirotor-controller (>= 1.1.15-1)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-ugv-controller (>= 1.1.1-1)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-multirotor-controller (>= 1.1.15-1)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-ugv-controller (>= 1.1.1-1)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-gazebo-sim-visualization (>= 1.1.0-2)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-gazebo-sim-vrpn-bridge (>= 1.1.0-2)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-gazebo-sim-worlds (>= 1.1.0-2)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-estimator-hover-thrust (>= 1.1.22-2)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-estimator-rigid-state (>= 1.1.3-2)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-gazebo-sim-visualization (>= 1.1.0-2)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-gazebo-sim-vrpn-bridge (>= 1.1.0-2)" .xgc2/scripts/package_debs.sh
grep -q "\${worlds_pkg} (>= 1.1.0-2)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-estimator-hover-thrust (>= 1.1.22-2)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-estimator-rigid-state (>= 1.1.3-2)" .xgc2/scripts/package_debs.sh

if search_files 'name="robot_namespace"' gazebo_sim_examples/launch >/tmp/xgc2-gazebo-sim-tools-legacy-args.txt; then
  echo "gazebo_sim_examples uses legacy Scout spawn arg robot_namespace; use ns." >&2
  cat /tmp/xgc2-gazebo-sim-tools-legacy-args.txt >&2
  exit 1
fi

echo "Package compliance checks passed."
