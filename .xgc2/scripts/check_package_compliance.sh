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
  .xgc2/scripts/run_package_tests.sh
  gazebo_sim_examples/CMakeLists.txt
  gazebo_sim_examples/package.xml
  gazebo_sim_examples/process-definitions/xgc2-gazebo-sim-tools.json
  xgc2_gazebo_scene/CMakeLists.txt
  xgc2_gazebo_scene/package.xml
  xgc2_gazebo_scene/include/xgc2_gazebo_scene/motion_controller.hpp
  xgc2_gazebo_scene/src/gazebo_scene_system_plugin.cpp
  xgc2_gazebo_scene/src/motion_controller.cpp
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

if search_files 'ros-noetic-xgc2-controller' .github .xgc2 gazebo_sim_examples \
  >/tmp/xgc2-gazebo-sim-tools-controller-deps.txt; then
  echo "gazebo_sim_tools must depend on split controller packages directly." >&2
  cat /tmp/xgc2-gazebo-sim-tools-controller-deps.txt >&2
  exit 1
fi

grep -q "ros-noetic-xgc2-multirotor-controller (>= 1.1.18-4)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-px4-multirotor-controller-msgs (>= 1.2.0-3)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-ugv-controller (>= 1.1.4-9)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-multirotor-controller (>= 1.1.18-4)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-px4-multirotor-controller-msgs (>= 1.2.0-3)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-ugv-controller (>= 1.1.4-9)" .xgc2/scripts/package_debs.sh
grep -q '^  recommends:$' .xgc2/product.yml
grep -Fq "\"Recommends: \${scene_dep}, \${visualization_dep}" \
  .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-gazebo-sim-visualization (>= 1.1.0-12)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-gazebo-sim-vrpn-bridge (>= 1.1.0-13)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-gazebo-sim-worlds (>= 1.1.0-14)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-estimator-hover-thrust (>= 1.1.24-6)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-estimator-rigid-state (>= 1.1.6-6)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-estimator-rigid-state-msgs (>= 1.2.0-3)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-gazebo-sim-visualization (>= 1.1.0-12)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-gazebo-sim-vrpn-bridge (>= 1.1.0-13)" .xgc2/scripts/package_debs.sh
grep -q "\${worlds_pkg} (>= 1.1.0-14)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-estimator-hover-thrust (>= 1.1.24-6)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-estimator-rigid-state (>= 1.1.6-6)" .xgc2/scripts/package_debs.sh
grep -q "ros-noetic-xgc2-estimator-rigid-state-msgs (>= 1.2.0-3)" .xgc2/scripts/package_debs.sh
grep -q '^  - xgc2_gazebo_scene$' .xgc2/product.yml
grep -q '^  - ros-noetic-xgc2-gazebo-scene$' .xgc2/product.yml
grep -q 'ros-noetic-xgc2-gazebo-scene (>= 1.1.0-31)' .xgc2/product.yml
grep -q '/opt/ros/noetic/lib/libxgc2_gazebo_scene_system.so' .xgc2/product.yml
grep -q 'scene_pkg="ros-noetic-xgc2-gazebo-scene"' .xgc2/scripts/package_debs.sh
grep -Fq 'dpkg-shlibdeps' .xgc2/scripts/package_debs.sh
if grep -Fq -- '--ignore-missing-info' .xgc2/scripts/package_debs.sh; then
  echo "Gazebo Scene packaging must not hide missing ELF dependency metadata" >&2
  exit 1
fi
if grep -Eq 'DEBIAN/(postinst|postrm)|ldconfig' .xgc2/scripts/package_debs.sh; then
  echo "Gazebo Scene must not run ldconfig for the ROS-only library path" >&2
  exit 1
fi
grep -Fq 'ros-noetic-gazebo-ros' .xgc2/scripts/package_debs.sh
grep -Fq '<build_export_depend>gazebo_dev</build_export_depend>' xgc2_gazebo_scene/package.xml
grep -Fq '<exec_depend>gazebo_ros</exec_depend>' xgc2_gazebo_scene/package.xml
if grep -Fq '<depend>gazebo_dev</depend>' xgc2_gazebo_scene/package.xml; then
  echo "gazebo_dev must remain a build/export dependency, not a runtime dependency" >&2
  exit 1
fi
if grep -Fq -- '--skip-install-check' .github/workflows/ci.yml .github/workflows/release.yml; then
  echo "CI and release must execute the Debian install checks" >&2
  exit 1
fi
grep -Fq '"default": "/opt/ros/noetic/lib/libxgc2_gazebo_scene_system.so"' \
  gazebo_sim_examples/process-definitions/xgc2-gazebo-sim-tools.json
python3 -m json.tool \
  gazebo_sim_examples/process-definitions/xgc2-gazebo-sim-tools.json >/dev/null
python3 - <<'PY'
import json
from pathlib import Path

manifest = json.loads(Path(
    "gazebo_sim_examples/process-definitions/xgc2-gazebo-sim-tools.json"
).read_text(encoding="utf-8"))
server = next(item for item in manifest["definitions"] if item["id"] == "gazebo-server")
script = server["command"]["args"][1]
compile(script, "gazebo-server-command", "exec")
stable_path = "/opt/ros/noetic/lib/libxgc2_gazebo_scene_system.so"
assert server["parameters"]["properties"]["scenePluginPath"]["default"] == stable_path
assert f'elif scene_plugin == "{stable_path}":' in script
assert "starting without it" in script
PY
grep -q 'LIBRARIES xgc2_gazebo_scene_motion xgc2_gazebo_scene_system' \
  xgc2_gazebo_scene/CMakeLists.txt
grep -q "LIBRARY DESTINATION \${CATKIN_PACKAGE_LIB_DESTINATION}" \
  xgc2_gazebo_scene/CMakeLists.txt

if search_files 'name="robot_namespace"' gazebo_sim_examples/launch >/tmp/xgc2-gazebo-sim-tools-legacy-args.txt; then
  echo "gazebo_sim_examples uses legacy Scout spawn arg robot_namespace; use ns." >&2
  cat /tmp/xgc2-gazebo-sim-tools-legacy-args.txt >&2
  exit 1
fi

echo "Package compliance checks passed."
