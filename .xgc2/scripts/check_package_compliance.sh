#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
cd "${repo_root}"

bash -n .xgc2/scripts/*.sh

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
  xgc2_gazebo_scene/CMakeLists.txt
  xgc2_gazebo_scene/package.xml
  xgc2_gazebo_scene/include/xgc2_gazebo_scene/motion_controller.hpp
  xgc2_gazebo_scene/src/gazebo_scene_system_plugin.cpp
  xgc2_gazebo_scene/src/motion_controller.cpp
)

for file in "${required_files[@]}"; do
  test -f "${file}" || { echo "Missing required file: ${file}" >&2; exit 1; }
done

test ! -e gazebo_sim_examples/process-definitions
if rg -n 'process-definitions|px4-multirotor-controller-msgs|Recommends:' \
    .xgc2/product.yml .xgc2/scripts/package_debs.sh gazebo_sim_examples; then
  echo "Retired gazebo_sim_examples must not publish runtime definitions or dependency closure." >&2
  exit 1
fi

grep -Fq 'install(DIRECTORY config launch' gazebo_sim_examples/CMakeLists.txt
grep -Fq '  "python3" \' .xgc2/scripts/package_debs.sh
grep -Fq 'test ! -e "/usr/share/xgc2/process-definitions/xgc2-gazebo-sim-tools.json"' \
  .xgc2/scripts/check_installed_packages.sh

grep -q '^  - xgc2_gazebo_scene$' .xgc2/product.yml
grep -q '^  - ros-noetic-xgc2-gazebo-scene$' .xgc2/product.yml
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
grep -q 'LIBRARIES xgc2_gazebo_scene_motion xgc2_gazebo_scene_system' \
  xgc2_gazebo_scene/CMakeLists.txt
grep -q "LIBRARY DESTINATION \${CATKIN_PACKAGE_LIB_DESTINATION}" \
  xgc2_gazebo_scene/CMakeLists.txt

echo "Package compliance checks passed."
