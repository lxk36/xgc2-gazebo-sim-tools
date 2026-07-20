#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"

if [[ -z "${VERSION}" ]]; then
  echo "package version is missing; set PACKAGE_VERSION or .xgc2/product.yml version" >&2
  exit 1
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

write_control() {
  local pkg_root="$1"
  local package="$2"
  local depends="$3"
  local description="$4"
  local extra_fields="${5:-}"

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package}"
  {
    cat <<EOF
Package: ${package}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${depends}
EOF
    if [[ -n "${extra_fields}" ]]; then
      printf '%s\n' "${extra_fields}"
    fi
    cat <<EOF
Description: ${description}
EOF
  } > "${pkg_root}/DEBIAN/control"
  printf '%s package\n' "${package}" > "${pkg_root}/usr/share/doc/${package}/README"
  chmod 0755 "${pkg_root}/DEBIAN"
}

copy_ros_package_paths() {
  local ros_pkg="$1"
  local dst_root="$2"

  copy_path "${PREFIX_ROOT}/share/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/include/${ros_pkg}" "${dst_root}"
}

build_ros_package_deb() {
  local package="$1"
  local ros_pkg="$2"
  local depends="$3"
  local description="$4"
  local extra_fields="${5:-}"

  local pkg_root="${BUILD_DIR}/${package}"
  rm -rf "${pkg_root}"
  mkdir -p "${pkg_root}"

  copy_ros_package_paths "${ros_pkg}" "${pkg_root}"
  write_control "${pkg_root}" "${package}" "${depends}" "${description}" "${extra_fields}"
  if [[ "${ros_pkg}" == "gazebo_sim_examples" ]]; then
    install -D -m 0644 \
      "${PREFIX_ROOT}/share/${ros_pkg}/process-definitions/xgc2-gazebo-sim-tools.json" \
      "${pkg_root}/usr/share/xgc2/process-definitions/xgc2-gazebo-sim-tools.json"
    python3 -m json.tool \
      "${pkg_root}/usr/share/xgc2/process-definitions/xgc2-gazebo-sim-tools.json" >/dev/null
    test -x "${pkg_root}${PREFIX}/lib/gazebo_sim_examples/uav_auto_takeoff_track.py"
  fi
  fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

examples_pkg="ros-noetic-xgc2-gazebo-sim-examples"
scene_pkg="ros-noetic-xgc2-gazebo-scene"
worlds_pkg="ros-noetic-xgc2-gazebo-sim-worlds"
visualization_dep="ros-noetic-xgc2-gazebo-sim-visualization (>= 1.1.0-12)"
vrpn_bridge_dep="ros-noetic-xgc2-gazebo-sim-vrpn-bridge (>= 1.1.0-13)"
scene_dep="${scene_pkg} (>= ${VERSION})"

build_scene_deb() {
  local package="$1"
  local pkg_root="${BUILD_DIR}/${package}"
  local motion_library="${pkg_root}${PREFIX}/lib/libxgc2_gazebo_scene_motion.so"
  local system_plugin="${pkg_root}${PREFIX}/lib/libxgc2_gazebo_scene_system.so"
  local shlibdeps_output
  local shlibdeps
  local shlibdeps_stderr="${BUILD_DIR}/dpkg-shlibdeps.stderr"
  local unexpected_shlibdeps_stderr="${BUILD_DIR}/dpkg-shlibdeps-unexpected.stderr"

  rm -rf "${pkg_root}"
  mkdir -p "${pkg_root}"

  copy_path "${PREFIX_ROOT}/share/xgc2_gazebo_scene" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/include/xgc2_gazebo_scene" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/libxgc2_gazebo_scene_motion.so" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/libxgc2_gazebo_scene_system.so" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/pkgconfig/xgc2_gazebo_scene.pc" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/lib/python3/dist-packages/xgc2_gazebo_scene" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/common-lisp/ros/xgc2_gazebo_scene" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/gennodejs/ros/xgc2_gazebo_scene" "${pkg_root}"
  copy_path "${PREFIX_ROOT}/share/roseus/ros/xgc2_gazebo_scene" "${pkg_root}"

  test -f "${pkg_root}${PREFIX}/share/xgc2_gazebo_scene/package.xml"
  test -f "${pkg_root}${PREFIX}/share/xgc2_gazebo_scene/msg/ObstacleDefinition.msg"
  test -f "${pkg_root}${PREFIX}/share/xgc2_gazebo_scene/srv/ConfigureMotions.srv"
  test -f "${pkg_root}${PREFIX}/include/xgc2_gazebo_scene/ObstacleDefinition.h"
  test -f "${pkg_root}${PREFIX}/include/xgc2_gazebo_scene/motion_controller.hpp"
  test -f "${pkg_root}${PREFIX}/lib/pkgconfig/xgc2_gazebo_scene.pc"
  test -f "${pkg_root}${PREFIX}/lib/python3/dist-packages/xgc2_gazebo_scene/msg/_ObstacleDefinition.py"
  test -f "${pkg_root}${PREFIX}/share/common-lisp/ros/xgc2_gazebo_scene/msg/ObstacleDefinition.lisp"
  test -f "${pkg_root}${PREFIX}/share/gennodejs/ros/xgc2_gazebo_scene/msg/ObstacleDefinition.js"
  test -f "${pkg_root}${PREFIX}/share/roseus/ros/xgc2_gazebo_scene/msg/ObstacleDefinition.l"
  test -f "${motion_library}"
  test -f "${system_plugin}"

  # Python bytecode embeds the build path and is regenerated by Python when
  # needed. Keep only the architecture-independent generated message sources.
  find "${pkg_root}${PREFIX}/lib/python3/dist-packages/xgc2_gazebo_scene" \
    -type d -name __pycache__ -prune -exec rm -rf {} +

  mkdir -p "${BUILD_DIR}/debian"
  cat > "${BUILD_DIR}/debian/control" <<EOF
Source: xgc2-gazebo-sim-tools
Section: misc
Priority: optional
Maintainer: XGC2 <apt@example.com>

Package: ${scene_pkg}
Architecture: any

Package: ${examples_pkg}
Architecture: any
EOF
  shlibdeps_output="$(
    cd "${BUILD_DIR}"
    dpkg-shlibdeps \
      -O \
      "-l${pkg_root}${PREFIX}/lib" \
      "-e${motion_library}" \
      "-e${system_plugin}" \
      2>"${shlibdeps_stderr}"
  )"
  grep -Ev \
    "^dpkg-shlibdeps: warning: can't extract name and version from library name '(libxgc2_gazebo_scene_motion|libroscpp|librosconsole|libroscpp_serialization|librostime)\\.so'$|^dpkg-shlibdeps: warning: binaries to analyze should already be installed in their package's directory$" \
    "${shlibdeps_stderr}" >"${unexpected_shlibdeps_stderr}" || true
  if [[ -s "${unexpected_shlibdeps_stderr}" ]]; then
    echo "dpkg-shlibdeps emitted an unexpected warning:" >&2
    cat "${unexpected_shlibdeps_stderr}" >&2
    exit 1
  fi
  shlibdeps="${shlibdeps_output#shlibs:Depends=}"
  if [[ "${shlibdeps}" == "${shlibdeps_output}" || -z "${shlibdeps}" ]]; then
    echo "dpkg-shlibdeps did not produce Gazebo Scene runtime dependencies" >&2
    exit 1
  fi
  if ! grep -Eq '(^|, )libgazebo11( |[(])' <<<"${shlibdeps}"; then
    echo "Gazebo Scene runtime dependencies do not include libgazebo11" >&2
    exit 1
  fi
  if grep -Eq '(^|, )(cmake|gazebo-dev|libgazebo11-dev|ros-noetic-message-generation)( |[(,]|$)' \
      <<<"${shlibdeps}"; then
    echo "Gazebo Scene runtime dependencies leaked a build-only package" >&2
    exit 1
  fi

  write_control \
    "${pkg_root}" \
    "${package}" \
    "${shlibdeps}, ros-noetic-gazebo-ros, ros-noetic-geometry-msgs, ros-noetic-message-runtime, ros-noetic-rosconsole, ros-noetic-roscpp, ros-noetic-roscpp-serialization, ros-noetic-rostime, ros-noetic-std-msgs" \
    "XGC2 Gazebo Classic ground-truth scene SystemPlugin"
  find "${pkg_root}" -type d -exec chmod 0755 {} +
  find "${pkg_root}" -type f -exec chmod 0644 {} +
  if readelf -d "${motion_library}" "${system_plugin}" | grep -Eq '(RPATH|RUNPATH)'; then
    echo "Gazebo Scene libraries contain a build-time RPATH/RUNPATH" >&2
    readelf -d "${motion_library}" "${system_plugin}" >&2
    exit 1
  fi
  if ! nm -D --defined-only "${system_plugin}" \
      | grep -E '[[:space:]]RegisterPlugin$' >/dev/null; then
    echo "Gazebo Scene SystemPlugin does not export RegisterPlugin" >&2
    exit 1
  fi
  if LD_LIBRARY_PATH="${pkg_root}${PREFIX}/lib:${PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
      ldd "${system_plugin}" | grep -q 'not found'; then
    echo "Gazebo Scene SystemPlugin has unresolved shared libraries" >&2
    LD_LIBRARY_PATH="${pkg_root}${PREFIX}/lib:${PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
      ldd "${system_plugin}" >&2
    exit 1
  fi

  fakeroot dpkg-deb --build \
    "${pkg_root}" \
    "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_scene_deb "${scene_pkg}"

build_ros_package_deb \
  "${examples_pkg}" \
  "gazebo_sim_examples" \
  "ros-noetic-vrpn-client-ros, ros-noetic-mavros, ros-noetic-mavros-msgs, ros-noetic-geometry-msgs, ros-noetic-nav-msgs, ros-noetic-std-msgs, ros-noetic-rospy, ros-noetic-roslaunch" \
  "XGC2 Gazebo Classic example launch orchestration" \
  "Recommends: ${scene_dep}, ${visualization_dep}, ${vrpn_bridge_dep}, ${worlds_pkg} (>= 1.1.0-14), ros-noetic-xgc2-gazebo-sim-fs150-sitl (>= 1.1.0-12), ros-noetic-xgc2-gazebo-sim-scout (>= 0.4.9-25), ros-noetic-xgc2-multirotor-controller (>= 1.1.18-4), ros-noetic-xgc2-px4-multirotor-controller-msgs (>= 1.2.0-3), ros-noetic-xgc2-ugv-controller (>= 1.1.4-9), ros-noetic-xgc2-estimator-hover-thrust (>= 1.1.24-6), ros-noetic-xgc2-estimator-rigid-state (>= 1.1.6-6), ros-noetic-xgc2-estimator-rigid-state-msgs (>= 1.2.0-3), xgc2-vrpn-router (>= 0.1.0-4+focal)"

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
