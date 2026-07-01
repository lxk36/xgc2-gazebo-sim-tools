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

  local pkg_root="${BUILD_DIR}/${package}"
  rm -rf "${pkg_root}"
  mkdir -p "${pkg_root}"

  copy_ros_package_paths "${ros_pkg}" "${pkg_root}"
  write_control "${pkg_root}" "${package}" "${depends}" "${description}"
  fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

manager_pkg="ros-noetic-xgc2-gazebo-sim-manager"
examples_pkg="ros-noetic-xgc2-gazebo-sim-examples"
visualization_pkg="ros-noetic-xgc2-gazebo-sim-visualization"
vrpn_bridge_pkg="ros-noetic-xgc2-gazebo-sim-vrpn-bridge"
worlds_pkg="ros-noetic-xgc2-gazebo-sim-worlds"

build_ros_package_deb \
  "${vrpn_bridge_pkg}" \
  "gazebo_sim_vrpn_bridge" \
  "libxgc2-math-dev (>= 0.5.1-1), ros-noetic-roscpp, ros-noetic-gazebo-msgs, ros-noetic-geometry-msgs, ros-noetic-tf2, ros-noetic-tf2-ros, ros-noetic-vrpn, ros-noetic-vrpn-client-ros" \
  "XGC2 Gazebo Classic model pose to VRPN tracker server bridge"

build_ros_package_deb \
  "${manager_pkg}" \
  "gazebo_session_manager" \
  "${vrpn_bridge_pkg} (= ${VERSION}), ${worlds_pkg} (>= 1.0.21-1), ros-noetic-rospy, ros-noetic-roslaunch, ros-noetic-rosnode, ros-noetic-gazebo-msgs, ros-noetic-gazebo-ros, ros-noetic-geometry-msgs, ros-noetic-controller-manager-msgs, ros-noetic-std-srvs" \
  "XGC2 Gazebo Classic session manager and WebUI tools"

build_ros_package_deb \
  "${examples_pkg}" \
  "gazebo_sim_examples" \
  "${vrpn_bridge_pkg} (= ${VERSION}), ${worlds_pkg} (>= 1.0.21-1), ros-noetic-xgc2-gazebo-sim-fs150-sitl, ros-noetic-xgc2-gazebo-sim-scout, ros-noetic-xgc2-multirotor-controller (>= 1.0.14-1), ros-noetic-xgc2-ugv-controller (>= 1.0.1-1), ros-noetic-xgc2-estimator-hover-thrust (>= 1.1.22-1), ros-noetic-xgc2-estimator-rigid-state (>= 1.1.2-1), ros-noetic-xgc2-vrpn-router, ros-noetic-vrpn-client-ros, ros-noetic-mavros, ros-noetic-mavros-msgs, ros-noetic-geometry-msgs, ros-noetic-nav-msgs, ros-noetic-rospy, ros-noetic-roslaunch" \
  "XGC2 Gazebo Classic example launch orchestration"

build_ros_package_deb \
  "${visualization_pkg}" \
  "gazebo_sim_visualization" \
  "${vrpn_bridge_pkg} (= ${VERSION}), ros-noetic-xgc2-gazebo-sim-fs150-sitl (>= 1.0.14-1), ros-noetic-xgc2-multirotor-controller (>= 1.0.14-1), ros-noetic-geometry-msgs, ros-noetic-nav-msgs, ros-noetic-rospy, ros-noetic-roslaunch, ros-noetic-std-msgs, ros-noetic-tf2-ros, ros-noetic-visualization-msgs, ros-noetic-robot-state-publisher, ros-noetic-rviz" \
  "XGC2 Gazebo Classic RViz visualization tools"

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
