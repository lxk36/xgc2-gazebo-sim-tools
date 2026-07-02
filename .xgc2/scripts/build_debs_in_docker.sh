#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -v "${REPO_ROOT}:/workspace/gazebo-sim:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends ca-certificates
    echo "deb [trusted=yes arch=$(dpkg --print-architecture)] https://xgc2.apt.xiaokang.ink focal main" \
      > /etc/apt/sources.list.d/xgc2.list
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      dpkg-dev \
      fakeroot \
      file \
      git \
      libxgc2-math-dev \
      netbase \
      rsync \
      ros-noetic-controller-manager-msgs \
      ros-noetic-gazebo-msgs \
      ros-noetic-gazebo-ros \
      ros-noetic-geometry-msgs \
      ros-noetic-mavros-msgs \
      ros-noetic-nav-msgs \
      ros-noetic-roslaunch \
      ros-noetic-rosnode \
      ros-noetic-rospack \
      ros-noetic-rostest \
      ros-noetic-rosunit \
      ros-noetic-rospy \
      ros-noetic-std-srvs \
      ros-noetic-tf2 \
      ros-noetic-tf2-ros \
      ros-noetic-vrpn \
      ros-noetic-vrpn-client-ros \
      ros-noetic-xgc2-gazebo-sim-visualization \
      ros-noetic-xgc2-gazebo-sim-vrpn-bridge \
      ros-noetic-xgc2-multirotor-controller \
      ros-noetic-xgc2-ugv-controller \
      ros-noetic-xgc2-estimator-hover-thrust \
      ros-noetic-xgc2-estimator-rigid-state \
      ros-noetic-xgc2-gazebo-sim-worlds
    dpkg --compare-versions "$(dpkg-query -W -f="\${Version}" libxgc2-math-dev)" ge 0.5.4-1

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/xgc2_gazebo_sim_tools
    rsync -a --delete /workspace/gazebo-sim/ /workspace/work/src/xgc2_gazebo_sim_tools/

    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCATKIN_ENABLE_TESTING=OFF

    /workspace/gazebo-sim/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y \
        /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb \
        /workspace/out/ros-noetic-xgc2-gazebo-sim-manager_*.deb
      dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb \
        | grep -F /opt/ros/noetic/share/gazebo_sim_examples/launch/fs150_ugv_vrpn.launch >/dev/null
      dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb Depends \
        | grep -F "ros-noetic-xgc2-gazebo-sim-worlds (>= 1.1.0-7)" >/dev/null
      dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb Depends \
        | grep -F "ros-noetic-xgc2-multirotor-controller (>= 1.1.15-6)" >/dev/null
      dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb Depends \
        | grep -F "ros-noetic-xgc2-ugv-controller (>= 1.1.1-6)" >/dev/null
      dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb Depends \
        | grep -F "ros-noetic-xgc2-estimator-rigid-state (>= 1.1.3-6)" >/dev/null
      dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-sim-manager_*.deb Depends \
        | grep -F "ros-noetic-xgc2-gazebo-sim-vrpn-bridge (>= 1.1.0-6)" >/dev/null
      dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb Depends \
        | grep -F "ros-noetic-xgc2-gazebo-sim-visualization (>= 1.1.0-6)" >/dev/null
      /workspace/gazebo-sim/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
