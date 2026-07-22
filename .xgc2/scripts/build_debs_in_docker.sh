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
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
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

      if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
        sed "s#${XGC2_APT_BASE_URL:-https://xgc2.apt.xiaokang.ink}#${XGC2_APT_OVERLAY_URL%/}#g" \
          /etc/apt/sources.list.d/xgc2.list \
          > /etc/apt/sources.list.d/00-xgc2-release-train.list
      fi
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      dpkg-dev \
      fakeroot \
      file \
      git \
      libgazebo11-dev \
      netbase \
      rsync \
      ros-noetic-gazebo-msgs \
      ros-noetic-gazebo-ros \
      ros-noetic-geometry-msgs \
      ros-noetic-message-generation \
      ros-noetic-roscpp \
      ros-noetic-roslaunch \
      ros-noetic-rospack \
      ros-noetic-rostest \
      ros-noetic-rosunit \
      ros-noetic-std-msgs \
      ros-noetic-std-srvs \
      ros-noetic-tf2 \
      ros-noetic-tf2-ros
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
      apt-get install -y --no-install-recommends \
        /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb \
        /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb
      dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb \
        | grep -F /opt/ros/noetic/lib/libxgc2_gazebo_scene_system.so >/dev/null
      dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb \
        | grep -F /opt/ros/noetic/lib/libxgc2_gazebo_scene_motion.so >/dev/null
      dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb \
        | grep -F /opt/ros/noetic/share/xgc2_gazebo_scene/msg/ObstacleDefinition.msg >/dev/null
      dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb \
        | grep -F /opt/ros/noetic/lib/python3/dist-packages/xgc2_gazebo_scene/msg/_ObstacleDefinition.py >/dev/null
      dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb Depends \
        | grep -E "(^|, )libgazebo11( |\\()" >/dev/null
      for runtime_dependency in \
          ros-noetic-gazebo-ros \
          ros-noetic-geometry-msgs \
          ros-noetic-message-runtime \
          ros-noetic-rosconsole \
          ros-noetic-roscpp \
          ros-noetic-roscpp-serialization \
          ros-noetic-rostime \
          ros-noetic-std-msgs; do
        dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb Depends \
          | grep -E "(^|, )${runtime_dependency}( |\\(|,|$)" >/dev/null
      done
      if dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb Depends \
          | grep -E "(^|, )(cmake|gazebo-dev|libgazebo11-dev|ros-noetic-message-generation)( |\\(|,|$)"; then
        echo "Gazebo Scene package leaked a build-only dependency" >&2
        exit 1
      fi
      rm -rf /tmp/xgc2-gazebo-scene-control
      dpkg-deb -e /workspace/out/ros-noetic-xgc2-gazebo-scene_*.deb \
        /tmp/xgc2-gazebo-scene-control
      test ! -e /tmp/xgc2-gazebo-scene-control/shlibs
      test ! -e /tmp/xgc2-gazebo-scene-control/postinst
      test ! -e /tmp/xgc2-gazebo-scene-control/postrm
      dpkg-deb -c /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb \
        | grep -F /opt/ros/noetic/share/gazebo_sim_examples/launch/fs150_ugv_vrpn.launch >/dev/null
      dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb Depends \
        | grep -Fx "python3" >/dev/null
      if dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-sim-examples_*.deb Recommends \
          | grep -q .; then
        echo "Retired Gazebo examples package must not recommend a runtime closure" >&2
        exit 1
      fi
      /workspace/gazebo-sim/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
