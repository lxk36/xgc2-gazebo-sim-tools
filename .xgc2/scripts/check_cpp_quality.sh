#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
cd "${repo_root}"

echo "No C++ packages are owned by xgc2-gazebo-sim-tools."
echo "C++ quality checks run in the split visualization and VRPN bridge repositories."
