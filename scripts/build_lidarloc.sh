#!/usr/bin/env bash
# Fetches and builds lidar_localization_core (the registration of
# lidar_localization_ros2, without ROS 2) for adapters/lidarloc/rne_adapter_lidarloc.
#
# Usage: scripts/build_lidarloc.sh [prefix-dir]
# Default prefix: target/lidarloc (found by the adapter's build.rs without any
# environment variable); the library and headers are installed under
# <prefix>/install. Needs git, CMake >= 3.16, a C++17 compiler, PCL, Eigen 3 and
# OpenMP (Debian/Ubuntu: apt-get install libpcl-dev libeigen3-dev libomp-dev).
set -euo pipefail

REPO="https://github.com/rsasaki0109/lidar_localization_ros2"
# Pinned revision the adapter is tested against (local re-acquisition, the NDT
# Hessian and the map-distance prefilter in the C API).
REV="efbbe6600892154a35f65cfa1c51b8149da6bfa4"
NDT_OMP_REPO="https://github.com/rsasaki0109/ndt_omp_ros2"
NDT_OMP_BRANCH="humble"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="${1:-$ROOT/target/lidarloc}"

fetch() {
  local dir="$1" repo="$2" rev="$3"
  if [ ! -d "$dir/.git" ]; then
    mkdir -p "$dir"
    git -C "$dir" init -q
    git -C "$dir" remote add origin "$repo"
  fi
  git -C "$dir" fetch -q --depth 1 origin "$rev"
  git -C "$dir" checkout -q --force FETCH_HEAD
}

fetch "$PREFIX/lidar_localization_ros2" "$REPO" "$REV"
fetch "$PREFIX/ndt_omp_ros2" "$NDT_OMP_REPO" "$NDT_OMP_BRANCH"

cmake -S "$PREFIX/lidar_localization_ros2" -B "$PREFIX/build" \
  -DLIDAR_LOCALIZATION_CORE_ONLY=ON \
  -DNDT_OMP_ROS2_SOURCE_DIR="$PREFIX/ndt_omp_ros2" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build "$PREFIX/build" --parallel "${LIDARLOC_JOBS:-4}"
cmake --install "$PREFIX/build" --prefix "$PREFIX/install" > /dev/null

echo "lidar_localization_core installed in $PREFIX/install"
if [ "$PREFIX" != "$ROOT/target/lidarloc" ]; then
  echo "export RNE_LIDARLOC_DIR=$PREFIX/install"
fi
