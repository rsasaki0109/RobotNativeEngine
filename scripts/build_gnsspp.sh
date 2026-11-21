#!/usr/bin/env bash
# Fetches and builds libgnss++ for adapters/gnsspp/rne_adapter_gnsspp.
#
# Usage: scripts/build_gnsspp.sh [checkout-dir]
# Default checkout: target/gnsspp/src (found by the adapter's build.rs without
# any environment variable). Needs git, CMake >= 3.14, a C++20 compiler and
# Eigen 3 (Debian/Ubuntu: apt-get install libeigen3-dev).
set -euo pipefail

REPO="https://github.com/rsasaki0109/gnssplusplus-library"
# Pinned revision the adapter is tested against.
REV="42bd4985ec5ae87aa66709e3c93274fcadfe9b36"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="${1:-$ROOT/target/gnsspp/src}"

if [ ! -d "$DIR/.git" ]; then
  mkdir -p "$DIR"
  git -C "$DIR" init -q
  git -C "$DIR" remote add origin "$REPO"
fi
git -C "$DIR" fetch -q --depth 1 origin "$REV"
GIT_LFS_SKIP_SMUDGE=1 git -C "$DIR" checkout -q --force FETCH_HEAD

cmake -S "$DIR" -B "$DIR/build" -DCMAKE_BUILD_TYPE=Release \
  -DGNSSPP_BUILD_PYTHON_BINDINGS=OFF -DBUILD_TESTING=OFF -DGNSSPP_USE_CHOLMOD=OFF
cmake --build "$DIR/build" --target gnss_rtk_runtime --parallel "${GNSSPP_JOBS:-4}"

echo "libgnss++ built in $DIR/build"
if [ "$DIR" != "$ROOT/target/gnsspp/src" ]; then
  echo "export RNE_GNSSPP_DIR=$DIR"
fi
