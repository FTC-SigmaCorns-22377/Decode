#!/usr/bin/env bash
# Build turret_planner JNI shared library for the host machine (x86-64 Linux/macOS).
# Mirrors the pattern used by decode_estimator and mecanum_LTV_OCP.
#
# Usage:
#   scripts/build_host_jni.sh [--build-dir <dir>]
#
# Output:
#   <build-dir>/libturret_planner_jni.so   (Linux)
#   <build-dir>/libturret_planner_jni.dylib  (macOS)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build-host-jni"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build-dir) BUILD_DIR="$2"; shift 2 ;;
        *) echo "Unknown argument: $1" >&2; exit 1 ;;
    esac
done

mkdir -p "${BUILD_DIR}"

# JNI headers are located by CMake's FindJNI module (see CMakeLists.txt).
# It honors JAVA_HOME and scans common JDK install locations.
cmake \
    -S "${REPO_ROOT}" \
    -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DTURRET_PLANNER_JNI=ON

cmake --build "${BUILD_DIR}" --target turret_planner_jni -j"$(nproc 2>/dev/null || sysctl -n hw.logicalcpu)"

echo "Built: $(find "${BUILD_DIR}" -name 'libturret_planner_jni.*' | head -1)"
