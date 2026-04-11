#!/usr/bin/env bash
# Build turret_planner JNI shared library for Android arm64-v8a (Cortex-A53).
# Matches the pattern used by decode_estimator and mecanum_LTV_OCP.
#
# Usage:
#   scripts/build_android_cortex_a53.sh --ndk <NDK_DIR> [--build-dir <dir>]
#
# Output:
#   <build-dir>/libturret_planner_jni.so  (copy to jniLibs/arm64-v8a/)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build-android-a53"
NDK_DIR=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ndk)       NDK_DIR="$2";   shift 2 ;;
        --build-dir) BUILD_DIR="$2"; shift 2 ;;
        *) echo "Unknown argument: $1" >&2; exit 1 ;;
    esac
done

if [[ -z "${NDK_DIR}" ]]; then
    echo "Error: --ndk <NDK_DIR> is required" >&2
    exit 1
fi

if [[ ! -d "${NDK_DIR}" ]]; then
    echo "Error: NDK directory does not exist: ${NDK_DIR}" >&2
    exit 1
fi

TOOLCHAIN="${NDK_DIR}/build/cmake/android.toolchain.cmake"
if [[ ! -f "${TOOLCHAIN}" ]]; then
    echo "Error: Android toolchain not found at ${TOOLCHAIN}" >&2
    exit 1
fi

mkdir -p "${BUILD_DIR}"

cmake \
    -S "${REPO_ROOT}" \
    -B "${BUILD_DIR}" \
    -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
    -DANDROID_ABI="arm64-v8a" \
    -DANDROID_PLATFORM="android-24" \
    -DANDROID_STL="c++_static" \
    -DCMAKE_BUILD_TYPE=Release \
    -DTURRET_PLANNER_ARM=ON \
    -DTURRET_PLANNER_JNI=ON

cmake --build "${BUILD_DIR}" --target turret_planner_jni -j"$(nproc 2>/dev/null || echo 4)"

LIB="$(find "${BUILD_DIR}" -name 'libturret_planner_jni.so' | head -1)"
echo "Built: ${LIB}"
echo "Copy to: jniLibs/arm64-v8a/libturret_planner_jni.so"
