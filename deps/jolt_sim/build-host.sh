#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build-host"
OUTPUT_DIR="${SCRIPT_DIR}/../../TeamCode/src/test/jniLibs"

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake "${SCRIPT_DIR}" \
    -DJOLT_BUILD_JNI=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON

if command -v nproc >/dev/null 2>&1; then
    PARALLEL_JOBS="$(nproc)"
else
    PARALLEL_JOBS="$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
fi

cmake --build . --parallel "${PARALLEL_JOBS}"

# Copy the shared library to test jniLibs
mkdir -p "${OUTPUT_DIR}"
if [[ "$(uname)" == "Darwin" ]]; then
    LIB_NAME="libjolt_sim_jni.dylib"
else
    LIB_NAME="libjolt_sim_jni.so"
fi
cp "${LIB_NAME}" "${OUTPUT_DIR}/"

echo "Built and copied ${LIB_NAME} to ${OUTPUT_DIR}/"
