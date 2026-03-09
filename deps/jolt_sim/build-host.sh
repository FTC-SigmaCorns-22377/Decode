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

cmake --build . --parallel "$(nproc)"

# Copy the shared library to test jniLibs
mkdir -p "${OUTPUT_DIR}"
cp libjolt_sim_jni.so "${OUTPUT_DIR}/"

echo "Built and copied libjolt_sim_jni.so to ${OUTPUT_DIR}/"
