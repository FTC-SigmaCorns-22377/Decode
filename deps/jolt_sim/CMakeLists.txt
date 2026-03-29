cmake_minimum_required(VERSION 3.16)
project(jolt_sim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

option(JOLT_BUILD_JNI "Build JNI shared library (requires JDK)" OFF)

include(FetchContent)

# Jolt Physics
FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG        v5.3.0
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  Build
)

# Configure Jolt options before making it available
set(DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
set(GENERATE_DEBUG_SYMBOLS OFF CACHE BOOL "" FORCE)
set(CROSS_PLATFORM_DETERMINISTIC OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
set(USE_SSE4_1 ON CACHE BOOL "" FORCE)
set(USE_SSE4_2 ON CACHE BOOL "" FORCE)
set(USE_AVX ON CACHE BOOL "" FORCE)
set(USE_AVX2 ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(JoltPhysics)

# ---------------------------------------------------------------------------
# Static library
# ---------------------------------------------------------------------------
add_library(jolt_sim STATIC
    src/jolt_world.cpp
)
target_include_directories(jolt_sim PUBLIC src/)
target_link_libraries(jolt_sim PUBLIC Jolt)

# ---------------------------------------------------------------------------
# JNI shared library
# ---------------------------------------------------------------------------
if(JOLT_BUILD_JNI)
    find_package(JNI REQUIRED)
    add_library(jolt_sim_jni SHARED src/jolt_sim_jni.cpp)
    target_link_libraries(jolt_sim_jni PRIVATE jolt_sim JNI::JNI)
    if(APPLE)
        target_link_options(jolt_sim_jni PRIVATE -Wl,-dead_strip)
    elseif(UNIX)
        target_link_options(jolt_sim_jni PRIVATE -Wl,--gc-sections)
    endif()
endif()
