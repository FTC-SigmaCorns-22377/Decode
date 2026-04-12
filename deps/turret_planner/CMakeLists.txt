cmake_minimum_required(VERSION 3.16)
project(turret_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(TURRET_PLANNER_ARM "Build for ARM Cortex-A53 (cross-compile)" OFF)

if(TURRET_PLANNER_ARM)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a53 -mtune=cortex-a53")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -fno-exceptions -fno-rtti")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections -fvisibility=hidden")
else()
    # Host build for testing — aggressive but no exceptions/rtti still
    if(MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native -ffast-math -fno-exceptions -fno-rtti")
    endif()
endif()

# LTO for cross-module inlining (reduces call overhead significantly)
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(turret_planner STATIC
    src/ballistics.cpp
    src/flight_time.cpp
    src/robust_shot.cpp
    src/robust_3shot.cpp
    src/path_scan.cpp
    src/preposition.cpp
    src/zone_tracker.cpp
    src/flywheel_model.cpp
    src/mecanum_model.cpp
)
target_include_directories(turret_planner PUBLIC include/)

# ---------------------------------------------------------------------------
# JNI shared library (Android or host testing)
# ---------------------------------------------------------------------------
option(TURRET_PLANNER_JNI "Build JNI shared library (requires JNI headers)" OFF)

if(TURRET_PLANNER_JNI)
    add_library(turret_planner_jni SHARED jni/turret_planner_jni.cpp)
    target_link_libraries(turret_planner_jni PRIVATE turret_planner)

    # On Android the NDK sysroot provides jni.h on the default include path.
    # On a host build we locate a JDK via CMake's FindJNI module, which
    # searches JAVA_HOME, common /usr/lib/jvm/* layouts, Homebrew, etc.
    if(NOT ANDROID)
        find_package(JNI REQUIRED)
        target_include_directories(turret_planner_jni PRIVATE ${JNI_INCLUDE_DIRS})
    endif()

    # -fno-rtti is already in the flags from the ARM/host blocks above.
    # JNI requires C++ but does not need exceptions or RTTI.
    # Strip symbols on release to minimise .so size.
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        target_link_options(turret_planner_jni PRIVATE -Wl,--strip-all)
    endif()
endif()

# ---------------------------------------------------------------------------
# Tests (host only)
# ---------------------------------------------------------------------------
enable_testing()

add_executable(test_ballistics  tests/test_ballistics.cpp)
target_link_libraries(test_ballistics  turret_planner)
add_test(NAME ballistics  COMMAND test_ballistics)

add_executable(test_flight_time tests/test_flight_time.cpp)
target_link_libraries(test_flight_time turret_planner)
add_test(NAME flight_time COMMAND test_flight_time)

add_executable(test_robust_shot tests/test_robust_shot.cpp)
target_link_libraries(test_robust_shot turret_planner)
add_test(NAME robust_shot COMMAND test_robust_shot)

add_executable(test_robust_3shot tests/test_robust_3shot.cpp)
target_link_libraries(test_robust_3shot turret_planner)
add_test(NAME robust_3shot COMMAND test_robust_3shot)

add_executable(test_path_scan tests/test_path_scan.cpp)
target_link_libraries(test_path_scan turret_planner)
add_test(NAME path_scan COMMAND test_path_scan)

add_executable(test_preposition tests/test_preposition.cpp)
target_link_libraries(test_preposition turret_planner)
add_test(NAME preposition COMMAND test_preposition)

add_executable(test_zone_tracker tests/test_zone_tracker.cpp)
target_link_libraries(test_zone_tracker turret_planner)
add_test(NAME zone_tracker COMMAND test_zone_tracker)
