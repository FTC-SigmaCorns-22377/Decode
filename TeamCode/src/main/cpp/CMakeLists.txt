cmake_minimum_required(VERSION 3.10)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# IMPORTANT: You must set CMAKE_PREFIX_PATH to the location of your Drake installation.
# e.g. -DCMAKE_PREFIX_PATH=/opt/drake

project(drake_sim_jni)

# Try to auto-detect Drake if the prefix/path isn't provided.
if(NOT drake_DIR AND NOT CMAKE_PREFIX_PATH)
    if(DEFINED ENV{DRAKE_DIR})
        set(drake_DIR "$ENV{DRAKE_DIR}")
    elseif(DEFINED ENV{DRAKE_PREFIX})
        list(APPEND CMAKE_PREFIX_PATH "$ENV{DRAKE_PREFIX}")
    elseif(DEFINED ENV{DRAKE_HOME})
        list(APPEND CMAKE_PREFIX_PATH "$ENV{DRAKE_HOME}")
    else()
        find_program(DRAKE_EXECUTABLE drake)
        if(DRAKE_EXECUTABLE)
            get_filename_component(_drake_bin_dir "${DRAKE_EXECUTABLE}" DIRECTORY)
            get_filename_component(_drake_prefix "${_drake_bin_dir}" DIRECTORY)
            list(APPEND CMAKE_PREFIX_PATH "${_drake_prefix}")
        endif()
    endif()
endif()

if(NOT drake_DIR AND NOT CMAKE_PREFIX_PATH)
    if(EXISTS "/opt/drake/lib/cmake/drake/drake-config.cmake")
        set(drake_DIR "/opt/drake/lib/cmake/drake")
    endif()
endif()

option(ENABLE_DRAKE "Enable Drake integration" ON)

if(ANDROID)
    set(ENABLE_DRAKE OFF)
endif()

if(ENABLE_DRAKE)
    find_package(drake CONFIG REQUIRED)

    if(NOT ANDROID)
        find_package(JNI REQUIRED)
    endif()

    add_library(native-lib SHARED
                native-lib.cpp
                DrakeSim.cpp)
    target_compile_definitions(native-lib PRIVATE USE_DRAKE=1)

    if(NOT ANDROID)
        target_link_libraries(native-lib JNI::JNI)
    endif()

    # Link against Drake
    target_link_libraries(native-lib
                          drake::drake)
else()
    if(NOT ANDROID)
        find_package(JNI REQUIRED)
    endif()

    add_library(native-lib SHARED
                native-lib.cpp
                DrakeSimStub.cpp)
    target_compile_definitions(native-lib PRIVATE USE_DRAKE=0)

    if(NOT ANDROID)
        target_link_libraries(native-lib JNI::JNI)
    endif()
endif()
