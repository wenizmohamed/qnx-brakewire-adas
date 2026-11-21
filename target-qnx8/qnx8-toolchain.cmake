# QNX Neutrino RTOS 8.0 Cross-Compilation Toolchain File
# Example:
#   source ~/qnx800/qnxsdp-env.sh
#   cmake -B build-x86_64 -DCMAKE_TOOLCHAIN_FILE=qnx8-toolchain.cmake -DQNX_ARCH=x86_64
#   cmake --build build-x86_64

set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_SYSTEM_VERSION 8.0)

if(NOT DEFINED QNX_ARCH)
    set(QNX_ARCH "x86_64" CACHE STRING "Target QNX architecture (x86_64 or aarch64le)")
endif()

set(CMAKE_SYSTEM_PROCESSOR ${QNX_ARCH})

if(NOT DEFINED ENV{QNX_HOST} OR NOT DEFINED ENV{QNX_TARGET})
    message(FATAL_ERROR "QNX_HOST and QNX_TARGET must be set. Run 'source ~/qnx800/qnxsdp-env.sh' first.")
endif()

set(QNX_HOST "$ENV{QNX_HOST}")
set(QNX_TARGET "$ENV{QNX_TARGET}")

set(CMAKE_C_COMPILER "${QNX_HOST}/usr/bin/qcc")
set(CMAKE_C_COMPILER_TARGET "gcc_nto${QNX_ARCH}")
set(CMAKE_CXX_COMPILER "${QNX_HOST}/usr/bin/q++")
set(CMAKE_CXX_COMPILER_TARGET "gcc_nto${QNX_ARCH}")

set(CMAKE_SYSROOT "${QNX_TARGET}")
set(CMAKE_FIND_ROOT_PATH "${QNX_TARGET}/${QNX_ARCH}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
