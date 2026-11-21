cmake_minimum_required(VERSION 3.20)
project(qnx_brakewire_adas C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_compile_options(-O2 -Wall -Wextra -D_QNX_SOURCE)

add_executable(brakewire_adas
    brakewire_adas.c
    brakewire_ipc.h
)
target_link_libraries(brakewire_adas PRIVATE slog2 m)

add_executable(canfd_sensor_rx
    canfd_sensor_rx.c
    brakewire_ipc.h
)
target_link_libraries(canfd_sensor_rx PRIVATE slog2 m)

install(TARGETS brakewire_adas canfd_sensor_rx
        RUNTIME DESTINATION bin)
