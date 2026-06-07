cmake_minimum_required(VERSION 3.20)
project(balance)

include_directories(lib)

add_executable(torque_ctrl
    dev/torque.cpp
    src/constants.cpp
    src/PID.cpp
    lib/hat/pi3hat.cc
)

add_executable(bot
    src/main.cpp
    src/constants.cpp
    src/PID.cpp
    lib/hat/pi3hat.cc
)

add_executable(record_step
    dev/step_responce.cpp
    lib/hat/pi3hat.cc
)