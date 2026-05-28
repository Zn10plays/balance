cmake_minimum_required(VERSION 3.20)
project(balance)

include_directories(lib)
add_executable(bot
    src/main.cpp
    src/constants.h
)

