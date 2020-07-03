cmake_minimum_required(VERSION 3.13)
project(stmlibxx
    VERSION 1.0.0
    LANGUAGES CXX C ASM
)
add_library(${PROJECT_NAME} INTERFACE)
target_sources(${PROJECT_NAME}
    INTERFACE
        stm32f303xc_it.cpp
        startup_stm32f303xc.cpp
        syscalls.c
)
target_include_directories(${PROJECT_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)