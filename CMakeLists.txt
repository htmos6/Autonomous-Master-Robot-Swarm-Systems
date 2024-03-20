# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/usr/share/pico-sdk")

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(ProjectName3 C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ProjectName3 ProjectName3.cpp lwipopts.h
               TCP_Server.hpp TCP_Server.cpp
               Motor_Driver.hpp Motor_Driver.cpp
               Encoder.hpp Encoder.cpp
               Odometry.hpp Odometry.cpp
               SpeedEstimator.hpp SpeedEstimator.cpp
               StateControl.hpp StateControl.cpp
               GoToGoal.hpp GoToGoal.cpp
               BroadcastIP.hpp BroadcastIP.cpp
               Pure_Pursuit.hpp Pure_Pursuit.cpp
               IMU.hpp IMU.cpp
               IMU_calib.hpp IMU_calib.cpp)

pico_set_program_name(ProjectName3 "ProjectName3")
pico_set_program_version(ProjectName3 "0.1")

pico_enable_stdio_uart(ProjectName3 1)
pico_enable_stdio_usb(ProjectName3 0)

# Add the standard library to the build
target_link_libraries(ProjectName3
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(ProjectName3 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(ProjectName3
        hardware_timer
        hardware_clocks
        pico_cyw43_arch_lwip_threadsafe_background
        hardware_pwm
        pico_time
        hardware_i2c
        pico_multicore
        )

pico_add_extra_outputs(ProjectName3)

