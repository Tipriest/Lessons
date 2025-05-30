cmake_minimum_required(VERSION 3.10)

project(virtual_model_control)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-w) # warning ignore
add_compile_options(-fPIC)
set(BUILD_PLATFORM "x86" CACHE STRING "select build cpu type")
option(BUILD SIM OFF)
option(USE_RAISIM OFF)
set(USE_PYBULLET ON)
set(USE_RAISIM OFF)
option(SEND_REMOTE OFF)
if (BUILD_PLATFORM STREQUAL arm)
  message("this is arm platform")
  set(BUILD_SIM OFF)
  set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
else()
  message("this is x86 platform")
endif()
set(CMAKE_CXX_STANDARD 17)

if (SEND_REMOTE)
  set(BUILD_SIM OFF)
endif()

message("BUILD_PLATFORM: ${BUILD_PLATFORM}")
message("BUILD_SIM:      ${BUILD_SIM}")
message("SEND_REMOTE:    ${SEND_REMOTE}")

get_filename_component(WORKSPACE_DIR ./ ABSOLUTE)
set(THIRD_PARTY ${WORKSPACE_DIR}/third_party)

include_directories(
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/Lite3_MotionSDK/include
  types
  utils
  interface/robot
  interface/user_command
  state_machine
  run_policy
)

add_subdirectory(interface)

if (BUILD_SIM)
message("USE_PYBULLET:    ${USE_PYBULLET}")

  if(USE_RAISIM)
    add_definitions(-DUSE_RAISIM)
    set(raisim_DIR "your raisim cmake path")
    find_package(raisim CONFIG REQUIRED)
  elseif(USE_PYBULLET)
    add_definitions(-DUSE_PYBULLET)
  endif()
endif()

if (SEND_REMOTE)
  set(PATH_LIB "../lib")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
endif()

file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")

add_executable(vmc_stand main.cpp ${STATE_MACHINE_SRC} )
target_link_libraries(vmc_stand interface -lpthread -lm -lrt -ldl -lstdc++)

if (BUILD_SIM)
  if(USE_RAISIM)
    message("build simulation")
    target_link_libraries(vmc_stand raisim::raisim)
  endif()
endif()

if (SEND_REMOTE)
  add_custom_target(push_to_robot ALL DEPENDS vmc_stand)
  add_custom_command(
    TARGET push_to_robot POST_BUILD
    COMMAND ./scripts/sftp_to_remote.sh
    WORKING_DIRECTORY ..
    COMMENT "run sftp_to_remote.sh"
  )
endif()
