cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME mujoco_tactile_sensor_plugin)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/MujocoTactileSensorPlugin)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(${PROJECT_NAME} LANGUAGES CXX)

# MuJoCo
if(NOT DEFINED MUJOCO_ROOT_DIR)
  message(FATAL_ERROR "Set \"MUJOCO_ROOT_DIR\"")
elseif(NOT EXISTS ${MUJOCO_ROOT_DIR})
  message(FATAL_ERROR "\"MUJOCO_ROOT_DIR\" is invalid path: ${MUJOCO_ROOT_DIR}")
endif()
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_ROOT_DIR}/lib")
set(MUJOCO_BIN_DIR "${MUJOCO_ROOT_DIR}/bin")
file(GLOB LIB_MUJOCO ${MUJOCO_BIN_DIR}/libmujoco[0-9][0-9][0-9].so
     ${MUJOCO_LIB_DIR}/libmujoco.so.*)
if(EXISTS ${LIB_MUJOCO})
  message(STATUS "MuJoCo lib found: " ${LIB_MUJOCO})
else()
  message(FATAL_ERROR "MuJoCo lib not found.")
endif()

if(DEFINED CATKIN_DEVEL_PREFIX)
  set(DOXYGEN_HTML_OUTPUT html)

  find_package(
    catkin REQUIRED COMPONENTS message_generation roscpp geometry_msgs
                               visualization_msgs eigen_conversions)

  add_message_files(FILES TactileSensorData.msg)

  generate_messages(DEPENDENCIES geometry_msgs)

  catkin_package(
    CATKIN_DEPENDS
    message_runtime
    roscpp
    geometry_msgs
    visualization_msgs
    eigen_conversions
    INCLUDE_DIRS
    include
    LIBRARIES
    TactileSensor
    TactileSensorPlugin)
else()
  set(DOXYGEN_HTML_OUTPUT doxygen-html)
endif()

add_subdirectory(src)
add_subdirectory(xml)

if(DEFINED CATKIN_DEVEL_PREFIX)
  add_subdirectory(node)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
