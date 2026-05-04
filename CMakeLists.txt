cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME mujoco_tactile_sensor_plugin)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/MujocoTactileSensorPlugin)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT FALSE)
set(CXX_DISABLE_WERROR ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(${PROJECT_NAME})

# MuJoCo
if(NOT DEFINED MUJOCO_ROOT_DIR AND DEFINED ENV{MUJOCO_ROOT_DIR})
  set(MUJOCO_ROOT_DIR "$ENV{MUJOCO_ROOT_DIR}")
endif()
if(NOT DEFINED MUJOCO_ROOT_DIR)
  message(FATAL_ERROR "Set \"MUJOCO_ROOT_DIR\"")
elseif(NOT EXISTS ${MUJOCO_ROOT_DIR})
  message(FATAL_ERROR "\"MUJOCO_ROOT_DIR\" is invalid path: ${MUJOCO_ROOT_DIR}")
endif()
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_ROOT_DIR}/lib")
set(MUJOCO_BIN_DIR "${MUJOCO_ROOT_DIR}/bin")
set(MUJOCO_REQUIRED_VERSION 300)

set(MUJOCO_MAIN_HEADER "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
if(NOT EXISTS "${MUJOCO_MAIN_HEADER}")
  message(
    FATAL_ERROR
      "MuJoCo main header (${MUJOCO_MAIN_HEADER}) not found, maybe MUJOCO_ROOT_DIR is badly set"
  )
endif()
file(READ "${MUJOCO_MAIN_HEADER}" MUJOCO_MAIN_HEADER_CONTENT)
if(MUJOCO_MAIN_HEADER_CONTENT MATCHES ".*#define mjVERSION_HEADER ([0-9]+).*")
  set(MUJOCO_VERSION ${CMAKE_MATCH_1})
else()
  message(
    FATAL_ERROR
      "Could not extract MuJoCo version from the header, please contact this package developers"
  )
endif()
if(${MUJOCO_VERSION} LESS ${MUJOCO_REQUIRED_VERSION})
  message(
    FATAL_ERROR
      "This package requires at least MuJoCo version ${MUJOCO_REQUIRED_VERSION} but you have ${MUJOCO_VERSION}"
  )
endif()

file(GLOB LIB_MUJOCO ${MUJOCO_BIN_DIR}/libmujoco[0-9][0-9][0-9].so
     ${MUJOCO_LIB_DIR}/libmujoco.so.*)
if(EXISTS ${LIB_MUJOCO})
  message(STATUS "MuJoCo lib found: " ${LIB_MUJOCO})
else()
  message(FATAL_ERROR "MuJoCo lib not found.")
endif()

set(AMENT_CMAKE_UNINSTALL_TARGET OFF)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_eigen REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME} "msg/TactileSensorData.msg" DEPENDENCIES std_msgs
  geometry_msgs builtin_interfaces)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

add_subdirectory(src)
add_subdirectory(xml)
add_subdirectory(node)

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

ament_package()
