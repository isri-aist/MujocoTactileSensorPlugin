add_library(TactileSensor SHARED TactileSensor.cpp)
target_include_directories(
  TactileSensor PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
target_include_directories(TactileSensor PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(TactileSensor PUBLIC ${LIB_MUJOCO})
if(DEFINED CATKIN_DEVEL_PREFIX)
  target_sources(TactileSensor PRIVATE TactileSensorRos.cpp)
  target_include_directories(TactileSensor PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(TactileSensor PUBLIC ${catkin_LIBRARIES})
  add_dependencies(TactileSensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_compile_definitions(ENABLE_ROS)
endif()

add_library(TactileSensorPlugin SHARED plugin.cpp)
target_include_directories(TactileSensorPlugin PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(TactileSensorPlugin PUBLIC TactileSensor)

if(DEFINED CATKIN_DEVEL_PREFIX)
  set(MUJOCO_PLUGIN_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
  set_target_properties(
    TactileSensorPlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                   ${MUJOCO_PLUGIN_OUTPUT_DIR})
  catkin_add_env_hooks(99.mujoco_tactile_sensor_plugin SHELLS sh DIRECTORY
                       ${PROJECT_SOURCE_DIR}/env-hooks)
endif()

install(
  TARGETS TactileSensor TactileSensorPlugin
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/MujocoTactileSensorPlugin
        DESTINATION "${INCLUDE_INSTALL_DIR}")
