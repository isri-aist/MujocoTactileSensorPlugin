add_library(TactileSensor SHARED TactileSensor.cpp)
target_include_directories(
  TactileSensor PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
target_include_directories(TactileSensor PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(TactileSensor PUBLIC ${LIB_MUJOCO})
# if(DEFINED ENV{COLCON_CURRENT_PREFIX})
#   target_sources(TactileSensor PRIVATE TactileSensorRos.cpp)
#   target_include_directories(TactileSensor PUBLIC ${catkin_INCLUDE_DIRS})
#   target_link_libraries(TactileSensor PUBLIC ${catkin_LIBRARIES})
#   add_dependencies(TactileSensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
#   add_compile_definitions(ENABLE_ROS)
# endif()

add_library(TactileSensorPlugin SHARED plugin.cpp)
target_include_directories(TactileSensorPlugin PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(TactileSensorPlugin PUBLIC TactileSensor)

install(
  TARGETS TactileSensor TactileSensorPlugin
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/MujocoTactileSensorPlugin
        DESTINATION "${INCLUDE_INSTALL_DIR}")
