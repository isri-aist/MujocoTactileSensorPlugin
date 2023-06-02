add_library(TactileSensor SHARED TactileSensor.cpp)
target_include_directories(
  TactileSensor PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
target_include_directories(TactileSensor PUBLIC ${MUJOCO_INCLUDE_DIR})
target_link_libraries(TactileSensor PUBLIC ${LIB_MUJOCO})
if(DEFINED CATKIN_DEVEL_PREFIX)
  target_include_directories(TactileSensor PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(TactileSensor PUBLIC ${catkin_LIBRARIES})
endif()

add_library(TactileSensorPlugin SHARED plugin.cpp)
target_link_libraries(TactileSensorPlugin PUBLIC TactileSensor)

install(
  TARGETS TactileSensor TactileSensorPlugin
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/MujocoTactileSensorPlugin
        DESTINATION "${INCLUDE_INSTALL_DIR}")
