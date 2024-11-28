add_library(TactileSensor SHARED TactileSensor.cpp)
target_include_directories(
  TactileSensor PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
target_include_directories(TactileSensor PRIVATE ${MUJOCO_INCLUDE_DIR})
if(DEFINED USE_ROS2)
  target_sources(TactileSensor PRIVATE TactileSensorRos.cpp)
  ament_target_dependencies(TactileSensor PUBLIC 
    rclcpp std_msgs geometry_msgs visualization_msgs
  )
  add_dependencies(TactileSensor ${PROJECT_NAME})
  rosidl_get_typesupport_target(cpp_typesupport_target 
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(TactileSensor PUBLIC ${cpp_typesupport_target} ${ament_LIBRARIES})
  target_include_directories(TactileSensor PUBLIC ${ament_INCLUDE_DIRS})
endif()
target_link_libraries(TactileSensor PUBLIC ${LIB_MUJOCO})

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
