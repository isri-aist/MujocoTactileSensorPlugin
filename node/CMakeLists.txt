add_executable(MarkerPublisher MarkerPublisher.cpp)
ament_target_dependencies(MarkerPublisher PUBLIC 
  rclcpp visualization_msgs tf2_eigen
)
add_dependencies(MarkerPublisher ${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(MarkerPublisher PUBLIC ${cpp_typesupport_target})
target_include_directories(MarkerPublisher PUBLIC ${ament_INCLUDE_DIRS})
target_link_libraries(MarkerPublisher PUBLIC ${cpp_typesupport_target} ${ament_LIBRARIES})
install(TARGETS
  MarkerPublisher
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)