add_executable(MarkerPublisher MarkerPublisher.cpp)
target_include_directories(MarkerPublisher PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(MarkerPublisher ${catkin_LIBRARIES})
add_dependencies(MarkerPublisher ${${PROJECT_NAME}_EXPORTED_TARGETS})
