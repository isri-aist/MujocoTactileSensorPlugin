add_library(TactileSensor SHARED TactileSensor.cpp plugin.cpp)
target_include_directories(TactileSensor PUBLIC ${PROJECT_SOURCE_DIR}/include
                                                ${MUJOCO_INCLUDE_DIR})
target_link_libraries(TactileSensor PRIVATE ${LIB_MUJOCO})

add_library(TactileSensorPlugin SHARED plugin.cpp)
target_link_libraries(TactileSensorPlugin PRIVATE TactileSensor)
