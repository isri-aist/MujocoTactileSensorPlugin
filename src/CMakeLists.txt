add_library(TactileSensor SHARED TactileSensor.h TactileSensor.cpp plugin.cpp)
target_include_directories(TactileSensor PUBLIC ${MUJOCO_INCLUDE_DIR})
target_link_libraries(TactileSensor PRIVATE ${LIB_MUJOCO})

add_library(TactileSensorPlugin SHARED plugin.cpp)
target_link_libraries(TactileSensorPlugin PRIVATE TactileSensor)
