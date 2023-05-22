add_library(MujocoTactileSensorPlugin SHARED
  MujocoTactileSensorPlugin.h
  MujocoTactileSensorPlugin.cpp
)
target_include_directories(MujocoTactileSensorPlugin PRIVATE
  ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(MujocoTactileSensorPlugin PRIVATE
  ${LIB_MUJOCO}
)
