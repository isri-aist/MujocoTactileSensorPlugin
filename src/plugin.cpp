#include <mujoco/mjplugin.h>

#include <MujocoTactileSensorPlugin/TactileSensor.h>
#if ENABLE_ROS
#  include <MujocoTactileSensorPlugin/TactileSensorRos.h>
#endif

mjPLUGIN_LIB_INIT
{
  mujoco::plugin::sensor::TactileSensor::RegisterPlugin();
#if ENABLE_ROS
  mujoco::plugin::sensor::TactileSensorRos::RegisterPlugin();
#endif
}
