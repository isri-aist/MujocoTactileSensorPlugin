#include <mujoco/mjplugin.h>

#include <MujocoTactileSensorPlugin/TactileSensor.h>
#if ENABLE_ROS
#  include <MujocoTactileSensorPlugin/TactileSensorRos.h>
#endif

namespace mujoco::plugin::sensor
{

mjPLUGIN_LIB_INIT
{
  TactileSensor::RegisterPlugin();
#if ENABLE_ROS
  TactileSensorRos::RegisterPlugin();
#endif
}

} // namespace mujoco::plugin::sensor
