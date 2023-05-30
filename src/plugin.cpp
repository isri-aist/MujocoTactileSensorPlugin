#include <mujoco/mjplugin.h>

#include "TactileSensor.h"

namespace mujoco::plugin::sensor
{

mjPLUGIN_LIB_INIT
{
  TactileSensor::RegisterPlugin();
}

} // namespace mujoco::plugin::sensor
