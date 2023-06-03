#include <mujoco/mujoco.h>

#include <MujocoTactileSensorPlugin/TactileSensorRos.h>

#include <mujoco_tactile_sensor_plugin/TactileSensorData.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace mujoco::plugin::sensor
{

void TactileSensorRos::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoTactileSensorRosPlugin";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char * attributes[] = {"sensor_nums", "sensor_interval", "surface_radius",
                               "is_hex_grid", "topic_name",      "publish_rate"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int // plugin_id
                   ) { return 0; };

  plugin.nsensordata = +[](const mjModel * m, int plugin_id, int // sensor_id
                        )
  {
    const char * sensor_nums_char = mj_getPluginConfig(m, plugin_id, "sensor_nums");
    if(strlen(sensor_nums_char) == 0)
    {
      mju_error("[TactileSensorRos] `sensor_nums` is missing.");
      return 0;
    }
    std::vector<int> sensor_nums;
    readVector(sensor_nums, std::string(sensor_nums_char));
    if(sensor_nums.size() != 2)
    {
      mju_error("[TactileSensorRos] Size of `sensor_nums` must be 2.");
      return 0;
    }
    if(sensor_nums[0] <= 0 || sensor_nums[1] <= 0)
    {
      mju_error("[TactileSensorRos] All elements in `sensor_nums` must be positive.");
      return 0;
    }
    int sensor_total_num = sensor_nums[0] * sensor_nums[1];
    /// f: force, p: position, n: normal
    // (f_0, ..., f_N, px_0, py_0, pz_0, ..., px_N, py_N, pz_N, nx_0, ny_0, nz_0, ..., nx_N, ny_N, nz_N)
    return 7 * sensor_total_num;
  };

  // Can only run after forces have been computed
  plugin.needstage = mjSTAGE_ACC;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    auto * plugin_instance = TactileSensorRos::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<TactileSensorRos *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, // plugin_state
                     void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class TactileSensorRos *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int // capability_bit
                    )
  {
    auto * plugin_instance = reinterpret_cast<class TactileSensorRos *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  plugin.visualize = +[](const mjModel * m, mjData * d, const mjvOption * opt, mjvScene * scn, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class TactileSensorRos *>(d->plugin_data[plugin_id]);
    plugin_instance->visualize(m, d, opt, scn, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

TactileSensorRos * TactileSensorRos::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // sensor_nums
  const char * sensor_nums_char = mj_getPluginConfig(m, plugin_id, "sensor_nums");
  if(strlen(sensor_nums_char) == 0)
  {
    mju_error("[TactileSensorRos] `sensor_nums` is missing.");
    return nullptr;
  }
  std::vector<int> sensor_nums;
  readVector(sensor_nums, std::string(sensor_nums_char));
  if(sensor_nums.size() != 2)
  {
    mju_error("[TactileSensorRos] Size of `sensor_nums` must be 2.");
    return nullptr;
  }
  if(sensor_nums[0] <= 0 || sensor_nums[1] <= 0)
  {
    mju_error("[TactileSensorRos] All elements in `sensor_nums` must be positive.");
    return nullptr;
  }

  // sensor_interval
  const char * sensor_interval_char = mj_getPluginConfig(m, plugin_id, "sensor_interval");
  if(strlen(sensor_interval_char) == 0)
  {
    mju_error("[TactileSensorRos] `sensor_interval` is missing.");
    return nullptr;
  }
  mjtNum sensor_interval = strtod(sensor_interval_char, nullptr);
  if(sensor_interval <= 0)
  {
    mju_error("[TactileSensorRos] `sensor_interval` must be positive.");
    return nullptr;
  }

  // surface_radius
  const char * surface_radius_char = mj_getPluginConfig(m, plugin_id, "surface_radius");
  if(strlen(surface_radius_char) == 0)
  {
    mju_error("[TactileSensorRos] `surface_radius` is missing.");
  }
  mjtNum surface_radius = strtod(surface_radius_char, nullptr);
  if(surface_radius < 0)
  {
    mju_error("[TactileSensorRos] `surface_radius` must be nonnegative.");
    return nullptr;
  }

  // is_hex_grid
  const char * is_hex_grid_char = mj_getPluginConfig(m, plugin_id, "is_hex_grid");
  if(strlen(is_hex_grid_char) == 0)
  {
    mju_error("[TactileSensorRos] `is_hex_grid` is missing.");
    return nullptr;
  }
  if(!(strcmp(is_hex_grid_char, "true") == 0 || strcmp(is_hex_grid_char, "false") == 0))
  {
    mju_error("[TactileSensorRos] `is_hex_grid` must be `true` or `false`.");
    return nullptr;
  }
  bool is_hex_grid = (strcmp(is_hex_grid_char, "true") == 0);

  // topic_name
  const char * topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
  if(strlen(topic_name_char) == 0)
  {
    mju_error("[TactileSensorRos] `topic_name` is missing.");
    return nullptr;
  }
  std::string topic_name = std::string(topic_name_char);

  // publish_rate
  const char * publish_rate_char = mj_getPluginConfig(m, plugin_id, "publish_rate");
  if(strlen(publish_rate_char) == 0)
  {
    mju_error("[TactileSensorRos] `publish_rate` is missing.");
    return nullptr;
  }
  mjtNum publish_rate = strtod(publish_rate_char, nullptr);
  if(publish_rate <= 0)
  {
    mju_error("[TactileSensorRos] `publish_rate` must be positive.");
    return nullptr;
  }

  // Set sensor_id
  int sensor_id = 0;
  for(; sensor_id < m->nsensor; sensor_id++)
  {
    if(m->sensor_type[sensor_id] == mjSENS_PLUGIN && m->sensor_plugin[sensor_id] == plugin_id)
    {
      break;
    }
  }
  if(sensor_id == m->nsensor)
  {
    mju_error("[TactileSensorRos] Plugin not found in sensors.");
    return nullptr;
  }
  if(m->sensor_objtype[sensor_id] != mjOBJ_SITE)
  {
    mju_error("[TactileSensorRos] Plugin must be attached to a site.");
    return nullptr;
  }

  std::cout << "[TactileSensorRos] Create." << std::endl;

  return new TactileSensorRos(m, d, sensor_id, sensor_nums.data(), sensor_interval, surface_radius, is_hex_grid,
                              topic_name, publish_rate);
}

TactileSensorRos::TactileSensorRos(const mjModel * m,
                                   mjData * d,
                                   int sensor_id,
                                   int sensor_nums[2],
                                   mjtNum sensor_interval,
                                   mjtNum surface_radius,
                                   bool is_hex_grid,
                                   const std::string & topic_name,
                                   mjtNum publish_rate)
: TactileSensor(m, d, sensor_id, sensor_nums, sensor_interval, surface_radius, is_hex_grid),
  publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1))
{
  int argc = 0;
  char ** argv = nullptr;
  if(!ros::isInitialized())
  {
    ros::init(argc, argv, "mujoco_ros", ros::init_options::NoSigintHandler);
  }

  nh_ = std::make_shared<ros::NodeHandle>();
  pub_ = nh_->advertise<mujoco_tactile_sensor_plugin::TactileSensorData>(topic_name, 1);
}

void TactileSensorRos::compute(const mjModel * m, mjData * d, int plugin_id)
{
  TactileSensor::compute(m, d, plugin_id);

  sim_cnt_++;
  if(sim_cnt_ % publish_skip_ != 0)
  {
    return;
  }

  mujoco_tactile_sensor_plugin::TactileSensorData msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = std::string(mj_id2name(m, mjOBJ_SITE, site_id_));
  msg.forces.resize(sensor_total_num_);
  msg.positions.resize(sensor_total_num_);
  msg.normals.resize(sensor_total_num_);
  mjtNum * sensordata = d->sensordata + m->sensor_adr[sensor_id_];
  for(int sensor_idx = 0; sensor_idx < sensor_total_num_; sensor_idx++)
  {
    msg.forces[sensor_idx] = *(sensordata + sensor_idx);
    msg.positions[sensor_idx].x = *(sensordata + 1 * sensor_total_num_ + 3 * sensor_idx + 0);
    msg.positions[sensor_idx].y = *(sensordata + 1 * sensor_total_num_ + 3 * sensor_idx + 1);
    msg.positions[sensor_idx].z = *(sensordata + 1 * sensor_total_num_ + 3 * sensor_idx + 2);
    msg.normals[sensor_idx].x = *(sensordata + 4 * sensor_total_num_ + 3 * sensor_idx + 0);
    msg.normals[sensor_idx].y = *(sensordata + 4 * sensor_total_num_ + 3 * sensor_idx + 1);
    msg.normals[sensor_idx].z = *(sensordata + 4 * sensor_total_num_ + 3 * sensor_idx + 2);
  }
  pub_.publish(msg);
}

} // namespace mujoco::plugin::sensor
