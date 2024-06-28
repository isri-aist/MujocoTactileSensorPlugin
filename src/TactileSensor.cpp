#include <mujoco/mujoco.h>

#include <MujocoTactileSensorPlugin/TactileSensor.h>

#include <iostream>

namespace mujoco::plugin::sensor
{

void TactileSensor::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoTactileSensorPlugin";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char * attributes[] = {"sensor_nums", "sensor_interval", "surface_radius", "is_hex_grid"};
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
      mju_error("[TactileSensor] `sensor_nums` is missing.");
      return 0;
    }
    std::vector<int> sensor_nums;
    readVector(sensor_nums, std::string(sensor_nums_char));
    if(sensor_nums.size() != 2)
    {
      mju_error("[TactileSensor] Size of `sensor_nums` must be 2.");
      return 0;
    }
    if(sensor_nums[0] <= 0 || sensor_nums[1] <= 0)
    {
      mju_error("[TactileSensor] All elements in `sensor_nums` must be positive.");
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
    auto * plugin_instance = TactileSensor::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<TactileSensor *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, // plugin_state
                     void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class TactileSensor *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int // capability_bit
                    )
  {
    auto * plugin_instance = reinterpret_cast<class TactileSensor *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  plugin.visualize = +[](const mjModel * m, mjData * d, const mjvOption * opt, mjvScene * scn, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class TactileSensor *>(d->plugin_data[plugin_id]);
    plugin_instance->visualize(m, d, opt, scn, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

TactileSensor * TactileSensor::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // sensor_nums
  const char * sensor_nums_char = mj_getPluginConfig(m, plugin_id, "sensor_nums");
  if(strlen(sensor_nums_char) == 0)
  {
    mju_error("[TactileSensor] `sensor_nums` is missing.");
    return nullptr;
  }
  std::vector<int> sensor_nums;
  readVector(sensor_nums, std::string(sensor_nums_char));
  if(sensor_nums.size() != 2)
  {
    mju_error("[TactileSensor] Size of `sensor_nums` must be 2.");
    return nullptr;
  }
  if(sensor_nums[0] <= 0 || sensor_nums[1] <= 0)
  {
    mju_error("[TactileSensor] All elements in `sensor_nums` must be positive.");
    return nullptr;
  }

  // sensor_interval
  const char * sensor_interval_char = mj_getPluginConfig(m, plugin_id, "sensor_interval");
  if(strlen(sensor_interval_char) == 0)
  {
    mju_error("[TactileSensor] `sensor_interval` is missing.");
    return nullptr;
  }
  mjtNum sensor_interval = strtod(sensor_interval_char, nullptr);
  if(sensor_interval <= 0)
  {
    mju_error("[TactileSensor] `sensor_interval` must be positive.");
    return nullptr;
  }

  // surface_radius
  const char * surface_radius_char = mj_getPluginConfig(m, plugin_id, "surface_radius");
  if(strlen(surface_radius_char) == 0)
  {
    mju_error("[TactileSensor] `surface_radius` is missing.");
  }
  mjtNum surface_radius = strtod(surface_radius_char, nullptr);
  if(surface_radius < 0)
  {
    mju_error("[TactileSensor] `surface_radius` must be nonnegative.");
    return nullptr;
  }

  // is_hex_grid
  const char * is_hex_grid_char = mj_getPluginConfig(m, plugin_id, "is_hex_grid");
  if(strlen(is_hex_grid_char) == 0)
  {
    mju_error("[TactileSensor] `is_hex_grid` is missing.");
    return nullptr;
  }
  if(!(strcmp(is_hex_grid_char, "true") == 0 || strcmp(is_hex_grid_char, "false") == 0))
  {
    mju_error("[TactileSensor] `is_hex_grid` must be `true` or `false`.");
    return nullptr;
  }
  bool is_hex_grid = (strcmp(is_hex_grid_char, "true") == 0);

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
    mju_error("[TactileSensor] Plugin not found in sensors.");
    return nullptr;
  }
  if(m->sensor_objtype[sensor_id] != mjOBJ_SITE)
  {
    mju_error("[TactileSensor] Plugin must be attached to a site.");
    return nullptr;
  }

  std::cout << "[TactileSensor] Create." << std::endl;

  return new TactileSensor(m, d, sensor_id, sensor_nums.data(), sensor_interval, surface_radius, is_hex_grid);
}

TactileSensor::TactileSensor(const mjModel * m,
                             mjData *, // d
                             int sensor_id,
                             int sensor_nums[2],
                             mjtNum sensor_interval,
                             mjtNum surface_radius,
                             bool is_hex_grid)
: sensor_id_(sensor_id), site_id_(m->sensor_objid[sensor_id]), sensor_total_num_(sensor_nums[0] * sensor_nums[1]),
  sensor_interval_(sensor_interval), surface_type_(surface_radius == 0 ? SurfacePlane : SurfaceCylinder),
  grid_type_(is_hex_grid ? GridHex : GridSquare)
{
  // Set sensor_pos_list_ and sensor_normal_list_
  sensor_pos_list_ = static_cast<mjtNum *>(mju_malloc(3 * sensor_total_num_ * sizeof(mjtNum)));
  sensor_normal_list_ = static_cast<mjtNum *>(mju_malloc(3 * sensor_total_num_ * sizeof(mjtNum)));
  for(int sensor_i = 0; sensor_i < sensor_nums[0]; sensor_i++)
  {
    for(int sensor_j = 0; sensor_j < sensor_nums[1]; sensor_j++)
    {
      int sensor_idx = sensor_i * sensor_nums[1] + sensor_j;
      mjtNum sensor_i_offset = 0;
      if(grid_type_ == GridHex)
      {
        sensor_i_offset = (sensor_j % 2 == 0 ? 0.25 : -0.25);
      }
      mjtNum dist_x = (sensor_i - 0.5 * (sensor_nums[0] - 1) + sensor_i_offset) * sensor_interval_;
      mjtNum dist_y = (sensor_j - 0.5 * (sensor_nums[1] - 1)) * sensor_interval_;

      if(surface_type_ == SurfacePlane)
      {
        sensor_pos_list_[3 * sensor_idx + 0] = dist_x;
        sensor_pos_list_[3 * sensor_idx + 1] = dist_y;
        sensor_pos_list_[3 * sensor_idx + 2] = 0.0;
        sensor_normal_list_[3 * sensor_idx + 0] = 0.0;
        sensor_normal_list_[3 * sensor_idx + 1] = 0.0;
        sensor_normal_list_[3 * sensor_idx + 2] = 1.0;
      }
      else // if(surface_type_ == SurfaceCylinder)
      {
        mjtNum cylinder_angle = dist_y / surface_radius;
        sensor_pos_list_[3 * sensor_idx + 0] = dist_x;
        sensor_pos_list_[3 * sensor_idx + 1] = surface_radius * mju_sin(cylinder_angle);
        sensor_pos_list_[3 * sensor_idx + 2] = surface_radius * (1 - mju_cos(cylinder_angle));
        sensor_normal_list_[3 * sensor_idx + 0] = 0.0;
        sensor_normal_list_[3 * sensor_idx + 1] = -1 * mju_sin(cylinder_angle);
        sensor_normal_list_[3 * sensor_idx + 2] = mju_cos(cylinder_angle);
        mju_normalize3(sensor_normal_list_ + 3 * sensor_idx);
      }
    }
  }
}

TactileSensor::~TactileSensor()
{
  mju_free(sensor_pos_list_);
  mju_free(sensor_normal_list_);
}

void TactileSensor::reset(const mjModel *, // m
                          int // plugin_id
)
{
}

void TactileSensor::compute(const mjModel * m, mjData * d, int // plugin_id
)
{
#if mjVERSION_HEADER >= 300
  mj_markStack(d);
#else
  mjMARKSTACK;
#endif

  mjtNum * sensordata = d->sensordata + m->sensor_adr[sensor_id_];
  mju_zero(sensordata, m->sensor_dim[sensor_id_]);

  int site_bodyid = m->site_bodyid[site_id_];
  int site_body_weldid = m->body_weldid[site_bodyid];
  mjtNum * site_pos = d->site_xpos + 3 * site_id_;
  mjtNum * site_mat = d->site_xmat + 9 * site_id_;

  for(int contact_idx = 0; contact_idx < d->ncon; contact_idx++)
  {
    int contact_bodyid1 = m->geom_bodyid[d->contact[contact_idx].geom1];
    int contact_body_weldid1 = m->body_weldid[contact_bodyid1];
    int contact_bodyid2 = m->geom_bodyid[d->contact[contact_idx].geom2];
    int contact_body_weldid2 = m->body_weldid[contact_bodyid2];

    if(!(contact_body_weldid1 == site_body_weldid || contact_body_weldid2 == site_body_weldid))
    {
      continue;
    }

    mjtNum contact_pos[3]; // Contact position in the site frame
    mju_sub3(contact_pos, d->contact[contact_idx].pos, site_pos);
    mju_rotVecMatT(contact_pos, contact_pos, site_mat);

    int sensor_idx_min = 0;
    mjtNum dist_min = 1e10;
    for(int sensor_idx = 0; sensor_idx < sensor_total_num_; sensor_idx++)
    {
      mjtNum * sensor_pos = sensor_pos_list_ + 3 * sensor_idx;
      mjtNum dist = mju_dist3(contact_pos, sensor_pos);
      if(dist < dist_min)
      {
        sensor_idx_min = sensor_idx;
        dist_min = dist;
      }
    }

    {
      mjtNum force[6]; // Contact force in the site frame (the three elements behind are not processed)
      mj_contactForce(m, d, contact_idx, force);
      mju_rotVecMatT(force, force, d->contact[contact_idx].frame);
      mju_rotVecMatT(force, force, site_mat);

      // Reverse the force direction so that the force is toward the site body
      if(site_body_weldid == contact_bodyid1)
      {
        mju_scl3(force, force, -1);
      }

      mjtNum * sensor_normal = sensor_normal_list_ + 3 * sensor_idx_min;
      sensordata[sensor_idx_min] += mju_dot(sensor_normal, force, 3);
    }
  }

  mju_copy(sensordata + 1 * sensor_total_num_, sensor_pos_list_, 3 * sensor_total_num_);
  mju_copy(sensordata + 4 * sensor_total_num_, sensor_normal_list_, 3 * sensor_total_num_);

#if mjVERSION_HEADER >= 300
  mj_freeStack(d);
#else
  mjFREESTACK;
#endif
}

void TactileSensor::visualize(const mjModel * m,
                              mjData * d,
                              const mjvOption *, // opt
                              mjvScene * scn,
                              int // plugin_id
)
{
#if mjVERSION_HEADER >= 300
  mj_markStack(d);
#else
  mjMARKSTACK;
#endif

  mjtNum * sensordata = d->sensordata + m->sensor_adr[sensor_id_];

  mjtNum * site_pos = d->site_xpos + 3 * site_id_;
  mjtNum * site_mat = d->site_xmat + 9 * site_id_;

  mjtNum force_max = 0.0;
  for(int sensor_idx = 0; sensor_idx < sensor_total_num_; sensor_idx++)
  {
    force_max = mjMAX(force_max, mju_abs(sensordata[sensor_idx]));
  }

  for(int sensor_idx = 0; sensor_idx < sensor_total_num_; sensor_idx++)
  {
    mjtNum sensor_pos_world[3];
    mjtNum * sensor_pos = sensor_pos_list_ + 3 * sensor_idx;
    mju_rotVecMat(sensor_pos_world, sensor_pos, site_mat);
    mju_addTo3(sensor_pos_world, site_pos);

    mjtNum sensor_mat_world[9];
    if(surface_type_ == SurfacePlane)
    {
      mju_copy(sensor_mat_world, site_mat, 9);
    }
    else // if(surface_type_ == SurfaceCylinder)
    {
      mjtNum * sensor_normal = sensor_normal_list_ + 3 * sensor_idx;
      mjtNum cylinder_angle = mju_atan2(-1 * sensor_normal[1], sensor_normal[2]);
      mjtNum cylinder_axis[3] = {1.0, 0.0, 0.0};
      mjtNum cylinder_quat[4], sensor_quat_world[4];
      mju_axisAngle2Quat(cylinder_quat, cylinder_axis, cylinder_angle);
      mju_mat2Quat(sensor_quat_world, site_mat);
      mju_mulQuat(sensor_quat_world, sensor_quat_world, cylinder_quat);
      mju_quat2Mat(sensor_mat_world, sensor_quat_world);
    }

    float rgba[4] = {1, 1, 1, 0.5};
    if(sensordata[sensor_idx] > 0)
    {
      rgba[0] = static_cast<float>(0.7 * (1.0 - sensordata[sensor_idx] / force_max));
    }
    else if(sensordata[sensor_idx] < 0)
    {
      rgba[1] = static_cast<float>(0.7 * (1.0 + sensordata[sensor_idx] / force_max));
    }

    mjvGeom * sensor_geom = scn->geoms + scn->ngeom;
    if(grid_type_ == GridSquare)
    {
      mjtNum size[3];
      size[0] = sensor_interval_ / 2;
      size[1] = sensor_interval_ / 2;
      size[2] = 0.001;
      mjv_initGeom(sensor_geom, mjGEOM_BOX, size, sensor_pos_world, sensor_mat_world, rgba);
    }
    else // if(grid_type_ == GridHex)
    {
      mjtNum size[2];
      size[0] = sensor_interval_ / 2;
      size[1] = 0.001;
      mjv_initGeom(sensor_geom, mjGEOM_CYLINDER, size, sensor_pos_world, sensor_mat_world, rgba);
    }
    sensor_geom->objtype = mjOBJ_UNKNOWN;
    sensor_geom->objid = -1;
    sensor_geom->category = mjCAT_DECOR;
    sensor_geom->segid = scn->ngeom;
    scn->ngeom++;
  }

#if mjVERSION_HEADER >= 300
  mj_freeStack(d);
#else
  mjFREESTACK;
#endif
}

} // namespace mujoco::plugin::sensor
