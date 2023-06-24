#pragma once

#include <ros/ros.h>

#include <MujocoTactileSensorPlugin/TactileSensor.h>

namespace mujoco::plugin::sensor
{

/** \brief Tactile sensor with ROS interface.

    A tactile sensor is attached to a site and senses contact normal forces between the site's parent body and all other
   bodies.
 */
class TactileSensorRos : public TactileSensor
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static TactileSensorRos * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  TactileSensorRos(TactileSensorRos &&) = default;

  /** \brief Compute.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  /** \brief Constructor.
      \param m model
      \param d data
      \param sensor_id sensor ID
      \param sensor_nums number of sensors
      \param sensor_interval sensor interval
      \param surface_radius sensor surface radius (0 for plane)
      \param is_hex_grid whether the sensor grid is hexagonal
      \param frame_id frame ID of topic
      \param topic_name topic name
      \param publish_rate publish rate
   */
  TactileSensorRos(const mjModel * m,
                   mjData * d,
                   int sensor_id,
                   int sensor_nums[2],
                   mjtNum sensor_interval,
                   mjtNum surface_radius,
                   bool is_hex_grid,
                   const std::string & frame_id,
                   const std::string & topic_name,
                   mjtNum publish_rate);

protected:
  //! Frame ID of topic
  std::string frame_id_ = "";

  //! ROS node handle
  std::shared_ptr<ros::NodeHandle> nh_;

  //! ROS publisher
  ros::Publisher pub_;

  //! Iteration interval to skip ROS publish
  int publish_skip_ = 0;

  //! Iteration count of simulation
  int sim_cnt_ = 0;
};

} // namespace mujoco::plugin::sensor
