#pragma once

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <vector>

namespace mujoco::plugin::sensor
{

/** \brief Tactile sensor.

    A tactile sensor is associated with a site and senses contact normal forces between the site's parent body and all
   other bodies.
 */
class TactileSensor
{
public:
  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID

      Checks that all config attributes are defined and within their allowed bounds.
   */
  static TactileSensor * Create(const mjModel * m, mjData * d, int plugin_id);

  /** \brief Register plugin. */
  static void RegisterPlugin();

public:
  /** \brief Copy constructor. */
  TactileSensor(TactileSensor &&) = default;

  /** \brief Register plugin. */
  ~TactileSensor() = default;

  /** \brief Reset.
      \param m model
      \param plugin_id plugin ID
   */
  void reset(const mjModel * m, int plugin_id);

  /** \brief Compute.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  void compute(const mjModel * m, mjData * d, int plugin_id);

  /** \brief Visualize.
      \param m model
      \param d data
      \param opt visualization option
      \param scn rendering scene
      \param plugin_id plugin ID
   */
  void visualize(const mjModel * m, mjData * d, const mjvOption * opt, mjvScene * scn, int plugin_id);

protected:
  //! Number of channels (1-6)
  int nchannel_;

  //! Horizontal and vertical resolution
  int size_[2];

  //! Horizontal and vertical field of view (in degrees)
  mjtNum fov_[2];

  //! Sensor ID
  int sensor_id_ = -1;

  //! Distances from site to contact points
  std::vector<mjtNum> distance_;

private:
  /** \brief Constructor.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  TactileSensor(const mjModel * m, mjData * d, int plugin_id, int nchannel, int * size, mjtNum * fov_x);
};

} // namespace mujoco::plugin::sensor
