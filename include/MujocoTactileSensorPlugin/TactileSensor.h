#pragma once

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <algorithm>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace mujoco::plugin::sensor
{

/** \brief Checks that a plugin config attribute exists. */
inline bool checkAttr(const std::string & input)
{
  char * end;
  std::string value = input;
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

/** \brief Converts a string into a numeric vector. */
template<typename T>
inline void readVector(std::vector<T> & output, const std::string & input)
{
  std::stringstream ss(input);
  std::string item;
  char delim = ' ';
  while(getline(ss, item, delim))
  {
    if(!checkAttr(item))
    {
      continue;
    }
    output.push_back(static_cast<T>(strtod(item.c_str(), nullptr)));
  }
}

/** \brief Tactile sensor.

    A tactile sensor is attached to a site and senses contact normal forces between the site's parent body and all other
   bodies.
 */
class TactileSensor
{
public:
  /** \brief Sensor surface type. */
  typedef enum SurfaceType_
  {
    //! Plane surface
    SurfacePlane = 0,

    //! Cylindrical surface
    SurfaceCylinder
  } SurfaceType;

  /** \brief Sensor grid type. */
  typedef enum GridType_
  {
    //! Square grid
    GridSquare = 0,

    //! Hexagonal grid
    GridHex
  } GridType;

public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static TactileSensor * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  TactileSensor(TactileSensor &&) = default;

  /** \brief Destructor. */
  ~TactileSensor();

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
  /** \brief Constructor.
      \param m model
      \param d data
      \param sensor_id sensor ID
      \param sensor_nums number of sensors
      \param sensor_interval sensor interval
      \param surface_radius sensor surface radius (0 for plane)
      \param is_hex_grid whether the sensor grid is hexagonal
   */
  TactileSensor(const mjModel * m,
                mjData * d,
                int sensor_id,
                int sensor_nums[2],
                mjtNum sensor_interval,
                mjtNum surface_radius,
                bool is_hex_grid);

protected:
  //! Sensor ID
  int sensor_id_ = -1;

  //! Site ID
  int site_id_ = -1;

  //! Total number of sensors
  int sensor_total_num_;

  //! Sensor interval
  mjtNum sensor_interval_;

  //! Sensor surface type
  SurfaceType surface_type_;

  //! Sensor grid type
  GridType grid_type_;

  //! List of sensor positions in the site frame
  mjtNum * sensor_pos_list_;

  //! List of sensor normal directions in the site frame
  mjtNum * sensor_normal_list_;
};

} // namespace mujoco::plugin::sensor
