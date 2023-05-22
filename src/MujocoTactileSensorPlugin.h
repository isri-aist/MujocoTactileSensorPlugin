#pragma once

#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::sensor {

// A touch grid sensor is associated with a site and senses contact forces
// and torques between the site's parent body and all other bodies. The site's
// frame determines the orientation of the sensor with the same convention used
// for cameras and lights: the sensor points in the frame's negative-Z
// direction, so the X and Y axes correspond to horizontal and vertical
// directions respectively.
//
// The output of the sensor is a stack of 6 "touch images" corresponding to
// forces and torques in the frame of the sensor. Forces and torques are in the
// [z, x, y] order, corresponding to the ordering in contact frames:
// [normal, tangent, tangent] and [torsional, rolling, rolling].
//
// The sensor has 6 parameters:
//  1. (int) Number of channels [1-6]. Defaults to 1.
//  2. (int) Horizontal resolution.
//  3. (int) Vertical resolution.
//  4. (float) Horizontal field-of-view (fov_x), in degrees.
//  5. (float) Vertical field-of-view (fov_y), in degrees.
//  6. (float) Foveal deformation. Defaults to 0.
class TouchGrid {
 public:
  static TouchGrid* Create(const mjModel* m, mjData* d, int instance);
  TouchGrid(TouchGrid&&) = default;
  ~TouchGrid() = default;

  void Reset(const mjModel* m, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                 mjvScene* scn, int instance);

  static void RegisterPlugin();

  int nchannel_;           // number of channels (1-6)
  int size_[2];            // horizontal and vertical resolution
  mjtNum fov_[2];          // horizontal and vertical field of view, in degrees
  mjtNum gamma_;           // foveal deformation

 private:
  TouchGrid(const mjModel* m, mjData* d, int instance, int nchannel, int* size,
            mjtNum* fov_x, mjtNum gamma);
  std::vector<mjtNum> distance_;
};

}  // namespace mujoco::plugin::sensor
