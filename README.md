**This is the branch for ROS2; use [the ros1 branch](https://github.com/isri-aist/MujocoTactileSensorPlugin/tree/ros1) for ROS1.**

# [MujocoTactileSensorPlugin](https://github.com/isri-aist/MujocoTactileSensorPlugin)
Plugin to simulate tactile sensors in MuJoCo

[![CI](https://github.com/isri-aist/MujocoTactileSensorPlugin/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/MujocoTactileSensorPlugin/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/MujocoTactileSensorPlugin/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/MujocoTactileSensorPlugin)](https://github.com/isri-aist/MujocoTactileSensorPlugin/blob/master/LICENSE)

https://github.com/isri-aist/MujocoTactileSensorPlugin/assets/6636600/840652c9-fd7f-472b-9402-442b5498d862

## Features
- Since it is in plugin style, you can use it without rebuilding MuJoCo from the source.
- Plane and cylinder surfaces are supported as sensor mounting surfaces.
- Square and hexagonal grids are supported as sensor arrays.
- It is built as a ROS 2 package and provides a MuJoCo plugin.
- Sensor information and visualization markers are published as ROS topics.

## Install

### Requirements
- Compiler supporting C++17
- Tested on `Ubuntu 22.04 / ROS Humble`

### Dependencies
- [MuJoCo](https://github.com/deepmind/mujoco) (>= 2.3.5)

### Installation procedure
```bash
# Setup catkin workspace.
$ mkdir -p ${HOME}/ros/ws_mujoco/src
$ cd ${HOME}/ros/ws_mujoco
$ wstool init src
$ wstool set -t src isri-aist/MujocoTactileSensorPlugin git@github.com:isri-aist/MujocoTactileSensorPlugin.git --git -y
$ wstool update -t src
# Install dependent packages.
$ source /opt/ros/${ROS_DISTRO}/setup.bash
$ rosdep install -y -r --from-paths src --ignore-src
# Build a package.
$ colcon build --packages-select mujoco_tactile_sensor_plugin --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=<absolute path to MuJoCo>
```
Add `source ${HOME}/ros/ws_mujoco/install/setup.bash` to `${HOME}/.bashrc`.

## Examples
### Example
Assume that MuJoCo is installed in `${HOME}/.mujoco/mujoco-2.3.5` from release, and the path to the catkin workspace is `${HOME}/ros/ws_mujoco`.
```bash
# Terminal 1
$ cd ${HOME}/.mujoco/mujoco-2.3.5/bin
$ ./simulate `ros2 pkg prefix mujoco_tactile_sensor_plugin`/share/xml/sample_tactile_sensor_ros.xml
# Terminal 2
$ ros2 launch mujoco_tactile_sensor_plugin display.launch.py
```

## Plugins
### MujocoTactileSensorPlugin
This is a ROS-independent plugin to simulate tactile sensors.

The following attributes are required.
- `sensor_nums`: Number of sensors in each of the X and Y directions in a 2D array
- `sensor_interval`: Interval between adjacent sensors [m]
- `surface_radius`: Radius of the sensor mounting surface (zero for plane, positive for cylinder)
- `is_hex_grid`: Whether the sensor grid is square or hexagonal (true for hexagonal, false for square)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoTactileSensorPlugin"/>
</extension>
<sensor>
  <plugin name="tactile_sensor" plugin="MujocoTactileSensorPlugin" objtype="site" objname="[site name]">
    <config key="sensor_nums" value="20 20"/>
    <config key="sensor_interval" value="0.02"/>
    <config key="surface_radius" value="0.5"/>
    <config key="is_hex_grid" value="true"/>
  </plugin>
</sensor>
```

### MujocoTactileSensorRosPlugin
This is a plugin with ROS interface to simulate tactile sensors.

In addition to the attributes of `MujocoTactileSensorPlugin`, the following attributes are required.
- `frame_id`: Frame ID of ROS topic (Site name is used if omitted)
- `topic_name`: ROS topic name of sensor data
- `publish_rate`: Period to publish the topic of sensor data [Hz]

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoTactileSensorRosPlugin"/>
</extension>
<sensor>
  <plugin name="tactile_sensor" plugin="MujocoTactileSensorRosPlugin" objtype="site" objname="[site name]">
    <config key="sensor_nums" value="20 20"/>
    <config key="sensor_interval" value="0.02"/>
    <config key="surface_radius" value="0.5"/>
    <config key="is_hex_grid" value="true"/>
    <config key="topic_name" value="/mujoco/tactile_sensor"/>
    <config key="publish_rate" value="10"/>
  </plugin>
</sensor>
```
