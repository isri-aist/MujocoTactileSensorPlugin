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
- It supports both building with cmake as a standalone project and building with catkin as a ROS package.
- If built as a ROS package, sensor information and visualization markers are published as ROS topics.

## Install

### Requirements
- Compiler supporting C++17
- Tested on `Ubuntu 20.04 / ROS Noetic`

### Dependencies
- [MuJoCo](https://github.com/deepmind/mujoco) (>= 2.3.5)

### Installation procedure as a standalone project
```bash
$ mkdir ${HOME}/src && cd ${HOME}/src
$ git clone git@github.com:isri-aist/MujocoTactileSensorPlugin.git --recursive
$ cd MujocoTactileSensorPlugin
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=<absolute path to MuJoCo>
$ make
$ make install
```
`<absolute path to MuJoCo>` is the path to the root directory of MuJoCo.
For example, `${HOME}/.mujoco/mujoco-2.3.5` if you installed MuJoCo from release, or `${HOME}/src/mujoco` if you installed it from source.

### Installation procedure as a ROS package
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
$ catkin init
$ catkin config --extend /opt/ros/${ROS_DISTRO}
$ catkin build mujoco_tactile_sensor_plugin -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=<absolute path to MuJoCo>
```
Add `source ${HOME}/ros/ws_mujoco/devel/setup.bash` to `${HOME}/.bashrc`.

## Examples
### Example as a standalone project
Assume that MuJoCo is installed in `${HOME}/.mujoco/mujoco-2.3.5` from release, and `MuJocoTactileSensorPlugin` is cloned under `${HOME}/src/`.
```bash
$ cp ${HOME}/src/MujocoTactileSensorPlugin/build/src/libTactileSensorPlugin.so ${HOME}/.mujoco/mujoco-2.3.5/bin/mujoco_plugin
$ cd ${HOME}/.mujoco/mujoco-2.3.5/bin
$ ./simulate ${HOME}/src/MujocoTactileSensorPlugin/xml/sample_tactile_sensor.xml
```

### Example as a ROS package
Assume that MuJoCo is installed in `${HOME}/.mujoco/mujoco-2.3.5` from release, and the path to the catkin workspace is `${HOME}/ros/ws_mujoco`.
```bash
# Terminal 1
$ cp ${HOME}/ros/ws_mujoco/devel/lib/libTactileSensorPlugin.so ${HOME}/.mujoco/mujoco-2.3.5/bin/mujoco_plugin
$ cd ${HOME}/.mujoco/mujoco-2.3.5/bin
$ ./simulate `rospack find mujoco_tactile_sensor_plugin`/xml/sample_tactile_sensor_ros.xml
# Terminal 2
$ roslaunch mujoco_tactile_sensor_plugin display.launch
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
This is only available when built as a ROS package.

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
