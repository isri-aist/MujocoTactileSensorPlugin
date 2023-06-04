# [MujocoTactileSensorPlugin](https://github.com/isri-aist/MujocoTactileSensorPlugin)
Plugin to simulate tactile sensors in MuJoCo

[![CI](https://github.com/isri-aist/MujocoTactileSensorPlugin/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/MujocoTactileSensorPlugin/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/MujocoTactileSensorPlugin/)

https://github.com/isri-aist/MujocoTactileSensorPlugin/assets/6636600/73f3c182-3861-4ff6-a9f9-6fcbde263ca5

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
$ catkin build mujoco_tactile_sensor_plugin -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=<absolute path to libtorch>
# Add the following line to `${HOME}/.bashrc`
#   source ${HOME}/ros/ws_mujoco/devel/setup.bash
```

## Examples
### Example as a standalone project
Assume that MuJoCo is installed in `${HOME}/.mujoco/mujoco-2.3.5` from release, and `MuJocoTactileSensorPlugin` is cloned under `${HOME}/src/`.
```bash
cp ${HOME}/src/MujocoTactileSensorPlugin/build/src/libTactileSensorPlugin.so ${HOME}/.mujoco/mujoco-2.3.5/bin/mujoco_plugin
cd ${HOME}/.mujoco/mujoco-2.3.5/bin
./simulate ${HOME}/src/MujocoTactileSensorPlugin/xml/sample_tactile_sensor.xml
```

### Example as a ROS package
Assume that MuJoCo is installed in `${HOME}/.mujoco/mujoco-2.3.5` from release, and the path to the catkin workspace is `${HOME}/ros/ws_mujoco`.
```bash
# Terminal 1
cp ${HOME}/ros/ws_mujoco/devel/lib/libTactileSensorPlugin.so ${HOME}/.mujoco/mujoco-2.3.5/bin/mujoco_plugin
cd ${HOME}/.mujoco/mujoco-2.3.5/bin
./simulate `rospack find mujoco_tactile_sensor_plugin`/xml/sample_tactile_sensor_ros.xml
# Terminal 2
roslaunch mujoco_tactile_sensor_plugin display.launch
```
