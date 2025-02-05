#!/bin/sh
set -e

apt-get update
apt-get upgrade -y

basic_dep="git \
           curl \
           nano \
           vim \
           python3-pip \
           wget \
           x11-apps"


ros_dep=""

python_dep="setuptools==58.2.0"

apt-get install -y $basic_dep
apt-get update
apt-get upgrade -y

# Install ROS dependencies
DEBIAN_FRONTEND=noninteractive apt-get install -y $ros_dep

## Install python dependencies
pip install $python_dep

cd /root/catkin_ws

# Clean up
apt-get autoremove -y
apt-get clean -y
