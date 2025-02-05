FROM osrf/ros:humble-desktop-full

# copy install scripts
COPY ./dockerfiles/install /root/install

# copy directory specific to scart
COPY ./ /root/catkin_ws/src/mujoco_tactile_sensor_plugin/

# update and install dependencies
RUN chmod +x "/root/install/i_dep.sh"
RUN /bin/sh -e -c /root/install/i_dep.sh

# update and install mujoco
RUN chmod +x "/root/install/i_mujoco.sh"
RUN /bin/sh -e -c /root/install/i_mujoco.sh

# set environment variable for ros
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/install/setup.bash" >> ~/.bashrc

# install ros dependencies
WORKDIR /root/catkin_ws 

# set default bash shell
SHELL ["/bin/bash", "-c"] 

# Install ros dependencies
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/humble/setup.bash && \
  colcon build 