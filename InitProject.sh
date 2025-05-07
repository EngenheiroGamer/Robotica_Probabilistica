#!/bin/bash
source /opt/ros/humble/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
colcon build --cmake-args -DBUILD_TESTING=ON

