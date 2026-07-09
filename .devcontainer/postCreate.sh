#!/bin/bash

conda deactivate
rosdep update
rosdep install --from-paths /home/ws/src --ignore-src -y -r
rm -rf build log install
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install

# Source conda setup to enable conda commands in this shell session
source /opt/conda/etc/profile.d/conda.sh
/opt/conda/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
/opt/conda/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
