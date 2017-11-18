#!/bin/bash

git clone https://github.com/ZJUZQ/BOOSTING_lib.git
cd BOOSTING_lib
chmod +x build.sh
./build.sh

mkdir -p ~/workspace/ros/catkin/src
cd ~/workspace/ros/catkin
catkin_make --force-cmake
rosws init ~/workspace/ros/catkin/devel
echo "source ~/workspace/ros/catkin/devel/setup.bash" >> ~/.bashrc

cd ~/workspace/ros/catkin/src
git clone https://github.com/ZJUZQ/ros_visual_localization.git
cd ~/workspace/ros/catkin
catkin_make --force-cmake