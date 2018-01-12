#!/bin/bash

cd ~
git clone https://github.com/ZJUZQ/BOOSTING_lib.git
cd ~/BOOSTING_lib
chmod +x build.sh
./build.sh

cd ~
git clone https://github.com/ZQ2QQ/boost_1_66_0_copy.git
cd ~/boost_1_66_0_copy
chmod +x *.sh
if[ ! -d ~/3rdparty/boost_1_66_0 ]
    mkdir -p ~/3rdparty/boost_1_66_0
fi
./bootstrap.sh --prefix=${HOME}/3rdparty/boost_1_66_0
./b2 install

mkdir -p ~/workspace/ros/catkin/src
cd ~/workspace/ros/catkin
catkin_make --force-cmake
#rosws init ~/workspace/ros/catkin/devel
echo "source ~/workspace/ros/catkin/devel/setup.bash" >> ~/.bashrc

cd ~/workspace/ros/catkin/src
git clone https://github.com/ZJUZQ/ros_visual_localization.git
cd ~/workspace/ros/catkin
catkin_make --force-cmake
