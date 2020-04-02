#!/usr/bin/env bash

cd ../../..
catkin_make
source devel/setup.bash
cd src/podcar/models/plugins
cmake . ; make
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH`pwd`
cd ..
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH`pwd`
source /usr/share/gazebo/setup.sh
