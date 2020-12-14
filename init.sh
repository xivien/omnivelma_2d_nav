#!/bin/sh

source ~/$(pwd | awk -F/ '{print FS $4}')/devel/setup.bash
export GAZEBO_MODEL_PATH=$(pwd)/../omnivelma/src:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$(pwd)/../models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$(pwd)/../omnivelma/src:$GAZEBO_PLUGIN_PATH
export GAZEBO_RESOURCE_PATH=$(pwd)/../omnivelma/src:$GAZEBO_RESOURCE_PATH
# export GAZEBO_PLUGIN_PATH=/home/xivien/velma_sim/src/gazebo_animatedbox_tutorial/build:$GAZEBO_PLUGIN_PATH
