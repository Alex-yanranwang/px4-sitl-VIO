#!/bin/bash
#
# Setup environment to make PX4 visible to Gazebo.
#
# Note, this is not necessary if using a ROS catkin workspace with the px4
# package as the paths are exported.
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

if [ "$#" != 3 ]; then
    echo -e "usage: source setup_gazebo_kino.bash src_dir build_dir root_dir\n"
    return 1
fi

SRC_DIR=$1
BUILD_DIR=$2
ROOT_DIR=$3

# setup Gazebo env and update package path
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${BUILD_DIR}/mavlink_sitl_gazebo:${BUILD_DIR}/gazebo_plugins:${ROOT_DIR}/devel/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SRC_DIR}/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/mavlink_sitl_gazebo

echo -e "GAZEBO_PLUGIN_PATH $GAZEBO_PLUGIN_PATH"
echo -e "GAZEBO_MODEL_PATH $GAZEBO_MODEL_PATH"
echo -e "LD_LIBRARY_PATH $LD_LIBRARY_PATH"
