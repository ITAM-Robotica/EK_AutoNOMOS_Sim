#!/bin/bash
echo "Begining instalation..."
PWD=$(pwd)

plugin="src/Gazebo_plugin/build"
models="src/autonomos_gazebo_simulation/models"
resource="src/autonomos_gazebo_simulation/worlds"

plugin_path=$pwd$plugin
echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PWD$plugin" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$PWD$models" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$PWD$resource" >> ~/.bashrc

#export GAZEBO_PLUGIN_PATH=/home/robotica/EK_AutoNOMOS/src/Gazebo_plugin/build

#export GAZEBO_MODEL_PATH=/home/robotica/EK_AutoNOMOS/src/autonomos_gazebo_simulation/models

#export GAZEBO_RESOURCE_PATH=/home/robotica/EK_AutoNOMOS/src/autonomos_gazebo_simulation/worlds/