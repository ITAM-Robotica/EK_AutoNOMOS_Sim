#!/bin/bash
echo "Begining instalation..."
SIM_PWD=$(pwd)

mkdir -p src/Gazebo_plugin/build
cd src/Gazebo_plugin/build
cmake ..
make 
cd $SIM_PWD 

plugin="/src/Gazebo_plugin/build"
models="/src/autonomos_gazebo_simulation/models"
resource="/src/autonomos_gazebo_simulation/worlds"

#plugin_path=$HOME_PWD$plugin
echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$SIM_PWD$plugin" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$SIM_PWD$models" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$SIM_PWD$resource" >> ~/.bashrc

source ~/.bashrc
#export GAZEBO_PLUGIN_PATH=/home/robotica/EK_AutoNOMOS/src/Gazebo_plugin/build

#export GAZEBO_MODEL_PATH=/home/robotica/EK_AutoNOMOS/src/autonomos_gazebo_simulation/models

#export GAZEBO_RESOURCE_PATH=/home/robotica/EK_AutoNOMOS/src/autonomos_gazebo_simulation/worlds/
