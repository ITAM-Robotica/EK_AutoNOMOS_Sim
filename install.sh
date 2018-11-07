#!/bin/bash
echo "Begining instalation..."
SIM_PWD=$(pwd)

catkin_make

plugin="/devel/lib"
models="/src/autonomos_gazebo_simulation/models"
resource="/src/autonomos_gazebo_simulation/worlds"

echo $plugin

if [[ $GAZEBO_PLUGIN_PATH = *"$SIM_PWD$plugin"* ]]; then
	echo "$SIM_PWD$plugin already in GAZEBO_PLUGIN_PATH"
else
	# no match ==> add the variable to the bashrc file
	echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$SIM_PWD$plugin" >> ~/.bashrc
	echo "GAZEBO_PLUGIN_PATH set"
fi

if [[ $GAZEBO_MODEL_PATH = *"$SIM_PWD$models"* ]]; then
	echo "$SIM_PWD$models already in GAZEBO_MODEL_PATH"
else
	# no match ==> add the variable to the bashrc file
	echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$SIM_PWD$models" >> ~/.bashrc
	echo "GAZEBO_MODEL_PATH set"
fi

if [[ $GAZEBO_RESOURCE_PATH = *"$SIM_PWD$resource"* ]]; then
	echo "$SIM_PWD$resource already in GAZEBO_RESOURCE_PATH"
else
	# no match ==> add the variable to the bashrc file
	echo "export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$SIM_PWD$resource" >> ~/.bashrc
	echo "GAZEBO_RESOURCE_PATH set"
fi

# echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$SIM_PWD$plugin" >> ~/.bashrc
# echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$SIM_PWD$models" >> ~/.bashrc
# echo "export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$SIM_PWD$resource" >> ~/.bashrc

source ~/.bashrc