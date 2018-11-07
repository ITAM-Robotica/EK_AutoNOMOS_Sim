cmake_minimum_required(VERSION 2.8.3)
project(autonomos_gazebo_simulation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	geometry_msgs
	cmake_modules REQUIRED
	gazebo_msgs
	)


catkin_package(

	LIBRARIES
	CATKIN_DEPENDS
    	roscpp
    	sensor_msgs
    	std_msgs
	DEPENDS
)

###########
## Build ##
###########

include_directories(include
			${catkin_INCLUDE_DIRS} 
			)

link_directories(
	${Boost_LIBRARY_DIRS}
)

# add_executable(tf2_broadcaster_node src/tf2_broadcaster_node.cpp)
# add_executable(robot_pose_publisher src/robot_pose_publisher.cpp)

# target_link_libraries(tf2_broadcaster_node ${catkin_LIBRARIES})
# target_link_libraries(robot_pose_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS AutoNOMOS_simulation AutoNOMOS_simulation_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Install worlds
# set(AUTONOMOS_DEST_PATH "${CATKIN_GLOBAL_SHARE_DESTINATION}/AutoNOMOS")
message("WORLD PATH: ${CATKIN_GLOBAL_SHARE_DESTINATION}")
install(DIRECTORY
	worlds/
  	DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/worlds
)

## Install models
install(DIRECTORY
	models/
  	DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/models/
)

# install(DIRECTORY
# 	models/straight_road/
#   	DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/models/straight_road
# )

# install(DIRECTORY
# 	models/curved_road/
#   	DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/models/curved_road
# )

# install(DIRECTORY
# 	models/parking_space/
#   	DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/models/parking_space
# )

## Launch files
install(DIRECTORY
	launch/
  	DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/launch
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_AutoNOMOS_simulation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
