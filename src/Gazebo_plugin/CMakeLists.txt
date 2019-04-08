cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gazebo_plugin)

add_compile_options(-std=c++11)

# Find Gazebo
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)
find_package(gazebo REQUIRED)
find_package(ignition-msgs1 QUIET)
find_package(ignition-msgs2 QUIET)
find_package(ignition-transport4 QUIET)
find_package(ignition-transport5 QUIET)
# find_package(ignition-msgs2 REQUIRED)
# find_package(ignition-transport5 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
)

message("Gazebo version:" ${GAZEBO_VERSION})

string(REGEX REPLACE "[.][0-9]*"
       "" GAZEBO_VERSION_MAJOR
       ${GAZEBO_VERSION})

message("Gazebo version major:" ${GAZEBO_VERSION_MAJOR})

add_definitions(-DGAZEBO_VERSION_MAJOR=${GAZEBO_VERSION_MAJOR})

if(ignition-msgs1_FOUND OR ignition-msgs2_FOUND)
	message("ignition msgs FOUND")
else()
	message(FATAL_ERROR "ignition msgs NOT FOUND. Version 1 or higher REQUIRED. Install it or set the correct paths")
endif()

if(ignition-transport4_FOUND OR ignition-transport5_FOUND)
	message("ignition transport FOUND")
else()
	message(FATAL_ERROR "ignition transport NOT FOUND. Version 4 or higher REQUIRED. Install it or set the correct paths")
endif()

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

# Build the plugins
add_library(autonomos_plugin SHARED src/autonomos_plugin.cc)
target_link_libraries(autonomos_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

# add_library(autonomos_ideal_plugin SHARED src/autonomos_ideal_plugin.cc)
# target_link_libraries(autonomos_ideal_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})
# add_executable(marker src/marker.cc)
# target_link_libraries(marker ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})

add_library(parking_lot SHARED src/parking_lot.cc)
target_link_libraries(parking_lot  ${GAZEBO_LIBRARIES})

# Install libraries
# message("Path: ${PATH}")
# message("Path to install: ${CATKIN_PACKAGE_SHARE_DESTINATION}")
if(WIN32)
	message("Really? Are you using windows?! Shame...")

else()
	install(DIRECTORY include/
	  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
	)
	install(TARGETS autonomos_plugin
	  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	  COMPONENT
	)
	install(TARGETS parking_lot
	  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	  COMPONENT
	)
	# install(DIRECTORY launch/
 #  		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 #  	)
endif()  

