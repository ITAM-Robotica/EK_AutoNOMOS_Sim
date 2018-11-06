cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gazebo_plugin)

add_compile_options(-std=c++11)

# Find Gazebo
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)
find_package(gazebo REQUIRED)
find_package(ignition-msgs2 REQUIRED)
find_package(ignition-transport5 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
)

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
target_link_libraries(autonomos_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

# add_executable(marker src/marker.cc)
# target_link_libraries(marker ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})

add_library(parking_lot SHARED src/parking_lot.cc)
target_link_libraries(parking_lot  ${GAZEBO_LIBRARIES})

# Install libraries
message("Path to install: ${CATKIN_PACKAGE_LIB_DESTINATION}")
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
endif()  

