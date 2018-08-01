cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_compile_options(-std=c++11)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(include)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(ignition-msgs1 QUIET)
find_package(ignition-msgs2 QUIET)
if(ignition-msgs1_FOUND OR ignition-msgs2_FOUND)
	message("ignition msgs FOUND")
else()
	message(FATAL_ERROR "ignition msgs NOT FOUND. Version 1 or higher REQUIRED. Install it or set the correct paths")
endif()

find_package(ignition-transport4 QUIET)
find_package(ignition-transport5 QUIET)
if(ignition-transport4_FOUND OR ignition-transport5_FOUND)
	message("ignition transport FOUND")
else()
	message(FATAL_ERROR "ignition transport NOT FOUND. Version 4 or higher REQUIRED. Install it or set the correct paths")
endif()


set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})


# Build our plugin
add_library(autonomos_plugin SHARED src/autonomos_plugin.cc)
target_link_libraries(autonomos_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

add_executable(marker src/marker.cc)
target_link_libraries(marker ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})

add_library(parking_lot SHARED src/parking_lot.cc)
target_link_libraries(parking_lot  ${GAZEBO_LIBRARIES})
