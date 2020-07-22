cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gazebo_plugin)

add_compile_options(-std=c++11)

# Find Gazebo
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs message_generation)
find_package(gazebo REQUIRED)
find_package(ignition-msgs1 QUIET)
find_package(ignition-msgs2 QUIET)
find_package(ignition-transport4 QUIET)
find_package(ignition-transport5 QUIET)
find_package(ignition-transport6 QUIET)
find_package(ignition-transport7 QUIET)
# find_package(ignition-msgs2 REQUIRED)
# find_package(ignition-transport5 REQUIRED)

pkg_check_modules(OGRE OGRE)
pkg_check_modules(libavcodec libavcodec)
if (NOT libavcodec_FOUND)
    BUILD_WARNING ("libavcodec not found. Audio-video capabilities will be disabled.")
else()
    include_directories(${libavcodec_INCLUDE_DIRS})
    link_directories(${libavcodec_LIBRARY_DIRS})
endif ()

add_message_files(
  FILES
  Line_Segment.msg
)

add_service_files(
  FILES
  trajectory_segment.srv
  obstacles_array.srv
  # Service2.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)


catkin_package(
 	INCLUDE_DIRS include
#  LIBRARIES navigation
	CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
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

if(ignition-transport4_FOUND OR ignition-transport5_FOUND OR ignition-transport6_FOUND OR ignition-transport7_FOUND)
	message("ignition transport FOUND")
else()
	message(FATAL_ERROR "ignition transport NOT FOUND. Version 4 or higher REQUIRED. Install it or set the correct paths")
endif()

#message("ogre_libs:" ${OGRE_INCLUDE_DIRS})
#SET(OGRE_LIBRARY_DIRS "${OGRE_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS}/Paging")
#message("ogre_libs:" ${OGRE_INCLUDE_DIRS})

include_directories(include
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${IGNITION-TRANSPORT_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}/Paging
	)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

# Build the plugins

add_library(${PROJECT_NAME}_utilities SHARED src/utilities/ros_timer.cpp)
target_link_libraries(${PROJECT_NAME}_utilities ${catkin_LIBRARIES})

add_library(autonomos_plugin SHARED src/autonomos_plugin.cc)
target_link_libraries(autonomos_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(autonomos_ideal_plugin SHARED src/autonomos_ideal_plugin.cc)
target_link_libraries(autonomos_ideal_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME}_utilities)
# add_executable(marker src/marker.cc)
# target_link_libraries(marker ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})

add_library(obstacles_plugin SHARED src/obstacles.cc)
target_link_libraries(autonomos_ideal_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${PROJECT_NAME}_utilities)


add_library(plot_tree_plugin SHARED src/plot_tree.cc)
target_link_libraries(plot_tree_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES}) 

add_library(parking_lot SHARED src/parking_lot.cc)
target_link_libraries(parking_lot  ${GAZEBO_LIBRARIES})

add_dependencies(autonomos_plugin  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(autonomos_ideal_plugin  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(plot_tree_plugin  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(parking_lot  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(obstacles_plugin  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

### Changed the above lines from this ones. The change should prevent having to compile two times, with the first time generating an error
### leaving this here as I cannot verify that this change works. The change is the ${${PROJECT_NAME}_EXPORTED_TARGETS}
### part. More details in here: http://wiki.ros.org/catkin/CMakeLists.txt
#add_dependencies(autonomos_plugin ${catkin_EXPORTED_TARGETS})
#add_dependencies(autonomos_ideal_plugin ${catkin_EXPORTED_TARGETS})
#add_dependencies(plot_tree_plugin ${catkin_EXPORTED_TARGETS})
#add_dependencies(parking_lot ${catkin_EXPORTED_TARGETS})
#add_dependencies(obstacles_plugin ${catkin_EXPORTED_TARGETS})


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

