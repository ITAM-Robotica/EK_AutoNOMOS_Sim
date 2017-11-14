#include <ros/ros.h>
#include "std_msgs/String.h"
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "gazebo_msgs/LinkStates.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "tf/tf.h"

std::string robot_name;

//rate_hz assignment
double rate_hz = 5;
std::string world;
gazebo_msgs::LinkStates links;
gazebo_msgs::LinkStates models;
ros::Time links_time ;

// static tf2_ros::TransformBroadcaster br;
void set_leaf(std::string parent_name, std::string child_name, geometry_msgs::Pose link_pose, geometry_msgs::Twist link_twist)
{
	geometry_msgs::TransformStamped transformStamped;
	std::string parent, child, grandchildren;


	// ROS_INFO_STREAM("Parent: " << parent_name << " , child: " << child_name);

	// Recursive method to create tf tree:
	// Base case: there is no child (child is empty).
	if (!child_name.empty())
	{
		std::size_t char_pos = child_name.find("::"); 							// find the position of the first '::' (multiple chained links  ==> multiple '::')
		parent = "world";// parent_name;													// the parent is alway parent_name
		if(char_pos ==  std::string::npos)										// check if '::' wasnt found (position is npos)
		{
			child = child_name;													// this is the last child, asign it without checking for grandchildren
			grandchildren = "";													// there are no more grand children
		} else {
			child = child_name.substr(0, char_pos);								// if '::' was found, split the child from the grandchildren
			grandchildren = child_name.substr(char_pos+2, std::string::npos);	// get the grandchildren
		}

		if(parent_name.empty())													// Only in the first iteration the parent name can be empty
		{
			set_leaf(child, grandchildren, link_pose, link_twist);				// Recursion
		} else {
			static tf2_ros::TransformBroadcaster br;							// class to broadcast TF

			transformStamped.header.stamp = links_time;							// assign the time of the pose
			transformStamped.header.frame_id = parent;							// assign the parent
			transformStamped.child_frame_id = child;							// assign the child

			transformStamped.transform.translation.x = link_pose.position.x;	// assign the position x
			transformStamped.transform.translation.y = link_pose.position.y;	// assign the position y
			transformStamped.transform.translation.z = link_pose.position.z;	// assign the position z

			transformStamped.transform.rotation.x = link_pose.orientation.x;	// assign the quaternion x
			transformStamped.transform.rotation.y = link_pose.orientation.y;	// assign the quaternion y
			transformStamped.transform.rotation.z = link_pose.orientation.z;	// assign the quaternion z
			transformStamped.transform.rotation.w = link_pose.orientation.w;	// assign the quaternion w
	
			br.sendTransform(transformStamped);									// broadcast the TF

			set_leaf(child, grandchildren, link_pose, link_twist);				// call for the next recursion
		}

	}

}


void poseCallback(const gazebo_msgs::LinkStates& msg){

	models = msg;

}

void set_links(const gazebo_msgs::LinkStates& msg)
{
	links = msg;
	links_time = ros::Time::now();
	// ROS_INFO_STREAM("Links: " << msg);
}

void stablish_tf()
{
	// ROS_INFO_STREAM("CREATING TREE:");
	for (int i = 0; i < links.name.size(); ++i)
	{
		set_leaf(world, links.name[i], links.pose[i], links.twist[i]);
	}
}

int main(int argc, char **argv){
	ros::init(argc,argv,"tf2_broadcaster_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("tf2_broadcaster_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());

	ROS_ASSERT_MSG(argc > 1, "A name for the robot is needed (as argument)...");
	robot_name = argv[1];
	world = argc>2 ? argv[2] : "world";
	
	ros::Subscriber sub_robot_pos = nh.subscribe("/gazebo/model_states", 1, &poseCallback); 
	ros::Subscriber sub_link_states = nh.subscribe("/gazebo/link_states", 1, &set_links); 

	ROS_INFO_STREAM("World frame name is \"" << world << "\". Robot frame name is \"" << robot_name << "\".");
	ROS_INFO_STREAM("Ready...");


	ros::Rate loop_rate(rate_hz);
	while(nh.ok())
	{


		stablish_tf();
		ros::spinOnce();
		loop_rate.sleep();

	}
    return 0;
}
