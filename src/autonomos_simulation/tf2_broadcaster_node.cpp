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
#include "tf/tf.h"

std::string robot_name;

//rate_hz assignment
double rate_hz = 5;

void poseCallback(const gazebo_msgs::LinkStates& msg){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	
	double yaw = tf::getYaw(msg.pose[1].orientation);
	yaw = yaw - M_PI/2;
	//aux_tf.transform.rotation = tf::createQuaternionFromRPY(0,0,-M_PI/2);

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = robot_name;

	transformStamped.transform.translation.x = msg.pose[1].position.x;
	transformStamped.transform.translation.y = msg.pose[1].position.y;
	transformStamped.transform.translation.z = msg.pose[1].position.z;
	
	// transformStamped.transform.rotation = tf::createQuaternionFromRPY(0, 0, yaw); //msg.pose[1].orientation;
	tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();
    // transformStamped.transform.rotation = msg.pose[1].orientation;

    br.sendTransform(transformStamped);

}

int main(int argc, char **argv){
	ros::init(argc,argv,"tf2_broadcaster_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("tf2_broadcaster_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());
	robot_name = "autoNOMOS_1";

	ros::Subscriber sub_robot_pos = nh.subscribe("/gazebo/model_states", 1, &poseCallback); 

	ros::Rate loop_rate(rate_hz);
	while(nh.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();

	}
    return 0;
}
