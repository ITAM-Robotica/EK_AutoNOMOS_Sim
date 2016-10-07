#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <stdlib.h>
#include <iostream>
// #include <Eigen/Dense>
// #include <Eigen/QR>
// #include <Eigen/LU>
#include <math.h>
#include <stdio.h>
#include <turtlesim/Pose.h>
// #include <Eigen/Geometry>
#include "gazebo_msgs/LinkStates.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// #include <LinkStates.h>

// using namespace Eigen;

//For geometry_msgs::Twist using:
// 		dummy.linear.x
// 		dummy.linear.y
// 		dummy.angular.z
geometry_msgs::Pose robot_pose;
geometry_msgs::Twist target_position;
std::string robot_name;
//rate_hz assignment
double rate_hz = 30;

void poseCallback(const gazebo_msgs::LinkStates& msg){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = robot_name;

	transformStamped.transform.translation.x = msg.pose[1].position.x;
	transformStamped.transform.translation.y = msg.pose[1].position.y;
	transformStamped.transform.translation.z = msg.pose[1].position.z;
	
    transformStamped.transform.rotation = msg.pose[1].orientation;

    br.sendTransform(transformStamped);

}

int main(int argc, char **argv){
	ros::init(argc,argv,"tf2_broadcaster_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("tf2_broadcaster_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());
	robot_name = "autoNOMOS_1";
	//Topic to publish velocity command, queue size equals rate_hz to keep up with the rate at which messages are generated,

    //Publish to the turtle topic "/turtle1/cmd_vel at rate_hz"
	// ros::Publisher pub_cloud_points = nh.advertise<sensor_msgs::PointCloud>("/autonomos_mini/points", rate_hz);

	//Topics to acquire robot and target position (from the vision node) 
	ros::Subscriber sub_robot_pos = nh.subscribe("/gazebo/model_states", 10, &poseCallback); 
	ros::Subscriber sub_points_pos = nh.subscribe("/gazebo/link_states", 10, &poseCallback); 
	// ros::Subscriber sub_ball_pos = nh.subscribe("/target_position_topic", 1, &getTargetPose);
	ros::spin();
    //Twist variable to publish velocity (trajectories)
	// geometry_msgs::Twist desired_velocity;
	// sensor_msgs::PointCloud cloud;
	// // double tiempo = 0;

 //    //define the max speed
	// // double cruise_speed = 30;

 //    //define the rate
	// ros::Rate rate(rate_hz);
	// tf::Transform transform;
	// double b1=0, b2=0;
	// while (ros::ok())
	// {
        
	// 	transform.setOrigin(robot_pose.position);
	// 	transform.setRotation(robot_pose.orientation);

	// 	b1 = robot_pose.position.y - tan(54 * PI / 180.0) * robot_pose.position.x;
	// 	b2 = robot_pose.position.y - tan(124 * PI / 180.0) * robot_pose.position.x;



	// 	ROS_INFO_STREAM("Point1 Position:"
	// 		<<" X="<<robot_position.linear.x
	// 		<<",Y="<<robot_position.linear.y);
	// 	ROS_INFO_STREAM("Target position:"
	// 		<<" X="<<target_position.linear.x
	// 		<<",Y="<<target_position.linear.y
	// 		<<",W="<<target_position.angular.z);

		
	// 	//publish the new velocity
	// 	pub_cloud_points.publish(desired_velocity);
		
	// 	ros::spinOnce();
	// 	rate.sleep();
 //        // tiempo+=(1/rate_hz); 
 //    }
    return 0;
}
