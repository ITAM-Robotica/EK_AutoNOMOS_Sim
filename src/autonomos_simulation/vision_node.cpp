#include <ros/ros.h>
#include "std_msgs/String.h"
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Pose.h>
// #include <stdlib.h>
// #include <iostream>
// // #include <Eigen/Dense>
// // #include <Eigen/QR>
// // #include <Eigen/LU>
// #include <math.h>
// #include <stdio.h>
// #include <turtlesim/Pose.h>
// // #include <Eigen/Geometry>
// #include "gazebo_msgs/LinkStates.h"

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
// using namespace Eigen;

//For geometry_msgs::Twist using:
// 		dummy.linear.x
// 		dummy.linear.y
// 		dummy.angular.z
geometry_msgs::Pose robot_pose;
geometry_msgs::Twist target_position;
std::string robot_name;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//rate_hz assignment
double rate_hz = 30;


void gen_point_cloud(const gazebo_msgs::LinkStates& msg){
    sensor_msgs::PointCloud output;

    for (int i = 7; i < len(msg.pose); ++i)
    {
    	output.points[i-7].x = (float32) msg.[i].position.x;
    	output.points[i-7].y = (float32) msg.[i].position.y;
    	output.points[i-7].z = (float32) msg.[i].position.z;
    }

}
int main(int argc, char **argv){
	ros::init(argc,argv,"vision_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("vision_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());
	// robot_name = "autoNOMOS_1";
	//Topic to publish velocity command, queue size equals rate_hz to keep up with the rate at which messages are generated,

    //Publish to the turtle topic "/turtle1/cmd_vel at rate_hz"
	// ros::Publisher pub_cloud_points = nh.advertise<sensor_msgs::PointCloud>("/autonomos_mini/points", rate_hz);

	//Topics to acquire robot and target position (from the vision node) 
	// ros::Subscriber sub_robot_pos = nh.subscribe("/gazebo/model_states", 10, &poseCallback); 
	ros::Subscriber sub_points_pos = nh.subscribe("/gazebo/link_states", 10, &gen_point_cloud); 
	// ros::Subscriber sub_ball_pos = nh.subscribe("/target_position_topic", 1, &getTargetPose);
	ros::spin();
    //Twist variable to publish velocity (trajectories)

 //    }
    return 0;
}
