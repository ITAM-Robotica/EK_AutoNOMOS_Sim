#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/LU>
#include <math.h>
#include <stdio.h>
#include <turtlesim/Pose.h>
#include <Eigen/Geometry>
#include "gazebo_msgs/LinkStates.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// #include <LinkStates.h>

using namespace Eigen;

//For geometry_msgs::Twist using:
// 		dummy.linear.x
// 		dummy.linear.y
// 		dummy.angular.z
geometry_msgs::Pose robot_pose;
geometry_msgs::Twist target_position;
std::string robot_name;
//rate_hz assignment
double rate_hz = 30;

//Assign the position of the robot (from other topic) to robot_position.
//Assuming the topic that generate the robot position uses geometry_msgs::Twist
//and the information is in *.linear. This might need to be modifed
void getRobotPose(const gazebo_msgs::LinkStates& msg) {

	// msg.name[1]
	//if there is a problem, check that autonomos_mini is in array position 1
	robot_pose = msg.pose[1];
}

//Assign the position of the target (from other topic) to target_position.
//Assuming the topic that generate the robot position uses geometry_msgs::Twist
//and the information is in *.linear. This might need to be modifed
void getTargetPose(const geometry_msgs::Twist& msg) {
	target_position.linear.x = msg.linear.x;
	target_position.linear.y = msg.linear.y;
    target_position.angular.z = msg.angular.z; 
}

//Function to determine if the goal is still far.
//Variables epsilon_x and epsilon_y used to determined if the goal has been reach
bool isGoalFar(geometry_msgs::Twist p_start, geometry_msgs::Twist p_goal) {
	double epsilon_x, epsilon_y;
	epsilon_x = .2; 
	epsilon_y = .2;
	double distance_x = abs(p_goal.linear.x - p_start.linear.x);
	double distance_y = abs(p_goal.linear.y - p_start.linear.y);
	if (distance_x > epsilon_x || distance_y > epsilon_y)
		return true;
	else
		return false;
}

//Function to generate a Linear Constant Velocity from robot's position to the target's position
geometry_msgs::Twist generateConstantVelocity(double constant_speed, geometry_msgs::Twist p_start, geometry_msgs::Twist p_goal){

    // Compute direction to goal
	Vector3d p_start_vector(p_start.linear.x,p_start.linear.y,p_start.angular.z);
	Vector3d p_goal_vector(p_goal.linear.x, p_goal.linear.y, p_goal.angular.z);
	// Vector3d goal_direction_vector = p_goal_vector-p_start_vector;
	Vector3d goal_direction_vector = p_start_vector - p_goal_vector;

    // Compute speed in the direction to goal
	Vector3d velocity_vector = constant_speed * (goal_direction_vector/ goal_direction_vector.norm());

	geometry_msgs::Twist velocity;

	velocity.linear.x = velocity_vector.x();
	velocity.linear.y = velocity_vector.y();
	velocity.angular.z = velocity_vector.z();

	return velocity;
}

// Function to keep velocity under the allowed robot limits
geometry_msgs::Twist boundVelocity(geometry_msgs::Twist velocity) {

	double max_linear_speed = 30;
	double min_linear_speed = 0;
	double max_angular_speed = M_PI*4;
	double min_angular_speed = M_PI/16;

	if (velocity.linear.x > max_linear_speed)
		velocity.linear.x = max_linear_speed;
	else if (velocity.linear.x < -max_linear_speed)
		velocity.linear.x = -max_linear_speed;
	if (velocity.linear.y > max_linear_speed)
		velocity.linear.y = max_linear_speed;
	else if (velocity.linear.y < -max_linear_speed)
		velocity.linear.y = -max_linear_speed;
	if (velocity.angular.z > max_angular_speed)
		velocity.angular.z = max_angular_speed;
	else if (velocity.angular.z < -max_angular_speed)
		velocity.angular.z = -max_angular_speed;

    // Lower speed bounds
	if (velocity.linear.x > 0 && velocity.linear.x < min_linear_speed)
		velocity.linear.x = min_linear_speed;
	else if (velocity.linear.x < 0 && velocity.linear.x > -min_linear_speed)
		velocity.linear.x = -min_linear_speed;
	if (velocity.linear.y > 0 && velocity.linear.y<min_linear_speed)
		velocity.linear.y = min_linear_speed;
	else if (velocity.linear.y < 0 && velocity.linear.y > -min_linear_speed)
		velocity.linear.y = -min_linear_speed;
	if (velocity.angular.z >0 && velocity.angular.z < min_angular_speed)
		velocity.angular.z = min_angular_speed;
	else if (velocity.angular.z < 0 && velocity.linear.z > - min_angular_speed )
		velocity.angular.z = -min_angular_speed;

}
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
	ros::init(argc,argv,"intersection_localization_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("intersection_localization initialized");
	ROS_INFO_STREAM(ros::this_node::getName());
	
	//Topic to publish velocity command, queue size equals rate_hz to keep up with the rate at which messages are generated,

    //Publish to the turtle topic "/turtle1/cmd_vel at rate_hz"
	ros::Publisher pub_cloud_points = nh.advertise<sensor_msgs::PointCloud>("/autonomos_mini/points", rate_hz);

	//Topics to acquire robot and target position (from the vision node) 
	ros::Subscriber sub_robot_pos = nh.subscribe("/gazebo/model_states", 1, &getRobotPose); 
	// ros::Subscriber sub_ball_pos = nh.subscribe("/target_position_topic", 1, &getTargetPose);

    //Twist variable to publish velocity (trajectories)
	// geometry_msgs::Twist desired_velocity;
	sensor_msgs::PointCloud cloud;
	// double tiempo = 0;

    //define the max speed
	// double cruise_speed = 30;

    //define the rate
	ros::Rate rate(rate_hz);
	tf::Transform transform;
	double b1=0, b2=0;
	while (ros::ok())
	{
        
		transform.setOrigin(robot_pose.position);
		transform.setRotation(robot_pose.orientation);

		b1 = robot_pose.position.y - tan(54 * PI / 180.0) * robot_pose.position.x;
		b2 = robot_pose.position.y - tan(124 * PI / 180.0) * robot_pose.position.x;



		ROS_INFO_STREAM("Point1 Position:"
			<<" X="<<robot_position.linear.x
			<<",Y="<<robot_position.linear.y);
		ROS_INFO_STREAM("Target position:"
			<<" X="<<target_position.linear.x
			<<",Y="<<target_position.linear.y
			<<",W="<<target_position.angular.z);

		
		//publish the new velocity
		pub_cloud_points.publish(desired_velocity);
		
		ros::spinOnce();
		rate.sleep();
        // tiempo+=(1/rate_hz); 
    }
    return 0;
}
