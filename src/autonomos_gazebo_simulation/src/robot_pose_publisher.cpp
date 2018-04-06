#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <Eigen/Geometry>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Pose2D.h>


#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))

//For geometry_msgs::Twist using:
// 		dummy.linear.x
// 		dummy.linear.y
// 		dummy.angular.z
geometry_msgs::Twist robot_position;
geometry_msgs::Twist velocity_msg;

geometry_msgs::PoseStamped pose_stp_w;
geometry_msgs::PoseStamped pose_stp_ack;

std::string car_name = "ackermann";
std::string world = "world";
//rate_hz assignment
double rate_hz = 10;
int des_steering = 0;
int speed = 0;
float curr_steering = 0;
float err = 0;
float sum_err = 0;
float kp = .1;
float ki = .0;

float set_steering_force()
{
	
	err = des_steering - curr_steering;
	sum_err += err;
	float pi = kp * err + ki * sum_err; 
	
	// if( pi > 1)
	// {
	// 	pi = 1;
	// }
	// if (pi < -1)
	// {
	// 	pi = -1;
	// }

	ROS_INFO_STREAM("des: " << des_steering << " curr: " << curr_steering);
	ROS_INFO_STREAM("err: " << err << " sum_err: " << sum_err << " tot: " << pi );

	return  pi;
}


// manual_control/steering  (std_msgs/Int16)
void get_steering(const std_msgs::Int16& msg) {
	 des_steering = msg.data;
	 // if(des_steering > 70)
	 // {
	 // 	ROS_WARN("steering > 70. Using 70." );
	 // 	des_steering = 70;
	 // } else if (des_steering < 20) {
	 // 	ROS_WARN("steering < 20. Using 20.");
	 // 	des_steering = 20;
	 // }
}

void get_speed(const std_msgs::Int16& msg) {
	
	 speed = msg.data;
}


int main(int argc, char **argv){
	ros::init(argc,argv,"robot_velocity_node");
	ros::NodeHandle nh("~");
	ROS_INFO_STREAM("robot_velocity_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());

	// if (nh.getParam("st_kp", kp))
	// {
	// 	ROS_INFO_STREAM("The steering kp values is: " << kp);
	// } else {
	// 	kp = 0.1;
	// 	ROS_INFO_STREAM("No steering kp received, using: kp = " << kp);
	// }

	nh.param<float>("st_kp", kp, 0.1);
	ROS_INFO_STREAM("The steering kp values is: " << kp);

	nh.param<float>("st_ki", ki, 0.0001);
	ROS_INFO_STREAM("The steering ki values is: " << ki);

	nh.param<std::string>("car_name", car_name, "ackermann");
	ROS_INFO_STREAM("Using gazebo model: " << car_name);
	// Suscribe to Gazebo service ApplyJointEffor

	
	ros::Publisher pub_cur = nh.advertise<geometry_msgs::Pose2D> ("/robot/pose", 1);
	ros::Publisher pub_des = nh.advertise<std_msgs::Float64> ("/car_des_steering_angle", 1);
	std_msgs::Float64 msg_fl;

    //define the rate
	ros::Rate rate(rate_hz);
	ros::Time start_time ;
	

	double effort[4];

	tf::TransformListener tfListener;
	tfListener.waitForTransform(world, car_name, ros::Time::now(), ros::Duration(3.0) );

	tf::StampedTransform transform_steering;
	double roll, pitch, yaw;

	float aux_pi;

	geometry_msgs::Pose2D car_pose;

	while (ros::ok())
	{
		try{
			// tfListener.transformPose("ackermann", pose_stp_w, pose_stp_ack);
			tfListener.lookupTransform("/ground_plane", "/steer_link", ros::Time(0), transform_steering);
			// ROS_INFO_STREAM("After transformPose: " << pose_stp_w << pose_stp_ack);
			// ROS_INFO_STREAM("After transformPose: " );

			tf::Quaternion q = transform_steering.getRotation();
			// ROS_INFO_STREAM(msg.pose[steering_link].orientation) ;
			// ROS_INFO_STREAM(q);
			tf::Matrix3x3 m(q);
		
			m.getRPY(roll, pitch, yaw); //USING yaw
			ROS_INFO_STREAM("-------------------------------------------------------");
			ROS_INFO_STREAM("( " << transform_steering.getOrigin().x() << " , " << transform_steering.getOrigin().y() << " , " << transform_steering.getOrigin().z() << " )");
			ROS_INFO_STREAM("( " << roll << " , " << pitch << " , " << yaw << " )");
			ROS_INFO_STREAM("rads: " << -yaw << " deg: " << yaw * 180/ M_PI );
			curr_steering = yaw * 180/ M_PI;
			ROS_INFO_STREAM("-------------------------------------------------------");

			car_pose.x = transform_steering.getOrigin().x();
			car_pose.y = transform_steering.getOrigin().y();
			car_pose.theta = yaw;

			// ROS_INFO_STREAM(transform_steering.getRotation() );
		} catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			// ROS_INFO_STREAM("ZERO Quaternion!!");
		}

		pub_cur.publish(car_pose);
		
  		
		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}
