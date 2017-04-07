#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <Eigen/Geometry>
#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/LinkStates.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))

//For geometry_msgs::Twist using:
// 		dummy.linear.x
// 		dummy.linear.y
// 		dummy.angular.z
geometry_msgs::Twist robot_position;
geometry_msgs::Twist velocity_msg;
std::string car_name = "ackermann";
//rate_hz assignment
double rate_hz = 1;
int des_steering = 0;
int speed = 0;
float curr_steering = 0;
float err = 0;
float sum_err = 0;
float kp = .5;
float ki = .01;

int set_steering_force()
{
	
	err = curr_steering - des_steering;
	sum_err += err;

	ROS_INFO_STREAM("des: " << des_steering << " curr: " << curr_steering);
	ROS_INFO_STREAM("err: " << err << " sum_err: " << sum_err << " tot: " << kp * err + ki * sum_err );

	return kp * err + ki * sum_err ;
}


// manual_control/steering  (std_msgs/Int16)
void get_steering(const std_msgs::Int16& msg) {
	 des_steering = msg.data;
}

void get_speed(const std_msgs::Int16& msg) {
	
	 speed = msg.data;
}

void get_curr_link_states(const gazebo_msgs::LinkStates& msg)
{
	int speed_left_link = -1;
	int speed_right_link = -1;
	int steering_link = -1;
	double roll, pitch, yaw;

	std::string steering_link_name = car_name + "::steer_link";
	std::string speed_left_link_name =  car_name + "::back_left_wheel_link";
	std::string speed_right_link_name = car_name + "::back_right_wheel_link";
	// ROS_INFO_STREAM("getting link states:" << msg);
	// ROS_INFO_STREAM("size: " << msg.name.size());
	for (int i = 0; i < msg.name.size(); ++i)
	{
		if(!steering_link_name.compare(msg.name[i]) )
		{
			ROS_DEBUG_STREAM("steering link is: " << i);
			steering_link = i;
		}
		if(!speed_left_link_name.compare(msg.name[i]) )
		{
			ROS_DEBUG_STREAM("speed left link is: " << i);
		}
		if(!speed_right_link_name.compare(msg.name[i]) )
		{
			ROS_DEBUG_STREAM("speed right link is: " << i);
		}
	}
	geometry_msgs::Quaternion qm = msg.pose[steering_link].orientation;
	tf::Quaternion q(qm.x, qm.y, qm.z, qm.w);
	// ROS_INFO_STREAM(msg.pose[steering_link].orientation) ;
	// ROS_INFO_STREAM(q);
	tf::Matrix3x3 m(q);
	
	m.getRPY(roll, pitch, yaw); //USING yaw
	ROS_DEBUG_STREAM("( " << roll << " , " << pitch << " , " << yaw << " )");

	curr_steering = yaw;

}
int main(int argc, char **argv){
	ros::init(argc,argv,"robot_velocity_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("robot_velocity_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());

	if(argc > 1)
	{
		car_name = argv[2];
	}
	ROS_INFO_STREAM("Using gazebo model: " << car_name);
	// Suscribe to Gazebo service ApplyJointEffor
	ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	gazebo_msgs::ApplyJointEffort eff_msg[4];
	

	ros::Subscriber sub_steering = nh.subscribe("/manual_control/steering", 1, &get_steering);
	ros::Subscriber sub_speed = nh.subscribe("/manual_control/speed", 1, &get_speed);
	ros::Subscriber sub_curr_link_states = nh.subscribe("/gazebo/link_states", 1, &get_curr_link_states);

    //define the rate
	ros::Rate rate(rate_hz);
	ros::Time start_time ;
	ros::Duration duration ;

	double effort[4];
	
	while (ros::ok())
	{
	// double* velMots = getMotorValue(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.angular.z);
	
	// effort [0] = velMots[0];
	// effort [1] = velMots[1];
	// effort [2] = velMots[2];
	// effort [3] = velMots[3];
  
		if(client.exists()){
			
			start_time.sec = 0;
			start_time.nsec = 0;
			duration.sec = 1/rate_hz;
			duration.nsec = 0;

			// Wheel-Joint 1
			eff_msg[0].request.joint_name = "steer_joint";
			eff_msg[0].request.duration = duration;
			eff_msg[0].request.effort = set_steering_force();
			eff_msg[0].request.start_time = start_time;

			// Wheel-Joint 2
			eff_msg[1].request.joint_name = "back_left_wheel_joint";
			eff_msg[1].request.duration = duration;
			eff_msg[1].request.effort = effort[1];
			eff_msg[1].request.start_time = start_time;

			// Wheel-Joint 3
			eff_msg[2].request.joint_name = "back_right_wheel_joint";
			eff_msg[2].request.duration = duration;
			eff_msg[2].request.effort = effort[2];
			eff_msg[2].request.start_time = start_time;


			client.call(eff_msg[0]);
			client.call(eff_msg[1]);																																																																								
			client.call(eff_msg[2]);
			// client.call(eff_msg[3]);
			ROS_INFO_STREAM("Joints ==> 1: " << ((eff_msg[0].response.success == 1) ? "TRUE" : "FALSE") <<
			" 2: " << ((eff_msg[1].response.success == 1) ? "TRUE" : "FALSE") <<
			" 3: " << ((eff_msg[2].response.success == 1) ? "TRUE" : "FALSE")
			// " 4: " << ((eff_msg[3].response.success == 1) ? "TRUE" : "FALSE")
			);
		}																																																																																																
		
		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}
