#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <Eigen/Geometry>
#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/JointRequest.h"
#include "gazebo_msgs/LinkStates.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"

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

	pose_stp_w.header.frame_id = "ackermann";
	pose_stp_w.pose.position = msg.pose[steering_link].position;
	pose_stp_w.pose.orientation = msg.pose[steering_link].orientation;
	
	m.getRPY(roll, pitch, yaw); //USING yaw
	ROS_INFO_STREAM("( " << roll << " , " << pitch << " , " << -yaw << " )");
	ROS_INFO_STREAM("rads: " << -yaw << " deg: " << -yaw * 180/ M_PI);
	curr_steering = -yaw * 180 / M_PI;

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
	// if(argc > 1)
	// {
	// 	car_name = argv[2];
	// }
	nh.param<std::string>("car_name", car_name, "ackermann");
	ROS_INFO_STREAM("Using gazebo model: " << car_name);
	// Suscribe to Gazebo service ApplyJointEffor
	ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	ros::ServiceClient client_clr = nh.serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces");
	gazebo_msgs::ApplyJointEffort eff_msg[4];
	
	gazebo_msgs::JointRequest jr;
	jr.request.joint_name = "steer_joint";

	ros::Subscriber sub_steering = nh.subscribe("/manual_control/steering", 1, &get_steering);
	ros::Subscriber sub_speed = nh.subscribe("/manual_control/speed", 1, &get_speed);
	// ros::Subscriber sub_curr_link_states = nh.subscribe("/gazebo/link_states", 1, &get_curr_link_states);

	ros::Publisher pub_cur = nh.advertise<std_msgs::Float64> ("/car_cur_steering_angle", 1);
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

	while (ros::ok())
	{
	// double* velMots = getMotorValue(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.angular.z);
	
	// effort [0] = velMots[0];
	// effort [1] = velMots[1];
	// effort [2] = velMots[2];
	// effort [3] = velMots[3];
		// ROS_INFO_STREAM("Before transformPose: " << pose_stp_w << pose_stp_ack);
		try{
			// tfListener.transformPose("ackermann", pose_stp_w, pose_stp_ack);
			tfListener.lookupTransform("/ackermann", "/steer_link", ros::Time(0), transform_steering);
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

			// ROS_INFO_STREAM(transform_steering.getRotation() );
		} catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			// ROS_INFO_STREAM("ZERO Quaternion!!");
		}
		
  		
		if(client.exists()){
			
			start_time.sec = 0;
			start_time.nsec = 0;
			// ros::Duration duration (1/rate_hz);
			ros::Duration duration;
			duration.sec = 0;
			duration.nsec = 0010000000;
			// set_steering_force(); 
			// Wheel-Joint 1
			aux_pi = set_steering_force();
			ROS_INFO_STREAM("PI: " << aux_pi << " duration: " << duration);

			eff_msg[0].request.joint_name = "steer_joint";
			eff_msg[0].request.duration = duration;
			eff_msg[0].request.effort = aux_pi;
			eff_msg[0].request.start_time = start_time;

			// Wheel-Joint 2
			eff_msg[1].request.joint_name = "back_left_wheel_joint";
			eff_msg[1].request.duration = duration;
			eff_msg[1].request.effort = 0;
			eff_msg[1].request.start_time = start_time;

			// Wheel-Joint 3
			eff_msg[2].request.joint_name = "back_right_wheel_joint";
			eff_msg[2].request.duration = duration;
			eff_msg[2].request.effort = 0;
			eff_msg[2].request.start_time = start_time;

			// client_clr.call(jr);
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
		msg_fl.data = curr_steering;
		pub_cur.publish(msg_fl);
		msg_fl.data = des_steering;
		pub_des.publish(msg_fl);

		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}
