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
#include "gazebo_msgs/LinkStates.h"

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>


#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_ros/transforms.h>
// #include <sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/transforms.h>
// using namespace Eigen;
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// #include <tf2_sensor_msgs/tf_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include "/opt/ros/indigo/ros_catkin_ws/install_isolated/include/pcl_ros/impl/transforms.hpp"
// #include <pcl_ros/impl/transfoms.hpp>
// #include <pcl/pcl_conversions.h>
// geometry_msgs::Pose robot_pose;
// geometry_msgs::Twist target_position;
// std::string robot_name;

//rate_hz assignment
double rate_hz = 30;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr pc_original (new PointCloud);
PointCloud::Ptr pc_car (new PointCloud);
std::string robot_name;
// sensor_msgs::PointCloud2 pc_original;
// sensor_msgs::PointCloud2 pc_car;
sensor_msgs::PointCloud2 sens_pc_original;
void gen_point_cloud(const gazebo_msgs::LinkStates& gazebo_msg){
    

	pc_original->header.frame_id = "world";
	pc_original->height = 1;
	pc_original->width = gazebo_msg.pose.size();
	
	pcl::PointXYZ points;
    for (int i = 7; i < gazebo_msg.pose.size(); ++i)
    {
    	// printf("gazebo_msg.pose.size: %d: %d\n", gazebo_msg.pose.size(),i);
    	 pc_original->points.push_back (pcl::PointXYZ(
    	 			gazebo_msg.pose[i].position.x, 
    	 			gazebo_msg.pose[i].position.y, 
    	 			gazebo_msg.pose[i].position.z));
    	// msg->points[i-7].x = i;
    	// msg->points[i-7].y = i+1;
    	// msg->points[i-7].z = i+2;
    	// output.points[i-7].x = gazebo_msg.[i].position.x;
    	// output.points[i-7].y = gazebo_msg.[i].position.y;
    	// output.points[i-7].z = gazebo_msg.[i].position.z;
    }
    // pc_original->header.stamp = ros::Time::now().toNSec();
    pc_original -> header.stamp = ros::Time::now()/1000 - 1;
  // pcl::toROSMsg( pc_original, sens_pc_original );
}

void transform_pointCloud(const tf::TransformListener& tfListener){

	// tf2_ros::Buffer tfBuffer;
 //    tf2_ros::TransformListener tfListener(tfBuffer);
	
    // tf::Buffer tfBuffer;
	// tf::TransformListener tfListener;
	// tf2_ros::BufferInterface bfTransform;
// 
	// geometry_msgs::TransformStamped transformStamped;
	// tf::StampedTransform transformStamped;
	robot_name = "autoNOMOS_1";
	try{
	// 	// bool 	transformPointCloud (const std::string &target_frame, const pcl::PointCloud< PointT > 
	// 	// 			&cloud_in, pcl::PointCloud< PointT > &cloud_out, const tf::TransformListener &tf_listener)
	//   // transformStamped = tfBuffer.lookupTransform("turtle2", "autoNOMOS_1",
	//                            // ros::Time(0));

	// 	//(const std::string &target_frame, const sensor_msgs::PointCloud &pcin, sensor_msgs::PointCloud &pcout) const
	// 	// tf::TransformListener::transfromPointCloud(robot_name, )

	// 	// tfListener.lookupTransform(robot_name, "world", ros::Time(0), transformStamped);
		pcl_ros::transformPointCloud(robot_name, *pc_original, *pc_car, tfListener);
	// 	// void 	tf2::doTransform (const sensor_msgs::PointCloud2 &p_in, 
	// 				// sensor_msgs::PointCloud2 &p_out, const geometry_msgs::TransformStamped &t_in)
		
	// 	// tf2::doTransform(pc_original, pc_car, transformStamped);
	}
	catch (tf::TransformException &ex) {
	  ROS_WARN("%s",ex.what());
	//   // ros::Duration(1.0).sleep();
	//   // continue;
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

	ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

	tf::TransformListener tfListener;
	// PointCloud::Ptr msg (new PointCloud);
	// // ros::Subscriber sub_ball_pos = nh.subscribe("/target_position_topic", 1, &getTargetPose);
	// msg->header.frame_id = "some_tf_frame";
	// msg->height = msg->width = 1;
	// msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));


	ros::Rate loop_rate(rate_hz);
	while (nh.ok())
	{
	 // pcl_conversions::toPCL(ros::Time::now(), pc_car->header.stamp);

	  pc_car->header.stamp = pcl_conversions::toPCL(ros::Time::now());//ros::Time::now().toNSec()* 0.0001; // 1000ull);

	  transform_pointCloud(tfListener);
	  // pub.publish (pc_car);
	  pub.publish (pc_car);
	  ros::spinOnce ();
	  loop_rate.sleep ();
	}
    //Twist variable to publish velocity (trajectories)

 //    }
    return 0;
}
