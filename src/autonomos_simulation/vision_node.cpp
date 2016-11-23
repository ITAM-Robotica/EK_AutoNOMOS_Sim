#include <ros/ros.h>
#include "std_msgs/String.h"
#include "gazebo_msgs/LinkStates.h"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>

#define PI 3.14159265

//rate_hz assignment
double rate_hz = 5;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr pc_original (new PointCloud); 	//pointcloud for the points in world coordinates
PointCloud::Ptr pc_car (new PointCloud);		//pointcloud for the points in car coordinates
PointCloud::Ptr pc_seen (new PointCloud);		//pointcloud for the points seen in car coordinates
std::string robot_name;
std::string world;

// calculate the slope of the lines limiting the vision of the camera
const double m1 = tan ( 54 * PI / 180.0 );
const double m2 = tan (124 * PI / 180.0 );

int pc_original_full = 0;

// Function to fill a PointCloud (pc_original) with the points detected from gazebo
void gen_point_cloud(const gazebo_msgs::LinkStates& gazebo_msg){
	// because points are static and wont change in the entire simulation, gathering the points and filling 
	// the pointcloud is only made once at the beginning of the simulation
	if(pc_original_full == 0)
	{
		// Assuming the name of each link is e.g.: 'AutoNOMOS_mini_intersection::L12_p50', get the last point (of the last line) and extract the total nomber of lines and points
		int str_size = gazebo_msg.name.size() - 1;
		int lines = 0;
		int points = 0;
		int i = 30;
		std::string str = gazebo_msg.name[str_size];

		if (str[29] == 'L'){
			while( str[i] != '_')
			{
				lines = lines * 10 + str[i] - '0' ;
				i++;
			}
			i++; i++;
			while(str[i] != '\0' )
			{
				points = points * 10 + str[i] - '0';
				i++;
			}
		}
		// Initialize the pointcloud 
		pc_original -> header.frame_id = "world";
		pc_original -> height = lines; 
		pc_original -> width = points;
		pc_original -> resize(lines*points); 
		
		// the pointcloud is represented as an array but used as a matrix according to height and width values, 
		// line and point vars are used to track the position of each point as if pc_original was a matrix.
		int line = 0;
		int point = 0;

		 int first_point = 11; //FIRST POINT IF USING ONLY THE AUTONOMOS-MINI
	//	int first_point = 7; //FIRST POINT IF USING ONLY THE TURTLEBOT


	    for (int i = first_point; i < gazebo_msg.pose.size(); ++i)
	    {
	       	pc_original -> points[line * points + point].x = gazebo_msg.pose[i].position.x;
	    	pc_original -> points[line * points + point].y = gazebo_msg.pose[i].position.y;
	    	pc_original -> points[line * points + point].z = gazebo_msg.pose[i].position.z;
	
	    	line++;
	    	if (line % lines == 0)
	    	{
	    		point++;
	    		line = 0;
	    	}
	    }
	    //now pc_original is full and the points in it shouldnt change
    	pc_original_full = 1;
    	// define the time of pointcloud
    	pc_original -> header.stamp = floor(ros::Time::now().toNSec()/1000 );
	} else {
		// update the time in pc_original
    	pc_original -> header.stamp = floor(ros::Time::now().toNSec()/1000 );
    }

}

// function to transform the points in pc_original to the robot coordinates and limit the points seen according to camera specifications
void transform_pointCloud(const tf::TransformListener& tfListener){

	// name of frame to look for
	//robot_name = "autoNOMOS_1";

	// define a NaN point 
	const float bad_val = std::numeric_limits<float>::quiet_NaN();
		pcl::PointXYZ bad_point;
		bad_point.x = bad_val;
		bad_point.y = bad_val;
		bad_point.z = bad_val;

	try{

		// initialize pc_car
		pc_car -> height = pc_original -> height;
		pc_car -> width = pc_original -> width;
		pc_car -> header.frame_id = robot_name;
		
		// use frame called robot_name to transform points in pc_original into pc_car using tfListener
		pcl_ros::transformPointCloud(robot_name, ros::Time(0),  *pc_original, world, *pc_car, tfListener);

		int i = 0;

		//initialize pc_seen
		pc_seen -> height = pc_original -> height;
		pc_seen -> width = pc_original -> width;
		pc_seen -> resize(pc_seen -> height * pc_seen -> width);

		// limit the points in pc_car according to what a camera can see
		BOOST_FOREACH (const pcl::PointXYZ& pt, pc_car -> points){
			//      points above 		points not too far 		   points to the right of the left limit   points to the left of the right limit 
			if(		0.2225 < pt.y 	&& 			pt.y < 1 		&& 			pt.y/m2 <= pt.x 			&& 		pt.x <= pt.y/m1					) 
			{
				pc_seen -> points[i] = pt;
			} else {

				pc_seen -> points[i] = bad_point;
			}
			i++;
		}

	}
	catch (tf::TransformException &ex)
	{
	  ROS_WARN("%s",ex.what());
	}
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"vision_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("vision_node initialized");																																							
	ROS_INFO_STREAM(ros::this_node::getName());

	ros::Subscriber sub_points_pos = nh.subscribe("/gazebo/link_states", 1, &gen_point_cloud); 
	ros::Publisher pub = nh.advertise<PointCloud> ("pointCloud_vision", 1);
	
	world = "world";
	robot_name = "autoNOMOS_1";

	tf::TransformListener tfListener;
	tfListener.waitForTransform(world, robot_name, ros::Time::now(), ros::Duration(3.0) );
	ros::Rate loop_rate(rate_hz);
	while (nh.ok())
	{
		//update cloudpoints time
		pc_car->header.stamp  = floor(ros::Time::now().toNSec()/1000 );
		pc_seen->header.stamp = floor(ros::Time::now().toNSec()/1000 );

		// call function to transform and limit point: pc_original into pc_seen
		transform_pointCloud(tfListener);
		
		// publish the transformed and limited points
		pub.publish (pc_seen);
		ros::spinOnce();
		loop_rate.sleep ();
	}

    return 0;
}
