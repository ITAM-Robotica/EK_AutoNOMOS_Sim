#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
// #include <turtlesim/Spawn.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "perception_node");

  ros::NodeHandle nh;
  // ros::init(argc, argv, "sub_pcl");
  // ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
  ros::spin();
  return 0;
};