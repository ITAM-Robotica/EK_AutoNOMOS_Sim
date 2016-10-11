#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
// #include <turtlesim/Spawn.h>

double rate_hz = 5;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  int line  = 0;
  int point = 0;
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  {
    printf ("Line: %d\tPoint: %d",line, point);
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    point++;
    if(point % msg -> width == 0)
    {
      line++;
      point = 0;
    }

  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "perception_node");

  ros::NodeHandle nh;
  // ros::init(argc, argv, "sub_pcl");
  // ros::NodeHandle nh;

  ros::Rate loop_rate(rate_hz);

  ros::Subscriber sub = nh.subscribe<PointCloud>("pointCloud_vision", 1, callback);

  while(nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};