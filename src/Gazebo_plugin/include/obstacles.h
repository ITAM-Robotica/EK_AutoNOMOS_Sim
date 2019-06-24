#ifndef _OBSTACLES_HH_
#define _OBSTACLES_HH_

// std
#include <thread>

// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

// msgs
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include "gazebo_plugin/obstacles_array.h"

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/PubQueue.h>

// ign
#include <ignition/math.hh>

// own
// #include "utilities/ros_timer.hpp"

#define WHEEL_PERIMETER M_PI * 0.03 * 2 // PI * DIAM = PI * RADIUS * 2
#define REAR_FRONT_DISTANCE 0.25

using namespace gazebo;
/// \brief A plugin to control a Velodyne sensor.
class obstacles_plugin : public WorldPlugin
{
  /// \brief Constructor
  public: 
    obstacles_plugin();

    ~obstacles_plugin();

    /**
     * @brief Called when the plugin is inserted
     * @details The load function is called by Gazebo when the plugin is inserted into simulation
     * 
     * @param _model A pointer to the model that this plugin is
     * @param _sdf A pointer to the plugin's SDF element
     */
    virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    void SetPosition();
    
    void drive();
     
    void OnUpdate(const common::UpdateInfo & /*_info*/);
    /// \brief ROS helper function that processes messages
  private: 
    ///////////////
    // FUNCTIONS //
    ///////////////

    void QueueThread();

    void autonomos_connect();

    void autonomos_disconnect();

    void next_pose();

    void autonomos_ideal_plugin_pose_disconnect();

    void autonomos_ideal_plugin_pose_connect();

    void update_next_state(const geometry_msgs::Pose2DConstPtr &_msg);

    bool get_obstacles(gazebo_plugin::obstacles_array::Request  &req, gazebo_plugin::obstacles_array::Response &res);

    geometry_msgs::Pose pose_ign_to_ros(ignition::math::Pose3d pose);

    geometry_msgs::Polygon get_box_vertices(ignition::math::Box box);

    geometry_msgs::Point box_dimensions(ignition::math::Box box);


    ///////////////
    // VARIABLES //
    ///////////////

    // ignition::math::Pose3d new_pose;

    // sys_ros_timer_t timer;

    double traj_duration;

    int get_obstacles_seq;

    // common::Time seg_init_time;
    // common::Time seg_fin_time;

    ros::Time seg_init_time;
    ros::Time seg_fin_time;

    bool waiting_valid_traj;

    float vel;

    float steering_pos_traget;
    float velocity_target;

    std::string steering_name;
    std::string left_wheel_name;
    std::string right_wheel_name;

    ros::Publisher pub_;
    PubQueue<geometry_msgs::Pose2D>::Ptr pub_queue_;
      
    PubMultiQueue pmq;
      
    event::ConnectionPtr updateConnection;

    /// \brief Pointer to the model.
    physics::ModelPtr model;

    physics::WorldPtr world;

    /// \brief Pointer to the joint.
    physics::JointPtr joint_steering;
    physics::JointPtr joint_left_wheel;
    physics::JointPtr joint_right_wheel;

    common::PID pid_vel_left;
    common::PID pid_vel_right;

    physics::JointControllerPtr joint_controller;

    ros::Publisher pub_pose;

    int autonomos_pose_connection;

    /// \brief A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Time prevUpdateTime;
    ros::Time current_time;
    ros::Duration step_time;
    // double t_step;
    /// \brief A ROS subscriber
    ros::Subscriber rosSub;
    
    ros::Subscriber sub_next_state;

    ros::ServiceServer ros_serv;
    ros::NodeHandle _nh;
        
    /// \brief A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;
        
    /// \brief A thread the keeps running the rosQueue
    std::thread rosQueueThread;

    // thread for moving the robot
    std::thread ideal_robot_move;

    // gazebo_plugin::trajectory_segment trj_seg_msg;

    ignition::math::Vector3d linear_vel;
};
    
#endif