#ifndef _AUTONOMOS_IDEAL_PLUGIN_HH_
#define _AUTONOMOS_IDEAL_PLUGIN_HH_

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
#include "gazebo_plugin/trajectory_segment.h"

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/PubQueue.h>

// ign
#include <ignition/math.hh>

#define WHEEL_PERIMETER M_PI * 0.03 * 2 // PI * DIAM = PI * RADIUS * 2
#define REAR_FRONT_DISTANCE 0.25

using namespace gazebo;
/// \brief A plugin to control a Velodyne sensor.
class autonomos_ideal_plugin : public ModelPlugin
{
  /// \brief Constructor
  public: 
    autonomos_ideal_plugin();

    ~autonomos_ideal_plugin();

    /**
     * @brief Called when the plugin is inserted
     * @details The load function is called by Gazebo when the plugin is inserted into simulation
     * 
     * @param _model A pointer to the model that this plugin is
     * @param _sdf A pointer to the plugin's SDF element
     */
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

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

    ///////////////
    // VARIABLES //
    ///////////////

    // ignition::math::Pose3d new_pose;

    common::Time seg_init_time;
    common::Time seg_fin_time;

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

    physics::JointControllerPtr joint_controller;


    /// \brief A PID controller for the joint.
    common::PID pid_steering;
    common::PID pid_vel_left;
    common::PID pid_vel_right;
    common::PID pid_vel_model;

    /// \brief A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> rosNode;

    common::Time prevUpdateTime;
    common::Time current_time;
    common::Time step_time;
    /// \brief A ROS subscriber
    ros::Subscriber rosSub;
    
    ros::Subscriber rosSub_vel;

    ros::ServiceClient ros_service_client;
    ros::NodeHandle _nh;
        
    /// \brief A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;
        
    /// \brief A thread the keeps running the rosQueue
    std::thread rosQueueThread;

    // thread for moving the robot
    std::thread ideal_robot_move;

    gazebo_plugin::trajectory_segment trj_seg_msg;

    ignition::math::Vector3d linear_vel;
};
    
#endif