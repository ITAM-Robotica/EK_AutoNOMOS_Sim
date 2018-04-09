#ifndef _autonomos_PLUGIN_HH_
#define _autonomos_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class autonomos_plugin : public ModelPlugin
  {
    /// \brief Constructor
    public: 
      autonomos_plugin();

      ~autonomos_plugin();



    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
      void SetPosition(const double &_pos);
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
      void OnRosMsg_steering(const std_msgs::Int16ConstPtr &_msg);

      void OnRosMsg_vel(const std_msgs::Int16ConstPtr &_msg);
      
      void OnUpdate(const common::UpdateInfo & /*_info*/);
    /// \brief ROS helper function that processes messages
    private: 
      void QueueThread();

      void autonomos_connect();

      void autonomos_disconnect();

      float position;

      float vel;

      ros::Publisher pub_;
      PubQueue<geometry_msgs::Pose2D>::Ptr pub_queue_;
      
      PubMultiQueue pmq;
      
      event::ConnectionPtr updateConnection;
    /// \brief Pointer to the model.
      physics::ModelPtr model;

    /// \brief Pointer to the joint.
      physics::JointPtr joint;
      physics::JointPtr joint_left_wheel;
      physics::JointPtr joint_right_wheel;

    /// \brief A PID controller for the joint.
      common::PID pid;
      common::PID pid_vel_left;
      common::PID pid_vel_right;

    /// \brief A node use for ROS transport
      std::unique_ptr<ros::NodeHandle> rosNode;

      common::Time prevUpdateTime;
    /// \brief A ROS subscriber
      ros::Subscriber rosSub;
      ros::Subscriber rosSub_vel;

      ros::NodeHandle _nh;
        
    /// \brief A ROS callbackqueue that helps process messages
      ros::CallbackQueue rosQueue;
        
    /// \brief A thread the keeps running the rosQueue
      std::thread rosQueueThread;

  };
        
}
#endif