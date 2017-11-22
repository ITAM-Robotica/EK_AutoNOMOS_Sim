#ifndef _VELOCITY_PLUGIN_HH_
#define _VELOCITY_PLUGIN_HH_

#include <thread> //Missing from tutorial!
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
// #include <gazebo/IOBPlugin.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"

#include <geometry_msgs/Twist.h>

// #include <ackermann_msgs/AckermannDrive.h>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelocityPlugin : public ModelPlugin
  {

    public: VelocityPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }
      std::cerr << "\nThis plugin is attach to model[" <<
        _model->GetName() << "]\n";
      
      // Store the model pointer for convenience.
      this->model = _model;

       // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint = _model->GetJoints()[1];
      std::cerr << "\nJoint1:" << _model->GetJoints()[1]->GetName();
      // Setup a P-controller, with a gain of 0.1.
      // this->pid = common::PID(0.1, 0, 0);

      // // Apply the P-controller to the joint.
      // this->model->GetJointController()->SetVelocityPID(
      //     this->joint->GetScopedName(), this->pid);

      // Default to zero velocity
      geometry_msgs::Twist velocity;
      velocity.linear.x = 1;
      velocity.angular.z = 0;

      // // Check that the velocity element exists, then read the value
      // if (_sdf->HasElement("velocity"))
      //   velocity = _sdf->Get<double>("velocity");

      // this->SetVelocity(velocity);
      this->SetVelocity(0);



      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      // this->node->Init(this->model->GetWorld()->GetName());
      this->node->Init(this->model->GetName());
      
      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
      
      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &VelocityPlugin::OnMsg, this);

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }
      
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      
      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        // ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/autonomos_ctrl/speed",
            1,
            boost::bind(&VelocityPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), 
            &this->rosQueue);

      this->rosSub = this->rosNode->subscribe(so);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&VelocityPlugin::QueueThread, this));
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    // public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    public: void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg)
    {
      // this -> SetVelocity(_msg -> data);
      // this->SetVelocity(_msg.linear.x);
      this->SetVelocity(_msg);
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // public: void SetVelocity(const double &_vel)
    public: void SetVelocity(const geometry_msgs::TwistConstPtr &_vel)
    {
      // Set the joint's target velocity.
      // this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 0);
      this->model->GetJoints()[0]->SetVelocity(0, _vel->linear.x);
      this->model->GetJoints()[1]->SetVelocity(0, _vel->linear.x);
      this->model->GetJoints()[6]->SetVelocity(0, _vel->angular.z);
      // std::cerr << "\n@SetVelocity" << this->joint->GetScopedName();
      
    }

    // public: void Set_Steering_Velocity(const double &_vel)
    // {
    //   // Set the joint's target velocity.
    //   // this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 0);
    //   this->model->GetJoints()[7]->SetVelocity(0, 0);
    //   // this->model->GetJoints()[1]->SetVelocity(0,10);
    //   // std::cerr << "\n@SetVelocity" << this->joint->GetScopedName();
      
    // }
    // private: void OnMsg(const geometry_msgs::TwistConstPtr &t_msg)
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      geometry_msgs::TwistConstPtr t_msg;
      // t_msg.linear.x = _msg->x();
      // t_msg.angular.z = _msg->z();
      this->SetVelocity(t_msg);
      // this->SetVelocity(_msg->x());
    }

        /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  //   public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  //   {
  // // Safety check
  //     if (_model->GetJointCount() == 0)
  //     {
  //       std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
  //       return;
  //     }

  //     // Default to zero velocity
  //     double velocity = 0;
      
  //     // Check that the velocity element exists, then read the value
  //     if (_sdf->HasElement("velocity"))
  //       velocity = _sdf->Get<double>("velocity");
      
  //     // Set the joint's target velocity. This target velocity is just
  //     // for demonstration purposes.
  //     this->model->GetJointController()->SetVelocityTarget(
  //     this->joint->GetScopedName(), velocity);
  //   }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelocityPlugin)
}
#endif
