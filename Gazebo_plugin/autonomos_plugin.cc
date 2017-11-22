#ifndef _autonomos_PLUGIN_HH_
#define _autonomos_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class autonomos_plugin : public ModelPlugin
  {
    /// \brief Constructor
    public: autonomos_plugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
          std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
          return;
        }
      
        // Store the model pointer for convenience.
        this->model = _model;
      
        const std::string val = "Printing from plugin...";
        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        this->joint = _model->GetJoint("steer_joint");

        std::cout << val << std::endl;
        
        // for (auto & element : _model->GetJoints() ) {
        //   std::cout << element->GetName() << std::endl;
        // }

        std::cout << this->joint->GetName() << std::endl;
        

        // Setup a P-controller, with a gain of 0.1.
        this->pid = common::PID(0.1, 0.001, 0.001);
      
        // Apply the P-controller to the joint.
        this->model->GetJointController()->SetVelocityPID(
            this->joint->GetScopedName(), this->pid);
      
        // Set the joint's target velocity. This target velocity is just
        // for demonstration purposes.
        // this->model->GetJointController()->SetVelocityTarget(
        //     this->joint->GetScopedName(), 10.0);

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
          ros::SubscribeOptions::create<std_msgs::Float32>(
              "/" + this->model->GetName() + "/manual_control/steering",
              1,
              boost::bind(&autonomos_plugin::OnRosMsg, this, _1),
              ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);
        
        // Spin up the queue helper thread.
        this->rosQueueThread =
          std::thread(std::bind(&autonomos_plugin::QueueThread, this));
      }

      /// \brief Set the velocity of the Velodyne
      /// \param[in] _vel New target velocity
      public: void SetPosition(const double &_pos)
      {
        // Set the joint's target velocity.
        this->model->GetJointController()->SetPositionTarget(
            this->joint->GetScopedName(), _pos);
      }
      /// \brief Handle an incoming message from ROS
      /// \param[in] _msg A float value that is used to set the velocity
      /// of the Velodyne.
      public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
      {
        this->SetPosition(_msg->data);
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

    };
        
        // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
        GZ_REGISTER_MODEL_PLUGIN(autonomos_plugin)
      }
#endif