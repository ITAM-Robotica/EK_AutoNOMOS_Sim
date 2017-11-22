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

        // if (_sdf->HasElement("debug"))
        // {
        //   std::cout << "Debugging true" << std::endl;
        // } else {
        //   std::cout << "Debugging false" << std::endl;
        // }
      
        // Store the model pointer for convenience.
        this->model = _model;
      
        const std::string val = "Printing from plugin...";
        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        this->joint = _model->GetJoint("steer_joint");

        this->joint_left_wheel = _model->GetJoint("back_left_wheel_joint");
        this->joint_right_wheel = _model->GetJoint("back_right_wheel_joint");

        std::cout << val << std::endl;
        
        // for (auto & element : _model->GetJoints() ) {
        //   std::cout << element->GetName() << std::endl;
        // }

        std::cout << this->joint->GetName() << std::endl;
        

        // Setup a P-controller, with a gain of 0.1.
        this->pid = common::PID(0.1, 0.001, 0.001);
        this->pid_vel_left = common::PID(0.1, 0.001, 0.001);
        this->pid_vel_right = common::PID(0.1, 0.001, 0.001);
      
        // Apply the P-controller to the joint.
        this->model->GetJointController()->SetPositionPID(
            this->joint->GetScopedName(), this->pid);

        this->model->GetJointController()->SetVelocityPID(
            this->joint_left_wheel->GetScopedName(), this->pid_vel_left);

        this->model->GetJointController()->SetVelocityPID(
            this->joint_right_wheel->GetScopedName(), this->pid_vel_right);
        
        this->position = 0;
        this->vel = 0;
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
              boost::bind(&autonomos_plugin::OnRosMsg_steering, this, _1),
              ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);
        
        ros::SubscribeOptions so_vel =
          ros::SubscribeOptions::create<std_msgs::Float32>(
              "/" + this->model->GetName() + "/manual_control/velocity",
              1,
              boost::bind(&autonomos_plugin::OnRosMsg_vel, this, _1),
              ros::VoidPtr(), &this->rosQueue);
        this->rosSub_vel = this->rosNode->subscribe(so_vel);
        
        
        
        // Spin up the queue helper thread.
        this->rosQueueThread =
          std::thread(std::bind(&autonomos_plugin::QueueThread, this));

          // Listen to the update event. This event is broadcast every
      // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&autonomos_plugin::OnUpdate, this, _1));
      }

      /// \brief Set the velocity of the Velodyne
      /// \param[in] _vel New target velocity
      public: void SetPosition(const double &_pos)
      {
        // Set the joint's target velocity.
        

        // this->model->GetJointController()->SetPositionTarget(
        //     this->joint->GetScopedName(), _pos);

        // this->model->GetJointController()->


      }
      /// \brief Handle an incoming message from ROS
      /// \param[in] _msg A float value that is used to set the velocity
      /// of the Velodyne.
      public: void OnRosMsg_steering(const std_msgs::Float32ConstPtr &_msg)
      {
        this->position = _msg->data;
      }

      public: void OnRosMsg_vel(const std_msgs::Float32ConstPtr &_msg)
      {
        this->vel = _msg->data;
        std::cout << "On OnRosMsg_vel" << this->vel << std::endl;

      }
      
      public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // std::cout << "OnUpdate function"  << std::endl;
        // std::cout << "The target position is: " << this->position << std::endl;
        // std::cout << "The current position is: " << this->joint->GetAngle(0).Radian() << std::endl;
        // std::cout << "The current position is: " << this->joint->GetAngle(0).Degree() << std::endl;


        // compute the steptime for the PID
        common::Time currTime = this->model->GetWorld()->GetSimTime();
        common::Time stepTime = currTime - this->prevUpdateTime;
        this->prevUpdateTime = currTime;

        // set the current position of the joint, and the target position, 
        // and the maximum effort limit
        float pos_target = this->position;
        float pos_curr = this->joint->GetAngle(0).Degree();
        
        float vel_target = this->vel;
        float vel_curr_left = this->joint_left_wheel->GetVelocity(0);
        float vel_curr_right = this->joint_right_wheel->GetVelocity(0);

        // std::cout << "The target vel is: " << vel_target << std::endl;
        // std::cout << "The current vel left is: " << vel_curr_left << std::endl;
        // std::cout << "The current vel right is: " << vel_curr_right << std::endl;

        
        // calculate the error between the current position and the target one
        double pos_err = pos_curr - pos_target;
        double vel_err_left = vel_curr_left - vel_target;
        double vel_err_right = vel_curr_right - vel_target;
        
        // compute the effort via the PID, which you will apply on the joint
        double effort_cmd = this->pid.Update(pos_err, stepTime);
        double effort_cmd_vl = this->pid_vel_left.Update(vel_err_left, stepTime);
        double effort_cmd_vr = this->pid_vel_right.Update(vel_err_right, stepTime);
        // check if the effort is larger than the maximum permitted one
        // effort_cmd = effort_cmd > max_cmd ? max_cmd :
        //                 (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);
        
        // apply the force on the joint
        this->joint->SetForce(0, effort_cmd);
        this->joint_left_wheel->SetForce(0, effort_cmd_vl);
        this->joint_right_wheel->SetForce(0, effort_cmd_vr);
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

        private: float position;

        private: float vel;

        private: event::ConnectionPtr updateConnection;
      /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief Pointer to the joint.
        private: physics::JointPtr joint;
        private: physics::JointPtr joint_left_wheel;
        private: physics::JointPtr joint_right_wheel;

        /// \brief A PID controller for the joint.
        private: common::PID pid;
        private: common::PID pid_vel_left;
        private: common::PID pid_vel_right;

        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        private: common::Time prevUpdateTime;
        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;
        private: ros::Subscriber rosSub_vel;

        private: ros::NodeHandle _nh;
        
        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;
        
        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

    };
        
        // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
        GZ_REGISTER_MODEL_PLUGIN(autonomos_plugin)
      }
#endif