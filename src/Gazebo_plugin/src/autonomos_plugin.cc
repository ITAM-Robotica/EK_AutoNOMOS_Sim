#include "autonomos_plugin.h"
//#include <gazebo/gazebo.hh>
//#include <gazebo/physics/physics.hh>
//#include <thread>
//#include <ros/ros.h>
//#include <ros/callback_queue.h>
//#include <ros/subscribe_options.h>
//#include <std_msgs/Float32.h>
//#include <std_msgs/Int16.h>

#define ADJ_FACT 1 / 12 // original 1/50

namespace gazebo
{
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(autonomos_plugin)

  autonomos_plugin::autonomos_plugin()
  {

  }

  autonomos_plugin::~autonomos_plugin()
  {

  }

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  void autonomos_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Safety check
    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Invalid joint count, autonomos_plugin not loaded\n";
      return;
    }

     this->pmq.startServiceThread();

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
    ros::SubscribeOptions::create<std_msgs::Int16>(
      "/" + this->model->GetName() + "/manual_control/steering",
      1,
      boost::bind(&autonomos_plugin::OnRosMsg_steering, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    ros::SubscribeOptions so_vel =
    ros::SubscribeOptions::create<std_msgs::Int16>(
      "/" + this->model->GetName() + "/manual_control/speed",
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

     ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<geometry_msgs::Pose2D>(
      "/" + this->model->GetName() + "/real_pose_from_gazebo", 1,
      boost::bind(&autonomos_plugin::autonomos_connect, this),
      boost::bind(&autonomos_plugin::autonomos_disconnect, this),
      ros::VoidPtr(), NULL);

      this->pub_ = this->rosNode->advertise(ao);

      
      this->pub_queue_ = this->pmq.addPub<geometry_msgs::Pose2D>();
  }

  /// \brief Set the velocity of the Velodyne
  /// \param[in] _vel New target velocity
  void autonomos_plugin::autonomos_connect()
  {
    printf("At: %s\n",__PRETTY_FUNCTION__);
  }

  void autonomos_plugin::autonomos_disconnect()
  {
    printf("At: %s\n",__PRETTY_FUNCTION__);    
  }

  void autonomos_plugin::SetPosition(const double &_pos)
  {

  }

  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the Velodyne.
  void autonomos_plugin::OnRosMsg_steering(const std_msgs::Int16ConstPtr &_msg)
  {

      // carro real config:
      // https://github.com/AutoModelCar/model_car/blob/version-3.1-kinetic/catkin_ws/src/fub_steering_calibration/scripts/SteerAngleActuator.xml
      // command 0  : 0.3967852340780831    //  22.7341193   // izquierda
      // command 30 : 0.28655792116953055   //  16.418559
      // command 60 : 0.13636611612784297   //  7.8132029
      // command 90 : -0.01                 //  -0.572957    // centro
      // command 120: -0.139860631021484    //  -8.013423
      // command 150: -0.2675472287070919   //  -15.329327
      // command 180: -0.41853095368099036  //  -23.980057   // derecha

      // sin corrección
      //    this->position = _msg->data - 90;
      // aproximación lineal a angulos reales del autonomos
      this->position = 0.252556 * (_msg->data - 90) + 0.572957; // eliminado signo negativo a ver si con eso

      std::cout << "On OnRosMsg_steering (new): " << this->position << ", msg_rcv: " << _msg->data << std::endl;

  }

  void autonomos_plugin::OnRosMsg_vel(const std_msgs::Int16ConstPtr &_msg)
  {
    // this->vel = _msg->data;
    this->vel = _msg->data * -15 / 31 * ADJ_FACT;

    std::cout << "On OnRosMsg_vel (new): " << this->vel << ", msg_rcv: " << _msg->data << std::endl;
  }

  void autonomos_plugin::OnUpdate(const common::UpdateInfo &)
  {    
    // compute the steptime for the PID
    float pos_target = 0;
    float pos_curr = 0;

    common::Time currTime;
    common::Time stepTime;

    pos_target = this->position;

    #if GAZEBO_VERSION_MAJOR >= 8
      currTime = this->model->GetWorld()->SimTime();
      ignition::math::Angle angle_aux(this->joint->Position(0)); // in radians
      pos_curr = angle_aux.Degree();
    #else
      currTime = this->model->GetWorld()->GetSimTime();
      pos_curr = this->joint->GetAngle(0).Degree();
    #endif
    
    stepTime = currTime - this->prevUpdateTime;
    this->prevUpdateTime = currTime;

    // set the current position of the joint, and the target position, 
    // and the maximum effort limit

    float vel_target = this->vel;
    float vel_curr_left = this->joint_left_wheel->GetVelocity(0);
    float vel_curr_right = this->joint_right_wheel->GetVelocity(0);

    // calculate the error between the current position and the target one
    double pos_err = pos_curr - pos_target;
    double vel_err_left = vel_curr_left - vel_target;
    double vel_err_right = vel_curr_right - vel_target;

    // compute the effort via the PID, which you will apply on the joint
    double effort_cmd = this->pid.Update(pos_err, stepTime);
    double effort_cmd_vl = this->pid_vel_left.Update(vel_err_left, stepTime);
    double effort_cmd_vr = this->pid_vel_right.Update(vel_err_right, stepTime);

    // apply the force on the joint
    this->joint->SetForce(0, effort_cmd);
    this->joint_left_wheel->SetForce(0, effort_cmd_vl);
    this->joint_right_wheel->SetForce(0, effort_cmd_vr);

    double x,y,z;
    geometry_msgs::Pose2D pose_msg;
    #if GAZEBO_VERSION_MAJOR >= 8
      ignition::math::Pose3d pose;
      pose = this->model->WorldPose();
      // ignition::math::Vector3d v(0, 0, 0);

      pose_msg.x = pose.Pos().X();
      pose_msg.y = pose.Pos().Y();
      pose_msg.theta = pose.Rot().Yaw();

    #else
      gazebo::math::Pose pose;     
      pose = this->model->GetWorldPose();
      // math::Vector3 v(0, 0, 0);
      pose_msg.x = pose.pos.x;
      pose_msg.y = pose.pos.y;
      pose_msg.theta = pose.rot.GetYaw();
    #endif

    this->pub_queue_->push(pose_msg, this->pub_);
  }
      
  /// \brief ROS helper function that processes messages
  void autonomos_plugin::QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }      
}
