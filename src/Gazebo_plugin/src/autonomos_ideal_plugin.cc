#include "autonomos_ideal_plugin.h"
#include <gazebo/rendering/rendering.hh>
using std::cout;
using std::endl;
using std::string;
using namespace gazebo;

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(autonomos_ideal_plugin)

autonomos_ideal_plugin::autonomos_ideal_plugin()
{
  waiting_valid_traj = true;
}

autonomos_ideal_plugin::~autonomos_ideal_plugin()
{
}

/// \brief The load function is called by Gazebo when the plugin is
/// inserted into simulation
/// \param[in] _model A pointer to the model that this plugin is
/// attached to.
/// \param[in] _sdf A pointer to the plugin's SDF element.
void autonomos_ideal_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzdbg << "autonomos_ideal_plugin: Loading plugin..." << endl;
  GZ_ASSERT(_model, "autonomos_ideal_plugin::Load: _model pointer is NULL");
  GZ_ASSERT(_sdf,   "autonomos_ideal_plugin::Load: _sdf pointer is NULL");

  // Safety check
  if (_model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count, autonomos_plugin not loaded\n";
    return;
  }

  this->pmq.startServiceThread();

  this -> model = _model;
  this -> world = _model -> GetWorld();

  this -> autonomos_pose_connection = 0;

  this -> joint_left_wheel = _model -> GetJoint("back_left_wheel_joint");
  this -> joint_right_wheel = _model -> GetJoint("back_right_wheel_joint");

  // this -> steering_name = this -> joint_steering -> GetScopedName();
  // this -> left_wheel_name = this -> joint_left_wheel -> GetScopedName();
  // this -> right_wheel_name = this -> joint_right_wheel -> GetScopedName();

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
  }

  gzdbg << "autonomos_ideal_plugin: ROS initialized..." << endl;

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  ros::NodeHandle nh("autonomos_ideal_plugin"); 
  this->rosNode.reset(&nh);
  
  // trj_seg_msg.request.seq = 0;

  this -> ros_service_client = nh.serviceClient<gazebo_plugin::trajectory_segment>("/navigation/get_next_control");

  string pub_pose_topic_name = "/" + this -> model -> GetName() + "/pose";
  ros::AdvertiseOptions pose_ao =
    ros::AdvertiseOptions::create<geometry_msgs::Pose2D>(
      pub_pose_topic_name,1,
      boost::bind( &autonomos_ideal_plugin::autonomos_ideal_plugin_pose_connect,this),
      boost::bind( &autonomos_ideal_plugin::autonomos_ideal_plugin_pose_disconnect,this),
      ros::VoidPtr(), NULL);

  string sub_obj_topic_name = "/" + this -> model -> GetName() + "/next_state";
  // ros::SubscribeOptions so_next_state =
  //   ros::SubscribeOptions::create<geometry_msgs::Pose2D>(
  //     sub_obj_topic_name, 1,
  //     boost::bind(&autonomos_ideal_plugin::update_next_state, this, _1),
  //     ros::VoidPtr(), NULL);

  // this -> sub_next_state = this -> rosNode -> subscribe(so_next_state);
  this -> sub_next_state = nh.subscribe(sub_obj_topic_name, 1, 
    &autonomos_ideal_plugin::update_next_state, this);

  this -> pub_pose = this->rosNode->advertise(pose_ao);
  // this -> pub_pose = nh.advertise<geometry_msgs::Pose2D>(pub_pose_topic_name, 1);

  this->rosQueueThread = std::thread(std::bind(&autonomos_ideal_plugin::QueueThread, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&autonomos_ideal_plugin::OnUpdate, this, _1));

  this -> steering_pos_traget = 0;
  this -> velocity_target = 0;

  gzdbg << "autonomos_ideal_plugin: plugin loaded" << endl;
  this -> traj_duration = 0;
  timer.reset();

}

void autonomos_ideal_plugin::autonomos_ideal_plugin_pose_connect()
{
  this->autonomos_pose_connection++;
}

void autonomos_ideal_plugin::autonomos_ideal_plugin_pose_disconnect()
{
  this->autonomos_pose_connection--;

}

void autonomos_ideal_plugin::update_next_state(const geometry_msgs::Pose2DConstPtr &_msg)
{
  gzdbg << "Updating next state marker" << endl;
  ignition::transport::Node node;
  ignition::msgs::Marker markerMsg;

  markerMsg.set_ns(this -> model -> GetName() + "/next_state");
  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  // markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
  matMsg->mutable_script()->set_name("Gazebo/Red");

  markerMsg.clear_point();

  ignition::msgs::Set(markerMsg.mutable_pose(), 
    ignition::math::Pose3d(_msg -> x, _msg -> y, 0, 0, 0, _msg -> theta));

  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(-.5, 0.25, 0));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(-0.25, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0.5, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(-0.5, -0.25, 0));

  node.Request("/marker", markerMsg);

} 

void autonomos_ideal_plugin::next_pose()
{
  // gzdbg << __PRETTY_FUNCTION__ << endl;
  double steering, x_vel_rob, x_vel, y_vel, theta_vel;
  double x_past, y_past, theta_past;
  double t_step;
  geometry_msgs::Pose2D pose;

  // t_step = this -> step_time.Double();
  t_step = this -> step_time.toSec();
  steering = this -> steering_pos_traget;
  x_vel_rob = this -> velocity_target;
  
  #if GAZEBO_VERSION_MAJOR >= 8
    ignition::math::Pose3d current_pose;
    ignition::math::Pose3d new_pose;
    current_pose = this -> model -> WorldPose();
  #else
    gazebo::math::Pose3d current_pose, new_pose;
  #endif

  x_past = current_pose.Pos().X();
  y_past = current_pose.Pos().Y();
  theta_past = current_pose.Rot().Yaw();
  x_past     += t_step * cos(theta_past) * x_vel_rob ;
  y_past     += t_step * sin(theta_past) * x_vel_rob ;
  // theta_past += t_step * steering;
  theta_past += t_step * tan(steering) * x_vel_rob / REAR_FRONT_DISTANCE;

  current_pose.Set(x_past, y_past, 0, 0, 0, theta_past);
  pose.x = x_past;
  pose.y = y_past;
  pose.theta = theta_past;

  // gzdbg << "\ttheta_past: " << theta_past
  //   << "\tx: " << current_pose.Pos().X()
  //   << "\ty: " << current_pose.Pos().Y() 
  //   << "\t: " << current_pose.Rot().Yaw()
  //   << endl;

  this -> model -> SetWorldPose(current_pose); 

  this->autonomos_pose_connection++;
  if (this->autonomos_pose_connection > 0)
  {
    this -> pub_pose.publish(pose);
  }

}

void autonomos_ideal_plugin::autonomos_connect()
{
  printf("At: %s\n",__PRETTY_FUNCTION__);
}

void autonomos_ideal_plugin::autonomos_disconnect()
{
  printf("At: %s\n",__PRETTY_FUNCTION__);    
}


void autonomos_ideal_plugin::OnUpdate(const common::UpdateInfo &)
{    
  prevUpdateTime = current_time;
  // #if GAZEBO_VERSION_MAJOR >= 8
  //   this -> current_time = this -> model -> GetWorld() -> SimTime();
  // #else
  //   this -> current_time = this -> model -> GetWorld() -> GetSimTime();
  // #endif
  current_time = ros::Time::now();
  step_time = current_time - prevUpdateTime;
  // // cout << "plugin OnUpdate" << endl;
  // common::Time diff_time;
  // ros::Duration diff_time;
  // if (!waiting_valid_traj)
  // {
  // diff_time = seg_fin_time - this -> world -> SimTime();
  // ROS_WARN_STREAM("traj_duration: " << traj_duration);
  // if (timer.measure() >= this -> traj_duration)
  // if ( diff_time <= ros::Duration(0.0) )
  if ( traj_duration <= current_time.toSec() )
  { 
    // gzdbg << "duration: " << traj_duration << endl;
    if (this -> ros_service_client.call(this -> trj_seg_msg))
    {
      // ROS_DEBUG("Now: %.4f\tdur: %.4f,gz_clock: %.4f", 
      //   current_time.toSec(), trj_seg_msg.response.duration, this -> world -> SimTime().Double());

      gzdbg << "New segment: " << trj_seg_msg.response.seq << endl;
      gzdbg << "\tSpeed: " << trj_seg_msg.response.speed << endl;
      gzdbg << "\tSteering: " << trj_seg_msg.response.steering << endl;
      gzdbg << "\tDuration: " << trj_seg_msg.response.duration << endl;

      // seg_init_time = this -> world -> SimTime();
      // seg_fin_time.Set(trj_seg_msg.response.duration);
      // seg_fin_time += seg_init_time;
      // seg_init_time = ros::Time::Now();
      this -> traj_duration  = current_time.toSec() + trj_seg_msg.response.duration;
      // ROS_WARN("traj_duration UPDATED!!!!");
      // seg_fin_time += seg_init_time;
      // timer.reset();
      // this -> traj_duration = trj_seg_msg.response.duration;
      this -> velocity_target = trj_seg_msg.response.speed;
      this -> steering_pos_traget = trj_seg_msg.response.steering;

    }
    else
    {
      this -> velocity_target     = 0;
      this -> steering_pos_traget = 0;
    }
  }

  next_pose();
}


/// \brief ROS helper function that processes messages
void autonomos_ideal_plugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}      