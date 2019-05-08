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
  
  trj_seg_msg.request.seq = 0;

  this -> ros_service_client = nh.serviceClient<gazebo_plugin::trajectory_segment>("/navigation/get_next_control");

  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&autonomos_ideal_plugin::QueueThread, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&autonomos_ideal_plugin::OnUpdate, this, _1));

  this -> steering_pos_traget = 0;
  this -> velocity_target = 0;

  gzdbg << "autonomos_ideal_plugin: plugin loaded" << endl;

  
}

void autonomos_ideal_plugin::next_pose()
{
  double steering, x_vel_rob, x_vel, y_vel, theta_vel;
  double x_past, y_past, theta_past;
  double t_step;

  t_step = this -> step_time.Double();
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
  theta_past += t_step * steering;

  current_pose.Set(x_past, y_past, 0, 0, 0, theta_past);
  
  // gzdbg << "\ttheta_past: " << theta_past
  //   << "\tx: " << current_pose.Pos().X()
  //   << "\ty: " << current_pose.Pos().Y() 
  //   << "\t: " << current_pose.Rot().Yaw()
  //   << endl;

  this -> model -> SetWorldPose(current_pose); 

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
  this -> prevUpdateTime = this -> current_time;
  #if GAZEBO_VERSION_MAJOR >= 8
    this -> current_time = this -> model -> GetWorld() -> SimTime();
  #else
    this -> current_time = this -> model -> GetWorld() -> GetSimTime();
  #endif
  this -> step_time = this -> current_time - this->prevUpdateTime;
  // cout << "plugin OnUpdate" << endl;
  common::Time diff_time;
  if (!waiting_valid_traj)
  {
    diff_time = seg_fin_time - this -> world -> SimTime();

    // cout << "\t Initial: " << seg_init_time.sec << "." << seg_init_time.nsec << endl;
    // cout << "\t   Final: " << seg_fin_time.sec << "." << seg_fin_time.nsec << endl;
    // cout << "\tSteering: " << trj_seg_msg.response.steering << endl;
    // cout << "\tVel_desi: " << this -> velocity_target << endl;

    if (diff_time <= 0.0 && this -> ros_service_client.call(this -> trj_seg_msg))
    {
      if (trj_seg_msg.response.is_valid)
      {
        // cout << "Segment: "    << trj_seg_msg.request.seq - 1 << "\tStep: " << this->step_time.Double() << endl;
        // cout << "\tDuration: " << trj_seg_msg.response.duration << endl;
        // cout << "\tVel_real: " << this -> model -> RelativeLinearVel().X() << endl; 

        seg_init_time = this -> world -> SimTime();
        seg_fin_time.Set(trj_seg_msg.response.duration);
        seg_fin_time += seg_init_time;
        
        this -> velocity_target = trj_seg_msg.response.speed;
        this -> steering_pos_traget = trj_seg_msg.response.steering;

        trj_seg_msg.request.seq++;

      }
      else
      {
        this -> velocity_target = 0;
        this -> steering_pos_traget = 0;
        seg_fin_time = this -> world -> SimTime();
      }
    }
  }
  else if(waiting_valid_traj && this -> ros_service_client.call(trj_seg_msg))
  {

    if(trj_seg_msg.response.is_valid)
    {
      // cout << "valid message" << endl;
      seg_init_time =  this -> world -> SimTime();
      seg_fin_time.Set(trj_seg_msg.response.duration);
      seg_fin_time += seg_init_time;
      
      this -> linear_vel.Set(trj_seg_msg.response.speed, 0, 0);
      // this -> model -> SetLinearVel(this -> linear_vel);
      this -> steering_pos_traget = trj_seg_msg.response.steering;
      
      trj_seg_msg.request.seq++;
      waiting_valid_traj = false;
      
      cout << "Segment: " << trj_seg_msg.request.seq - 1 << endl;
      cout << "\t Initial: " << seg_init_time.sec << "." << seg_init_time.nsec << endl;
      cout << "\t   Final: " << seg_fin_time.sec << "." << seg_fin_time.nsec << endl;
      cout << "\tSteering: " << trj_seg_msg.response.steering << endl;
      cout << "\tVelocity: " << trj_seg_msg.response.speed << endl;
      cout << "\tDuration: " << trj_seg_msg.response.duration << endl;

      // cout << "\tDiff: " << diff_time.sec << "." << diff_time.nsec << endl;
    }
    else
    {
      // cout << "NOT a valid message" << endl;
    }
  }
  
  // gzdbg << "Seq: "    << trj_seg_msg.request.seq - 1 << "\tvalid: " << trj_seg_msg.response.is_valid 
  //   << "\ttime: " << trj_seg_msg.response.duration << "\tt_left: " << diff_time.Double()
  //   << "\tvel: "  << this -> velocity_target << "\tste: " << this -> steering_pos_traget
  //   << endl;

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