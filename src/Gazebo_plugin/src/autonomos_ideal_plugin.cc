#include "autonomos_ideal_plugin.h"

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
  cout << "autonomos_ideal_plugin: Loading plugin..." << endl;

  // Safety check
  if (_model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count, autonomos_plugin not loaded\n";
    return;
  }

  this->pmq.startServiceThread();

  this -> model = _model;
  this -> world = _model -> GetWorld();

  this -> joint_steering    = _model -> GetJoint("steer_joint");
  this -> joint_left_wheel  = _model -> GetJoint("back_left_wheel_joint");
  this -> joint_right_wheel = _model -> GetJoint("back_right_wheel_joint");

  this -> steering_name = this -> joint_steering -> GetScopedName();
  this -> left_wheel_name = this -> joint_left_wheel -> GetScopedName();
  this -> right_wheel_name = this -> joint_right_wheel -> GetScopedName();
  // this->position = 0;
  // this->vel = 0;

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
  }
  cout << "autonomos_ideal_plugin: ROS initialized..." << endl;

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  ros::NodeHandle nh("autonomos_ideal_plugin"); 
  this->rosNode.reset(&nh);
  
  trj_seg_msg.request.seq = 0;

  // ros::ServiceClientOptions sco = ros::ServiceClientOptions::init(srv_name, false, header);
  this -> ros_service_client = nh.serviceClient<gazebo_plugin::trajectory_segment>("/navigation/get_next_control");

  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&autonomos_ideal_plugin::QueueThread, this));

  // this -> ideal_move_thread = std::thread(std::bind(&autonomos_ideal_plugin::move_robot, this));
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&autonomos_ideal_plugin::OnUpdate, this, _1));

  // this -> joint_steering -> SetPosition(2, 0);
  this -> pid_steering. Init(0.1, 0.001, 0.001, 10000, -10000);
  this -> pid_vel_left. Init(1, 0.001, 0.001, 10000, -10000);
  this -> pid_vel_right.Init(1, 0.001, 0.001, 10000, -10000);
  this -> pid_vel_model.Init(500, 100, 1, 100000000, -100000000);

  this -> joint_controller = this -> model -> GetJointController();

  this -> joint_controller -> SetPositionPID(
    this -> steering_name, this -> pid_steering);

  this -> joint_controller -> SetVelocityPID(
    this -> left_wheel_name, this -> pid_vel_left);

  this -> joint_controller -> SetVelocityPID(
    this -> right_wheel_name, this -> pid_vel_right);

  this -> steering_pos_traget = 0;
  this -> velocity_target = 0;

  // this -> model -> SetJointPosition("steer_joint", 0);


  cout << "autonomos_ideal_plugin: plugin loaded" << endl;

  
}

void autonomos_ideal_plugin::next_pose()
{
  float theta, x_vel_rob, x_vel, y_vel, theta_vel;

  theta = this -> steering_pos_traget;
  x_vel_rob = this -> velocity_target;
  
  #if GAZEBO_VERSION_MAJOR >= 8
    ignition::math::Pose3d current_pose, new_pose;
    // ignition::math::Pose3d current_pose, new_pose;
    current_pose = this -> model -> WorldPose();
  #else
    gazebo::math::Pose3d current_pose, new_pose;
  #endif

  x_vel = cos(theta) * x_vel_rob;
  y_vel = sin(theta) * x_vel_rob;
  theta_vel = ( tan(theta) / REAR_FRONT_DISTANCE ) * x_vel_rob;

  new_pose.Set(
    x_vel * this -> step_time.Double(),     // x
    y_vel * this -> step_time.Double(),     // y
    0,                                      // z
    0,                                      // roll
    0,                                      // pitch
    theta_vel * this -> step_time.Double()  // yaw
    );

  new_pose += current_pose;

  this -> model -> SetWorldPose(new_pose); 

}

/// \brief Set the velocity of the Velodyne
/// \param[in] _vel New target velocity
void autonomos_ideal_plugin::autonomos_connect()
{
  printf("At: %s\n",__PRETTY_FUNCTION__);
}

void autonomos_ideal_plugin::autonomos_disconnect()
{
  printf("At: %s\n",__PRETTY_FUNCTION__);    
}

void autonomos_ideal_plugin::SetPosition()
{
  float pos_curr = 0;
  double pos_err = 0;
  double effort_cmd = 0;

  #if GAZEBO_VERSION_MAJOR >= 8
    ignition::math::Angle angle_aux(this -> joint_steering -> Position(0)); // in radians
    pos_curr = angle_aux.Degree();
  #else
    pos_curr = this -> joint -> GetAngle(0).Degree();
  #endif

  pos_err = pos_curr - this -> steering_pos_traget;

  effort_cmd = this -> pid_steering.Update(pos_err, this -> step_time);

  this -> joint_steering -> SetForce(0, effort_cmd);

}

void autonomos_ideal_plugin::drive()
{
  float vel_curr = 0;
  float error_x = 0;
  // double vel_err = 0;
  double pid_res = 0;
  float vel_curr_left = 0;
  // float vel_curr_right = 0;
  double vel_err_left  = 0;
  // double vel_err_right = 0;
  // double effort_cmd_vl = 0;
  // double effort_cmd_vr = 0;

  #if GAZEBO_VERSION_MAJOR >= 8
    ignition::math::Vector3d linear_vel = this -> model -> RelativeLinearVel();
    vel_curr = linear_vel.X();
  #else
    gazebo::math::Vector3d linear_vel = this -> model -> RelativeLinearVel();
    vel_curr = linear_vel.x;
  #endif

  error_x = this -> velocity_target - vel_curr;

  // cout << "\tVelocity: " << trj_seg_msg.response.speed << endl;

  pid_res = -1 * this -> pid_vel_model.Update(error_x, this -> step_time);
  
  this -> joint_controller -> SetVelocityTarget(this -> left_wheel_name, pid_res);
  this -> joint_controller -> SetVelocityTarget(this -> right_wheel_name, pid_res);
  
  vel_curr_left = this -> joint_left_wheel -> GetVelocity(0);
  // vel_curr_right = this -> joint_right_wheel -> GetVelocity(0);

  vel_err_left  = vel_curr_left - this -> velocity_target;
  // vel_err_right = vel_curr_right - pid_res;

  // effort_cmd_vl = this -> pid_vel_left.Update(vel_err_left, this -> step_time);
  cout << "\tErr_vel : " << error_x << "\tpid_res: " << pid_res << endl;
  cout << "\tErr_left: " << vel_err_left << "\tVel_real: " << vel_curr_left << endl;
  // effort_cmd_vr = this -> pid_vel_right.Update(vel_err_right, this -> step_time);

  // cout << "\tErr_lef : " << vel_err_left << "\tpid_res: " << effort_cmd_vl << endl;
  // cout << "\tErr_lef : " << vel_err_right << "\tpid_res: " << effort_cmd_vr << endl;

  // this -> joint_left_wheel -> SetForce(0, effort_cmd_vl);
  // this -> joint_right_wheel -> SetForce(0, effort_cmd_vr);
  // this -> joint_controller -> SetVelocityTarget(this -> left_wheel_name, pid_res);
  // this -> joint_controller -> SetVelocityTarget(this -> right_wheel_name, pid_res);
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
  if (!waiting_valid_traj)
  {
    common::Time diff_time = seg_fin_time - this -> world -> SimTime();

    cout << "Segment: "    << trj_seg_msg.request.seq - 1 << "\tStep: " << this->step_time.Double() << endl;
    cout << "\t Initial: " << seg_init_time.sec << "." << seg_init_time.nsec << endl;
    cout << "\t   Final: " << seg_fin_time.sec << "." << seg_fin_time.nsec << endl;
    cout << "\tDuration: " << trj_seg_msg.response.duration << endl;
    cout << "\tSteering: " << trj_seg_msg.response.steering << endl;
    cout << "\tVel_desi: " << this -> velocity_target << endl;
    cout << "\tVel_real: " << this -> model -> RelativeLinearVel().X() << endl; 

    if (diff_time <= 0.0 && this -> ros_service_client.call(trj_seg_msg))
    {
      if (trj_seg_msg.response.is_valid)
      {
        seg_init_time = this -> world -> SimTime();
        seg_fin_time.Set(trj_seg_msg.response.duration);
        seg_fin_time += seg_init_time;
        
        // this -> linear_vel.Set(trj_seg_msg.response.speed, 0, 0);
        // this -> velocity_target = trj_seg_msg.response.speed;
        this -> velocity_target = 0.5;
        // this -> model -> SetLinearVel(this -> linear_vel);
        this -> steering_pos_traget = trj_seg_msg.response.steering;

        trj_seg_msg.request.seq++;

      }
      else
      {
        // waiting_valid_traj = true;
        // trj_seg_msg.request.seq = 0;
        this -> linear_vel.Set(0, 0, 0);
        // this -> model -> SetLinearVel(this -> linear_vel);
        this -> steering_pos_traget = 0;
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
  
  // SetPosition();
  next_pose();
  // drive();
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