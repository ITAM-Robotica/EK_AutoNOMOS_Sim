#include "obstacles.h"
// #include <gazebo/rendering/rendering.hh>
using std::cout;
using std::endl;
using std::string;
using namespace gazebo;

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_WORLD_PLUGIN(obstacles_plugin)

obstacles_plugin::obstacles_plugin()
{

}

obstacles_plugin::~obstacles_plugin()
{
}

/// \brief The load function is called by Gazebo when the plugin is
/// inserted into simulation
/// \param[in] _model A pointer to the model that this plugin is
/// attached to.
/// \param[in] _sdf A pointer to the plugin's SDF element.
void obstacles_plugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  gzdbg << "obstacles_plugin: Loading plugin..." << endl;
  GZ_ASSERT(_parent, "obstacles_plugin::Load: _model pointer is NULL");
  GZ_ASSERT(_sdf,   "obstacles_plugin::Load: _sdf pointer is NULL");

  // Safety check
  // if (_model->GetJointCount() == 0)
  // {
  //   std::cerr << "Invalid joint count, autonomos_plugin not loaded\n";
  //   return;
  // }

  // this->pmq.startServiceThread();

  // this -> model = _model;
  this -> world = _parent;

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

  gzdbg << "obstacles_plugin: ROS initialized..." << endl;

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  ros::NodeHandle nh("obstacles_plugin"); 
  this->rosNode.reset(&nh);
  
  // trj_seg_msg.request.seq = 0;

  this -> ros_serv = nh.advertiseService(
    "/simulation/get_obstacles", &obstacles_plugin::get_obstacles, this);
  // this -> ros_serv_obs_dyn = nh.advertiseService(
  //   "/simulation/get_obstacles/dynamic", &obstacles_plugin::get_obstacles_dynamic, this);

  // string sub_obj_topic_name = "/" + this -> model -> GetName() + "/next_state";

  // this -> sub_next_state = this -> rosNode -> subscribe(so_next_state);
  // this -> sub_next_state = nh.subscribe(sub_obj_topic_name, 1, 
  //   &obstacles_plugin::update_next_state, this);

  // this -> pub_pose = this->rosNode->advertise(pose_ao);
  // this -> pub_pose = nh.advertise<geometry_msgs::Pose2D>(pub_pose_topic_name, 1);

  // this->rosQueueThread = std::thread(std::bind(&obstacles_plugin::QueueThread, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&obstacles_plugin::OnUpdate, this, _1));

  // this -> steering_pos_traget = 0;
  // this -> velocity_target = 0;

  gzdbg << "obstacles_plugin: plugin loaded" << endl;
  // this -> traj_duration = 0;
  // timer.reset();
  get_obstacles_seq = 0;
}

// void obstacles_plugin::obstacles_plugin_pose_connect()
// {
//   this->autonomos_pose_connection++;
// }

// void obstacles_plugin::obstacles_plugin_pose_disconnect()
// {
//   this->autonomos_pose_connection--;

// }

bool obstacles_plugin::get_obstacles(gazebo_plugin::obstacles_array::Request  &req, gazebo_plugin::obstacles_array::Response &res)
{
  get_obstacles_seq++;
  physics::Model_V models = world -> Models();
  
  res.header.seq = get_obstacles_seq;
  res.header.stamp = ros::Time::now();
  res.header.frame_id = "world";

  for(auto m : models)
  {
    //if (m -> GetName() != "ground" || m -> GetName() != "ground_plane")
    //gzdbg << "Model name: " << m -> GetName() << "\tGround found: " << m -> GetName().find("ground") << endl;
    if (m -> GetName().find("ground"))
    {
      // gzdbg << "Model name: " << m -> GetName() << endl;
      res.names.push_back(m -> GetName());
      res.bounding_boxes.push_back(get_box_vertices(m -> CollisionBoundingBox()));
      res.bounding_boxes_dimensions.push_back(box_dimensions(m -> CollisionBoundingBox()));
      res.poses.push_back(pose_ign_to_ros(m -> WorldPose()));
      res.is_static.push_back(m -> IsStatic());
    }
  }

  return true;
}

  /** Add the 8 vertices of the box to a polygon msg.
   * 
   * 
   */

/**
 * @brief Add the 8 vertices of the box to a polygon msg
 * @details Add the 8 vertices of the box to a polygon msg according to the following numbering:
 * 
 * @param box Bounding box 
 * @return polygon msg where each point is a vertex of the box
 * 
 * DETAILS:
 *    The vertices are numbered        |        In binary the numbering is:
 *    in decimal as follows:           |                 ijk --> zyx 
 *                                     |   (zyx being the coordinates of the vertice)
 * 
 *           6 ------------- 7                       110-------------111    
 *          /!              /|                       /!              /|    
 *         / !             / |                      / !             / |    
 *        /  !            /  |                     /  !            /  |    
 *       /   !           /   |                    /   !           /   |    
 *      4 --------------5    |                  100 ------------101   |    
 *      |    !          |    |                   |    !          |    |    
 *      |    !          |    |     =======>      |    !          |    |    
 *      |    2 - - - - -|- - 3                   |   010- - - - -|- -011    
 *      |               |   /                    |               |   /     
 *      |  /            |  /                     |  /            |  /      
 *      |               | /                      |               | /       
 *      |/              |/                       |/              |/        
 *      0 ------------- 1                       000 ----------- 001         
 */
geometry_msgs::Polygon obstacles_plugin::get_box_vertices(ignition::math::Box box)
{
  int v_n = 1;
  double x_min, y_min, z_min;
  double x_max, y_max, z_max;
  ignition::math::Vector3d v_max, v_min;
  geometry_msgs::Polygon p;
  v_max = box.Max();
  v_min = box.Min();
  x_min = v_min.X();
  y_min = v_min.Y();
  z_min = v_min.Z();
  x_max = v_max.X();
  y_max = v_max.Y();
  z_max = v_max.Z();

  // gzdbg << "Bounding box: " << box << endl;
  // gzdbg << "######### Vertices #########" << endl;
  // printf("V%d: ( %.2f, %.2f, %.2f )\n", v_n++, x_min, y_min, z_min ); // 0
  // printf("V%d: ( %.2f, %.2f, %.2f )\n", v_n++, x_max, y_min, z_min ); // 1
  // printf("V%d: ( %.2f, %.2f, %.2f )\n", v_n++, x_min, y_max, z_min ); // 2
  // printf("V%d: ( %.2f, %.2f, %.2f )\n", v_n++, x_max, y_max, z_min ); // 3
  // printf("V%d: ( %.2f, %.2f, %.2f )\n", v_n++, x_min, y_min, z_max ); // 4
  // printf("V%d: ( %.2f, %.2f, %.2f )\n", v_n++, x_max, y_min, z_max ); // 5
  // printf("V%d: ( %.2f, %.2f, %.2f )\n", v_n++, x_min, y_max, z_max ); // 6
  // printf("V%d: ( %.2f, %.2f, %.2f )\n", v_n++, x_max, y_max, z_max ); // 7

  for (int i = 0; i < 8; ++i)
  {
    geometry_msgs::Point32 p_i;
    p_i.x =    i     % 2 ? v_max.X() : v_min.X();
    p_i.y = (i >> 1) % 2 ? v_max.Y() : v_min.Y();
    p_i.z = (i >> 2) % 2 ? v_max.Z() : v_min.Z();
    p.points.push_back(p_i);
  }

  return p;
}

geometry_msgs::Point obstacles_plugin::box_dimensions(ignition::math::Box box)
{
  ignition::math::Vector3d v_max, v_min;
  geometry_msgs::Point p;
  v_max = box.Max();
  v_min = box.Min();

  p.x = v_max.X() - v_min.X();
  p.y = v_max.Y() - v_min.Y();
  p.z = v_max.Z() - v_min.Z();

  return p;
}

geometry_msgs::Pose obstacles_plugin::pose_ign_to_ros(ignition::math::Pose3d pose)
{
  geometry_msgs::Pose p;
  p.position.x = pose.Pos().X();
  p.position.y = pose.Pos().Y();
  p.position.z = pose.Pos().Z();

  p.orientation.x = pose.Rot().X();
  p.orientation.y = pose.Rot().Y();
  p.orientation.z = pose.Rot().Z();
  p.orientation.w = pose.Rot().W();

  return p;
}

void obstacles_plugin::update_next_state(const geometry_msgs::Pose2DConstPtr &_msg)
{
  // gzdbg << "Updating next state marker" << endl;
  // ignition::transport::Node node;
  // ignition::msgs::Marker markerMsg;

  // markerMsg.set_ns(this -> model -> GetName() + "/next_state");
  // ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  // markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  // // markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  // markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
  // matMsg->mutable_script()->set_name("Gazebo/Red");

  // markerMsg.clear_point();

  // ignition::msgs::Set(markerMsg.mutable_pose(), 
  //   ignition::math::Pose3d(_msg -> x, _msg -> y, 0, 0, 0, _msg -> theta));

  // ignition::msgs::Set(markerMsg.add_point(),
  //     ignition::math::Vector3d(-.5, 0.25, 0));
  // ignition::msgs::Set(markerMsg.add_point(),
  //     ignition::math::Vector3d(-0.25, 0, 0));
  // ignition::msgs::Set(markerMsg.add_point(),
  //     ignition::math::Vector3d(0.5, 0, 0));
  // ignition::msgs::Set(markerMsg.add_point(),
  //     ignition::math::Vector3d(-0.5, -0.25, 0));

  // node.Request("/marker", markerMsg);

} 

void obstacles_plugin::next_pose()
{
  // double steering, x_vel_rob, x_vel, y_vel, theta_vel;
  // double x_past, y_past, theta_past;
  // double t_step;
  // geometry_msgs::Pose2D pose;

  // // t_step = this -> step_time.Double();
  // t_step = this -> step_time.toSec();
  // steering = this -> steering_pos_traget;
  // x_vel_rob = this -> velocity_target;
  
  // #if GAZEBO_VERSION_MAJOR >= 8
  //   ignition::math::Pose3d current_pose;
  //   ignition::math::Pose3d new_pose;
  //   current_pose = this -> model -> WorldPose();
  // #else
  //   gazebo::math::Pose3d current_pose, new_pose;
  // #endif

  // x_past = current_pose.Pos().X();
  // y_past = current_pose.Pos().Y();
  // theta_past = current_pose.Rot().Yaw();
  // x_past     += t_step * cos(theta_past) * x_vel_rob ;
  // y_past     += t_step * sin(theta_past) * x_vel_rob ;
  // // theta_past += t_step * steering;
  // theta_past += t_step * tan(steering) * x_vel_rob / REAR_FRONT_DISTANCE;

  // current_pose.Set(x_past, y_past, 0, 0, 0, theta_past);
  // pose.x = x_past;
  // pose.y = y_past;
  // pose.theta = theta_past;

  // gzdbg << "\ttheta_past: " << theta_past
  //   << "\tx: " << current_pose.Pos().X()
  //   << "\ty: " << current_pose.Pos().Y() 
  //   << "\t: " << current_pose.Rot().Yaw()
  //   << endl;

  // this -> model -> SetWorldPose(current_pose); 

  // this->autonomos_pose_connection++;
  // if (this->autonomos_pose_connection > 0)
  // {
  //   this -> pub_pose.publish(pose);
  // }

}

void obstacles_plugin::autonomos_connect()
{
  printf("At: %s\n",__PRETTY_FUNCTION__);
}

void obstacles_plugin::autonomos_disconnect()
{
  printf("At: %s\n",__PRETTY_FUNCTION__);    
}


void obstacles_plugin::OnUpdate(const common::UpdateInfo &)
{    

}


/// \brief ROS helper function that processes messages
void obstacles_plugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}      