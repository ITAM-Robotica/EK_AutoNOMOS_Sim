#include "plot_tree.hh"

using std::endl;
using std::cout;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(plot_tree_plugin)

/////////////////////////////////////////////////
plot_tree_plugin::plot_tree_plugin()
{
  gzdbg  << "plot_tree_plugin: constructor..." << endl;
  this -> id = 0;
}

/////////////////////////////////////////////////
plot_tree_plugin::~plot_tree_plugin()
{
  gzdbg << "plot_tree_plugin: destroying..." << endl;

  this -> rosNode -> shutdown();
  this -> rosNode.release();
  // delete this -> rosNode;
}

/////////////////////////////////////////////////
void plot_tree_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzdbg  << "plot_tree_plugin: loading..." << endl;
  GZ_ASSERT(_model, "LinkPlot3DPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LinkPlot3DPlugin _sdf pointer is NULL");
  
  // std::vector<std::string> serviceList;
  // this -> ign_node.ServiceList(serviceList);

  // if (std::find(serviceList.begin(), serviceList.end(), "/marker")
  //     == serviceList.end())
  // {
  //   gzerr << "Error: /marker service not present on network.\n";
  //   return;
  // }

  // this -> ign_node.Advertise<ignition::msgs::Marker>("/marker");

  this -> model = _model;
  // this -> visual = _visual;

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "plot_tree_plugin", ros::init_options::NoSigintHandler);
  }
  gzdbg << "plot_tree_plugin: ROS initialized..." << endl;

  // // Create our ROS node. This acts in a similar manner to the Gazebo node
  ros::NodeHandle nh("plot_tree_plugin_nh"); 
  this->rosNode.reset(&nh);
  
  // trj_seg_msg.request.seq = 0;
  this -> ros_service_server = nh.advertiseService("/gz_visual/line_segment", 
    &plot_tree_plugin::get_next_line_segment, this);
    // boost::bind(&plot_tree_plugin::get_next_line_segment, this, _1, _2));

  // ros::ServiceClientOptions sco = ros::ServiceClientOptions::init(srv_name, false, header);
  // this -> ros_service_client = nh.serviceClient<gazebo_plugin::trajectory_segment>("/navigation/get_next_control");

  // Spin up the queue helper thread.
  // this->rosQueueThread = std::thread(std::bind(&autonomos_ideal_plugin::QueueThread, this));

  // this -> ideal_move_thread = std::thread(std::bind(&autonomos_ideal_plugin::move_robot, this));
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  // this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    // boost::bind(&autonomos_ideal_plugin::OnUpdate, this, _1));
  // this -> model -> SetJointPosition("steer_joint", 0);


  // Message
  // ignition::msgs::Marker markerMsg;
  // markerMsg.set_ns("tree_" + this -> model -> GetName());
  // markerMsg.set_id(0);
  // markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  // markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);

  // std::string mat;
  // mat = "Gazebo/Black";
  // ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  // matMsg->mutable_script()->set_name(mat);

  // // this -> msg = markerMsg;
  // ignition::msgs::Set(markerMsg.add_point(),
  //   ignition::math::Vector3d(2,2,0));
  // ignition::msgs::Set(markerMsg.add_point(),
  //   ignition::math::Vector3d(1,1,0));
  // this -> ign_node.Request("/marker", markerMsg);

  // if (!this -> ign_node.Request("/marker", msg))
  //   gzerr << "Error adding a marker\n";

  // gzdbg << "plot_tree_plugin: plugin loaded" << endl;

}

bool plot_tree_plugin::get_next_line_segment(gazebo_plugin::gz_visual_line::Request &req,
                 gazebo_plugin::gz_visual_line::Response &res)
{
  // this->line = this -> visual -> CreateDynamicLine(rendering::RENDERING_LINE_STRIP);

  gzdbg << req << endl;

  // ignition::msgs::Marker markerMsg;
  // ignition::msgs::Material *matMsg = markerMsg.mutable_material();

  // matMsg->mutable_script()->set_name("Gazebo/Blue");
  // markerMsg.set_id(2);
  // markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  // markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
  // // markerMsg.clear_point();

  // ignition::msgs::Set(markerMsg.add_point(),
  //   ignition::math::Vector3d(req.start.x, req.start.y, req.start.z));
  // ignition::msgs::Set(markerMsg.add_point(),
  //   ignition::math::Vector3d(req.end.x, req.end.y, req.end.z));

  // // this->dataPtr->node.Request("/marker", plot.msg);
  // this -> ign_node.Request("/marker", markerMsg);

  // res.success = 1;
  
  // if (!this -> ign_node.Request("/marker", msg))
  // {
  //   gzerr << "Error adding a marker\n";
  // } 
  // else 
  // {
  //   gzdbg << "request successful" << endl;
  // }

  ignition::transport::Node node;

  // Create the marker message
  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  markerMsg.set_id(0);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::SPHERE);

  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/BlueLaser");

  // The rest of this function adds different shapes and/or modifies shapes.
  // Read the terminal statements to figure out what each node.Request
  // call accomplishes.

  std::cout << "Spawning a sphere at the origin\n";
  gazebo::common::Time::Sleep(4);
  node.Request("/marker", markerMsg);

  std::cout << "Moving the sphere to x=0, y=0, z=1\n";
  gazebo::common::Time::Sleep(4);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(0, 0, 1, 0, 0, 0));
  node.Request("/marker", markerMsg);

  std::cout << "Shrinking the sphere\n";
  gazebo::common::Time::Sleep(4);
  ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(0.2, 0.2, 0.2));
  node.Request("/marker", markerMsg);

  std::cout << "Changing the sphere to red\n";
  gazebo::common::Time::Sleep(4);
  matMsg->mutable_script()->set_name("Gazebo/Red");
  node.Request("/marker", markerMsg);

  std::cout << "Adding a green box\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(1);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::BOX);
  matMsg->mutable_script()->set_name("Gazebo/Green");
  ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(1.0, 1.0, 1.0));
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(2, 0, .5, 0, 0, 0));
  node.Request("/marker", markerMsg);

  std::cout << "Change the green box to a cylinder\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_type(ignition::msgs::Marker::CYLINDER);
  node.Request("/marker", markerMsg);

  std::cout << "Adding a line between the sphere and cylinder\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(2);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 0, 1.1));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 0, 0.5));
  node.Request("/marker", markerMsg);

  std::cout << "Adding a square around the origin\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(3);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
  ignition::msgs::Set(markerMsg.mutable_point(0),
      ignition::math::Vector3d(0.5, 0.5, 0.05));
  ignition::msgs::Set(markerMsg.mutable_point(1),
      ignition::math::Vector3d(0.5, -0.5, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(-0.5, -0.5, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(-0.5, 0.5, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0.5, 0.5, 0.05));
  node.Request("/marker", markerMsg);

  std::cout << "Adding 100 points inside the square\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(4);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::POINTS);
  markerMsg.clear_point();
  for (int i = 0; i < 100; ++i)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(
          ignition::math::Rand::DblUniform(-0.49, 0.49),
          ignition::math::Rand::DblUniform(-0.49, 0.49),
          0.05));
  }
  node.Request("/marker", markerMsg);

  std::cout << "Adding a semi-circular triangle fan\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(6);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 1.5, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.05));
  double radius = 2;
  for (double t = 0; t <= M_PI; t+= 0.01)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(radius * cos(t), radius * sin(t), 0.05));
  }
  node.Request("/marker", markerMsg);

  std::cout << "Adding two triangles using a triangle list\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(7);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, -1.5, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.05));

  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 1, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 2, 0.05));

  node.Request("/marker", markerMsg);

  std::cout << "Adding a rectangular triangle strip\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(8);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(-2, -2, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 1, 0.05));

  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 2, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 2, 0.05));

  node.Request("/marker", markerMsg);

  std::cout << "Delete all the markers\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
  node.Request("/marker", markerMsg);

  return true;
}

/////////////////////////////////////////////////
// void plot_tree_plugin::Update()
// {

// }

/////////////////////////////////////////////////
// void plot_tree_plugin::OnInfo(ConstPosesStampedPtr &_msg)
// {
  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // this->dataPtr->currentSimTime = msgs::Convert(_msg->time());
// }
