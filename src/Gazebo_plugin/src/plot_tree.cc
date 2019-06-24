#include "plot_tree.hh"

using std::endl;
using std::cout;
using std::string;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(plot_tree_plugin)
// GZ_REGISTER_WORLD_PLUGIN(plot_tree_plugin)
// GZ_REGISTER_SYSTEM_PLUGIN(plot_tree_plugin)
// GZ_REGISTER_VISUAL_PLUGIN(plot_tree_plugin)
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
// void plot_tree_plugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
// void plot_tree_plugin::Load(int /*_argc*/, char ** /*_argv*/)
// void plot_tree_plugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
void plot_tree_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzdbg  << "plot_tree_plugin: loading..." << endl;
  GZ_ASSERT(_model, "LinkPlot3DPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LinkPlot3DPlugin _sdf pointer is NULL");
  
  this -> model = _model;

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "plot_tree_plugin", ros::init_options::NoSigintHandler);
  }
  gzdbg << "plot_tree_plugin: ROS initialized..." << endl;

  // // Create our ROS node. This acts in a similar manner to the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  
  
  string topic_name = "/" + this -> model -> GetName() + "/gz_visual/line_segment";

  cout << "topic_name: " << topic_name << endl;
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<gazebo_plugin::Line_Segment>(
      topic_name,
      100000,
      boost::bind(&plot_tree_plugin::get_next_line_segment, this, _1),
      ros::VoidPtr(), NULL
      // , &this->rosQueue
      );

  this->rosSub = this->rosNode->subscribe(so);
  
  ros::AsyncSpinner spinner(2); // Use 4 threads
  spinner.start();
  // this->rosQueueThread =
    // std::thread(std::bind(&plot_tree_plugin::QueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&plot_tree_plugin::OnUpdate, this, _1));

}

void plot_tree_plugin::get_next_line_segment(const gazebo_plugin::Line_SegmentConstPtr &msg)
{

  gzdbg << __PRETTY_FUNCTION__ << endl;
  gazebo_plugin::Line_Segment aux;
  aux.header    = msg -> header;
  aux.line_list = msg -> line_list;
  aux.points    = msg -> points;
  aux.color     = msg -> color;
  aux.action    = msg -> action;
  this -> lines_queue.push(aux);

}

/////////////////////////////////////////////////
void plot_tree_plugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  // gzdbg << __PRETTY_FUNCTION__ << endl;
  if(! this -> lines_queue.empty())
  {
    std::hash<string> ptr_hash;
    ignition::transport::Node node;
    ignition::msgs::Marker markerMsg;
    markerMsg.set_ns(this -> model -> GetName());

    gazebo_plugin::Line_Segment msg;// = this -> lines_queue.front();
    ignition::msgs::Material *matMsg = markerMsg.mutable_material();

    msg = this -> lines_queue.front();
    this -> lines_queue.pop();
    markerMsg.set_id(ptr_hash(msg.header.frame_id));
    // TODO: separate ids and use msg add_modify
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);

    if (msg.line_list)
    {
      markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
    }
    else
    {
      markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
    }

    matMsg->mutable_script()->set_name(msg.color);

    for (auto&  pt : msg.points) 
    {
      // gzdbg << "msg: " << msg.header.frame_id << " "
      //   << pt.x << " " << pt.y << " " << pt.z << " "
      //   << endl;
      
      ignition::msgs::Set(markerMsg.add_point(),
          ignition::math::Vector3d(pt.x, pt.y, pt.z));
    }

    // gzdbg << "msg: " << msg.header.frame_id << " "
    //   << msg.start.x << " " << msg.start.y << " " << msg.start.z << " "
    //   << msg.end.x   << " " << msg.end.y   << " " << msg.end.z
    //   << endl;

    node.Request("/marker", markerMsg);
  }
  
}

void plot_tree_plugin::QueueThread()
{
  static const double timeout = 0.01;
  // 
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}   

/////////////////////////////////////////////////
// void plot_tree_plugin::OnInfo(ConstPosesStampedPtr &_msg)
// {
  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // this->dataPtr->currentSimTime = msgs::Convert(_msg->time());
// }
