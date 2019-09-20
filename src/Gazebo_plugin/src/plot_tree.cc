#include "plot_tree.hh"

using std::endl;
using std::cout;
using std::string;
using std::vector;
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

  // TODO: populate this vector from file  
  vector<string> materials_names = {
  "0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100", "0to10"};
  vector<string> normal_names = {"Gazebo/Green"};
  // for (int i = 0; i <= 100; i+=10)
  for(auto& i : materials_names)
  {
    ignition::msgs::Marker markerMsg;
    ignition::msgs::Material *matMsg = markerMsg.mutable_material();
    std::stringstream ss_name, ss_uri;

    ss_name << "EK/percent_" << i;
    ss_uri << "file://materials/scripts/percent_" << i << ".material";
    gzdbg << "name: " << ss_name.str() << "\turi: " << ss_uri.str() << endl;
    markerMsg.set_ns(this -> model -> GetName());
    matMsg -> mutable_script() -> set_name(ss_name.str());
    matMsg -> mutable_script() -> add_uri(ss_uri.str());
    // matMsg -> mutable_script() -> set_name("EK/percent_10");
    // matMsg -> mutable_script() -> add_uri ("file://materials/scripts/percent_10.material");
    v_markers_msg.push_back(markerMsg);

  }

  for (auto& i : normal_names)
  {
    ignition::msgs::Marker markerMsg;
    ignition::msgs::Material *matMsg = markerMsg.mutable_material();

    markerMsg.set_ns(this -> model -> GetName());
    matMsg -> mutable_script() -> set_name(i);
    // matMsg -> mutable_script() -> add_uri(ss_uri.str());
    v_markers_msg.push_back(markerMsg);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&plot_tree_plugin::OnUpdate, this, _1));
}

void plot_tree_plugin::get_next_line_segment(const gazebo_plugin::Line_SegmentConstPtr &msg)
{

  // gzdbg << __PRETTY_FUNCTION__ << endl;
  gazebo_plugin::Line_Segment aux;
  aux.header    = msg -> header;
  aux.line_list = msg -> line_list;
  aux.points    = msg -> points;
  aux.color     = msg -> color;
  aux.action    = msg -> action;
  aux.risk    = msg -> risk;
  this -> lines_queue.push(aux);

}

/////////////////////////////////////////////////
void plot_tree_plugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  if(! this -> lines_queue.empty())
  {
    std::hash<string> ptr_hash;
    // ignition::transport::Node node;
    gazebo_plugin::Line_Segment msg;// = this -> lines_queue.front();

    msg = this -> lines_queue.front();
    this -> lines_queue.pop();
    // gzdbg << "v_markers_msg size: " << v_markers_msg.size() << endl;
    ignition::math::Vector3d aux;
    bool first = true;
    int index = 0;
    auto it_risk = msg.risk.begin();
    for (auto&  pt : msg.points) 
    {
      index = (int)(*it_risk * 10);
      if (*it_risk > 0 && *it_risk < 0.1)
      {
        index = 11;
        // printf("ARTIF: risk: %f\tindex: %d\tpt: ( %.3f, %.3f, %.3f )\n", *it_risk, index, pt.x, pt.y, pt.z);
      }
      else if (*it_risk == -1)
      {
        index = 12;
        // printf("SLN: risk: %f\tindex: %d\tpt: ( %.3f, %.3f, %.3f )\n", *it_risk, index, pt.x, pt.y, pt.z);
      }
      
      // if ( *it_risk > 0)
      //   printf("risk: %f\tindex: %d\tpt: ( %.3f, %.3f, %.3f )\n", *it_risk, index, pt.x, pt.y, pt.z);
      
      if ( !first )
      {
        // ignition::msgs::Set(v_markers_msg[index].add_point(), aux);
        ignition::msgs::Set(v_markers_msg[index].add_point(),
          ignition::math::Vector3d(pt.x, pt.y, pt.z + index / 1000.0));
      }
      else
      {
        first = false;
      }

      aux = ignition::math::Vector3d(pt.x, pt.y, pt.z + index / 1000.0);
      it_risk++;
    }

    int d = 0;
    // std::reverse(v_markers_msg.begin(), v_markers_msg.end());
    for (auto& markerMsg : v_markers_msg)
    {
      std::stringstream ss_id;
      ss_id << msg.header.frame_id << "_" << d;
      markerMsg.set_id(ptr_hash(ss_id.str()));
      // TODO: separate ids and use msg add_modify
      markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);

      // if (!msg.line_list)
      // {
      //   markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
      // }
      // else
      // {
        markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
      // }
      d++;
      node.Request("/marker", markerMsg);
      markerMsg.clear_point();
    }

    // std::reverse(v_markers_msg.begin(), v_markers_msg.end());

  }
  // if (v_markers_msg[markers_index].point_size() > 0 )
  // {
  //   node.Request("/marker", v_markers_msg[markers_index]);
  // }
  
}

// int plot_tree_plugin::get_risk_color(float risk)
// {
//   
// }

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
