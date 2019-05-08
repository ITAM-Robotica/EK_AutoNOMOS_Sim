#ifndef PLOT_TREE_HH_
#define PLOT_TREE_HH_

// std
#include <mutex>
#include <memory>
#include <thread>

// gazebo
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/PubQueue.h>

// ignition
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

// gazebo_plugin
#include "gazebo_plugin/gz_visual_line.h"
#include "gazebo_plugin/Line_Segment.h"


// namespace gazebo
// {
using namespace gazebo;

// class plot_tree_plugin : public WorldPlugin
// class plot_tree_plugin : public SystemPlugin
// class GAZEBO_VISIBLE plot_tree_plugin : public VisualPlugin
class plot_tree_plugin : public ModelPlugin
{
    /// \brief Constructor.
    public: plot_tree_plugin();

  /// \brief Destructor.
  public: ~plot_tree_plugin();

  // Documentation inherited
  public: 
    // virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf );
    // virtual void Load(int /*_argc*/, char ** /*_argv*/);
    // virtual void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  void OnUpdate(const common::UpdateInfo & /*_info*/);
  /// \brief Update the plugin once every iteration of simulation.
  // private: void Update();

  /// \brief Callback to receive info.
  // private: void OnInfo(ConstPosesStampedPtr &_msg);

  private: //bool get_next_line_segment(
           //     gazebo_plugin::gz_visual_line::Request &req,
           //     gazebo_plugin::gz_visual_line::Response &res);

  void get_next_line_segment(const gazebo_plugin::Line_SegmentConstPtr &msg);

  void QueueThread();

  ros::Subscriber rosSub;
  
  ros::CallbackQueue rosQueue;

  std::thread rosQueueThread;

  ros::Subscriber sub_start;

  // std::vector<gazebo_plugin::Line_SegmentPtr> lines_queue;
  std::queue<gazebo_plugin::Line_Segment> lines_queue;
  /// \internal
  /// \brief Private data pointer
  // private: std::unique_ptr<plot_tree_plugin_private> dataPtr;

  private: std::unique_ptr<ros::NodeHandle> rosNode;

  private: ros::ServiceServer ros_service_server;

  private: ignition::msgs::Marker msg;

  private: ignition::transport::Node ign_node;

  private: physics::ModelPtr model;

  private: int id;

  event::ConnectionPtr updateConnection;

  public: 
    // std::unique_ptr<rendering::DynamicLines> dynamicRenderable;
    rendering::DynamicLines *dynamicRenderable;

  rendering::VisualPtr world_visual_ptr;

  public: bool loaded = false;
  // private: MarkerVisualPrivate *dPtr;


  /// \brief For example a line to visualize the force
  // private: rendering::DynamicLines *line;

    // private: rendering::VisualPtr visual;

};
#endif