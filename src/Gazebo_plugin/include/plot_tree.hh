#ifndef PLOT_TREE_HH_
#define PLOT_TREE_HH_

// std
#include <memory>
#include <mutex>

// gazebo
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Color.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/Node.hh>
// #include <gazebo/rendering/Visual.hh>

// #include <gazebo/rendering/RenderTypes.hh>
// #include <gazebo/rendering/DynamicLines.hh>
// #include <rendering/rendering.hh>
// #include <gazebo/rendering/Visual.hh>
// #include <gazebo/rendering/Scene.hh>

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

// namespace gazebo
// {
using namespace gazebo;

class plot_tree_plugin : public ModelPlugin
{
    /// \brief Constructor.
    public: plot_tree_plugin();

  /// \brief Destructor.
  public: ~plot_tree_plugin();

  // Documentation inherited
  public: virtual void Load(physics::ModelPtr _model,
      sdf::ElementPtr _sdf);

  /// \brief Update the plugin once every iteration of simulation.
  // private: void Update();

  /// \brief Callback to receive info.
  // private: void OnInfo(ConstPosesStampedPtr &_msg);

  private: bool get_next_line_segment(
                gazebo_plugin::gz_visual_line::Request &req,
                gazebo_plugin::gz_visual_line::Response &res);

  /// \internal
  /// \brief Private data pointer
  // private: std::unique_ptr<plot_tree_plugin_private> dataPtr;

  private: std::unique_ptr<ros::NodeHandle> rosNode;

  private: ros::ServiceServer ros_service_server;

  private: ignition::msgs::Marker msg;

  private: ignition::transport::Node ign_node;

  private: physics::ModelPtr model;

  private: int id;
  /// \brief For example a line to visualize the force
  // private: rendering::DynamicLines *line;

    // private: rendering::VisualPtr visual;

};
#endif