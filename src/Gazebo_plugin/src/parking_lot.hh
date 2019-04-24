#ifndef _PARKING_LOT_HH_
#define _PARKING_LOT_HH_

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/gazebo.hh>
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <gazebo/common/Time.hh>

#define PROB_SPAWN_CAR 2 //number between 0 and 9
#define COL_SEP 2 //meters between a col and other
#define ROW_SEP 5 //meters between a col and other
namespace gazebo
{
  class parking_lot : public WorldPlugin
  {
    public:
      parking_lot();
      virtual ~parking_lot();
      void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
      void Reset();

    private:
	
      physics::WorldPtr world;

      transport::NodePtr node_1;
      transport::PublisherPtr factoryPub;
      sdf::SDFPtr car_sdfptr;
      sdf::SDFPtr parking_space_sdfptr;
      sdf::SDFPtr lamp_post_sdfptr;

      std::string car_model;
      std::string parking_space_model;
      std::string lamp_post;
      transport::SubscriberPtr sub_info;
      int total_models;

      int car_counter;
      int parking_space_counter;
      int lamp_post_counter;
      int  models_spawned;
      void place_parking_space(ignition::math::Pose3d pose);
      void multiple_parking_spaces(ignition::math::Vector3d center_xyz, int num_spaces, ignition::math::Quaterniond new_quat);
      sdf::SDFPtr get_sdf_file(std::string model_name );

      void change_sdf_model_name(sdf::SDFPtr &sdfElement, std::string name, int &counter  );
      void spawn_car(ignition::math::Vector3d v_xyz, ignition::math::Quaterniond quat);
      void double_parking(ignition::math::Vector3d offset_xyz);
      void row_parking(ignition::math::Vector3d row_init, int length);
      void matrix_parking(ignition::math::Vector3d col_init, int height, int length);
      void spawn_lamp_post(ignition::math::Vector3d v_xyz, ignition::math::Quaterniond quat);
      void multiple_lamp_posts(ignition::math::Vector3d center_xyz, int num_spaces, ignition::math::Quaterniond new_quat, int mod_lamp);
      void spawn_model_callback(ConstWorldStatisticsPtr &_msg);

  };
}
#endif
