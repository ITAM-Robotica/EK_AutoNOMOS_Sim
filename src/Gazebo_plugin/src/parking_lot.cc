#include <ignition/math/Pose3.hh>
// #include <gazebo/common/CommonIface.hh>
#include <gazebo/common/Plugin.hh>
// #include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/physics/World.hh>

#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <gazebo/common/Time.hh>

namespace gazebo
{
class parking_lot : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    printf("At WorldPlugin load\n");


      ignition::transport::Node node;


      ignition::msgs::ModelConfiguration link_1;
      ignition::msgs::Model model_1;
      model_1.set_name("AutoNOMOS_mini");
      model_1.set_id(0);
      model_1.set_pose(ignition::math::Pose3d(-1, 1, 0, 0, 0, 0))
      // Create the marker message
      // ignition::msgs::Marker markerMsg;
      // markerMsg.set_ns("default");
      // markerMsg.set_id(0);
      // markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
      // markerMsg.set_type(ignition::msgs::Marker::SPHERE);

      //ignition::msgs::Material *matMsg ;//= markerMsg.mutable_material();
      // ignition::msgs::Material *matMsg = markerMsg.mutable_material();
      // matMsg->mutable_script()->set_name("Gazebo/BlueLaser");

      // The rest of this function adds different shapes and/or modifies shapes.
      // Read the terminal statements to figure out what each node.Request
      // call accomplishes.

      // std::cout << "Spawning a sphere at the origin\n";
      // gazebo::common::Time::Sleep(4);
      // node.Request("/marker", model_1);

      // std::cout << "Moving the sphere to x=0, y=0, z=1\n";
      // gazebo::common::Time::Sleep(4);
      // ignition::msgs::Set(model_1.mutable_pose(),
                          // ignition::math::Pose3d(1, 1, 0, 0, 0, 0));
      // node.Request("/marker", model_1);

      // std::cout << "Shrinking the sphere\n";
      // // gazebo::common::Time::Sleep(4);
      // ignition::msgs::Set(model_1.mutable_scale(),
      //                   ignition::math::Vector3d(0.2, 0.2, 0.2));
      // node.Request("/marker", model_1);

      // std::cout << "Changing the sphere to red\n";
      // gazebo::common::Time::Sleep(4);
      // matMsg->mutable_script()->set_name("Gazebo/Red");
      // node.Request("/marker", markerMsg);

      std::cout << "Adding a green box\n";
      // gazebo::common::Time::Sleep(4);
      ignition::msgs::Marker markerMsg;
      markerMsg.set_ns("default");
      markerMsg.set_id(1);
      markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
      markerMsg.set_type(ignition::msgs::Marker::BOX);
      ignition::msgs::Material *matMsg = markerMsg.mutable_material();
      matMsg->mutable_script()->set_name("Gazebo/Green");
      ignition::msgs::Set(markerMsg.mutable_scale(),
                        ignition::math::Vector3d(1.0, 1.0, 1.0));
      ignition::msgs::Set(markerMsg.mutable_pose(),
                        ignition::math::Pose3d(2, 0, .5, 0, 0, 0));
      node.Request("/marker", markerMsg);

      std::cout << "Change the green box to a cylinder\n";
      // gazebo::common::Time::Sleep(4);
      markerMsg.set_type(ignition::msgs::Marker::CYLINDER);
      node.Request("/marker", markerMsg);

      std::cout << "Adding a line between the sphere and cylinder\n";
      // gazebo::common::Time::Sleep(4);
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
      // gazebo::common::Time::Sleep(4);
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
      // gazebo::common::Time::Sleep(4);
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

      std::cout << "Adding HELLO at 0, 0, 2\n";
      // gazebo::common::Time::Sleep(4);
      markerMsg.set_id(5);
      markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
      markerMsg.set_type(ignition::msgs::Marker::TEXT);
      markerMsg.set_text("HELLO");
      ignition::msgs::Set(markerMsg.mutable_scale(),
                        ignition::math::Vector3d(0.2, 0.2, 0.2));
      ignition::msgs::Set(markerMsg.mutable_pose(),
                        ignition::math::Pose3d(0, 0, 2, 0, 0, 0));
      node.Request("/marker", markerMsg);

      std::cout << "Adding a semi-circular triangle fan\n";
      // gazebo::common::Time::Sleep(4);
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
      // gazebo::common::Time::Sleep(4);
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
      // gazebo::common::Time::Sleep(4);
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

      // std::cout << "Delete all the markers\n";
      // gazebo::common::Time::Sleep(4);
      // markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
      // node.Request("/marker", markerMsg);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(parking_lot)
}
