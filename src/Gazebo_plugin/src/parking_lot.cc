#include "parking_lot.hh"


using namespace gazebo;
using namespace physics;

// class parking_lot : public WorldPlugin
// {
GZ_REGISTER_WORLD_PLUGIN(parking_lot)

void parking_lot::spawn_model_callback(ConstWorldStatisticsPtr &_msg)
{
	models_spawned++;
	printf("At callback:\t%d\n", models_spawned);
	if (models_spawned == total_models)
	{
		world -> SetPaused(false);
		printf("Done spawning models");
	}
}

parking_lot::parking_lot()
{
   //srand (231192);

  this->node_1 = transport::NodePtr(new transport::Node());
  this->node_1->Init();
  this->factoryPub = this->node_1->Advertise<msgs::Factory>("~/factory");
  sub_info = node_1->Subscribe("~/model/info", &parking_lot::spawn_model_callback, this);
  // printf("Cons of parking_lot\n" );
  // Create the message
  sdf::addURIPath 	( "model://",  std::getenv("GAZEBO_MODEL_PATH")	);

  car_model = "AutoNOMOS_mini_static";
  parking_space_model = "parking_space";
  lamp_post = "lamp_post_autonomos";

  car_counter = 0;
  parking_space_counter = 0;
  lamp_post_counter = 0;
  total_models = 0;

  car_sdfptr = get_sdf_file(car_model);
  parking_space_sdfptr = get_sdf_file(parking_space_model);
  lamp_post_sdfptr = get_sdf_file(lamp_post);

}

parking_lot::~parking_lot()
{

}

void parking_lot::Reset()
{
	printf("At reset\n");
	models_spawned = 0;
	//	physics::WorldPtr world = physics::get_world("default");
//	world->Clear();
//	world->InsertModelFile("model://ground_plane");
	bool pauseState = world->IsPaused();
	world->SetPaused(true);

//	this->publishModelPoses.clear();

  // Remove all models
  #if GAZEBO_VERSION_MAJOR >= 8
    Model_V models = world -> Models();
  #else 
    Model_V models = world -> GetModels();
  #endif
  for (Model_V::iterator iter = models.begin();
       iter != models.end(); ++iter)
  {
  //  this->rootElement->RemoveChild((*iter)->GetId());
//  	std::cout << "Model: " << (*iter) -> GetName() << std::endl;
	if ( (*iter) -> GetName() != "ground_plane") 
	{

//	  	std::cout << "Removing: " << (*iter) -> GetName() << std::endl;
//		(*iter) -> GetParentModel() -> RemoveChild((*iter) -> GetId());
		(*iter) -> Fini();
		
	}
  } 
  total_models = 0;
	std::cout << "Done removing models." << std::endl;
	std::cout << "The simulation is: " << world -> IsPaused() << std::endl;
	matrix_parking(ignition::math::Vector3d(9, -6, 0), 3, 5);

//	world -> SetPaused(false);
	int other=0;
	printf("models_spawned:\t%d\n", models_spawned);
//	for(double dummy_1 = 0; dummy_1 < 10000000 ; dummy_1 += 0.0001)
//	{
//		other++;
//	}
	
	printf("models_spawned:\t%d\n", models_spawned);
	
	std::cout << "Done reseting world." << std::endl;
}

void parking_lot::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // printf("At WorldPlugin load\n");
	world = _parent;
  matrix_parking(ignition::math::Vector3d(9, -6, 0), 3, 5);

  printf("Done loading parkinglot plugin");
}

void parking_lot::matrix_parking(ignition::math::Vector3d col_init, int height, int length)
{
	
//	world -> SetPaused(true);
  for (size_t i = 0; i < height; i++)
  {
    row_parking(col_init + ignition::math::Vector3d(0, ROW_SEP * i, 0), length);
  }
//	world -> SetPaused(false);
}

void parking_lot::row_parking(ignition::math::Vector3d row_init, int length)
{
  for (size_t i = 0; i < length; i++)
  {
    double_parking(row_init - ignition::math::Vector3d(COL_SEP * i, 0, 0));
  }
}

void parking_lot::double_parking(ignition::math::Vector3d offset_xyz)
{
  multiple_parking_spaces( ignition::math::Vector3d(0, 0, 0) + offset_xyz,
    10, ignition::math::Quaterniond(0, 0, 0) );
  multiple_parking_spaces( ignition::math::Vector3d(.5, 0, 0) + offset_xyz,
    10, ignition::math::Quaterniond(0, 0, M_PI) );
  multiple_lamp_posts(ignition::math::Vector3d(.15, -.1, 0) + offset_xyz,
    10, ignition::math::Quaterniond(0, 0, M_PI/2), 0);
  multiple_lamp_posts(ignition::math::Vector3d(.35, .24, 0) + offset_xyz,
    10, ignition::math::Quaterniond(0, 0, -M_PI/2), 1);

}

sdf::SDFPtr parking_lot::get_sdf_file(std::string model_name )
{
  sdf::SDFPtr sdfElement(new sdf::SDF());
  sdf::init(sdfElement);
  std::string model("model://");
  model.append(model_name);
  std::string sdf_file =  sdf::findFile(model);
  sdf_file.append("/model.sdf");
  std::cout << "file: " << sdf_file << '\n';
  if (!sdf::readFile(sdf_file, sdfElement))
  {
    std::cerr << "sdf NOT loaded: " << sdf_file << '\n';
  }
  return sdfElement;
}

void parking_lot::change_sdf_model_name(sdf::SDFPtr &sdfElement, std::string name, int &counter )
{
  const sdf::ElementPtr rootElement = sdfElement->Root();
  if (!rootElement->HasElement("model"))
  {
    std::cerr << " is not a model SDF file!" << std::endl;
  }
  const sdf::ElementPtr modelElement = rootElement->GetElement("model");
  sdf::ParamPtr param_name = modelElement->GetAttribute("name");
  std::string new_name;
  new_name.assign(name);
  new_name.append("_");
  new_name.append(std::to_string(counter));

  param_name->SetFromString(new_name);
  // std::cout << "param_name is: " << param_name->GetAsString()  << '\n';
  counter++;
}

void parking_lot::spawn_car(ignition::math::Vector3d v_xyz, ignition::math::Quaterniond quat)
{
  msgs::Factory msg;
  change_sdf_model_name(car_sdfptr, car_model, car_counter );
  msg.set_sdf(car_sdfptr->ToString());
  msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(v_xyz, quat));
  this->factoryPub->Publish(msg, true);
  total_models++;
}

void parking_lot::spawn_lamp_post(ignition::math::Vector3d v_xyz, ignition::math::Quaterniond quat)
{
  msgs::Factory msg;
  change_sdf_model_name(lamp_post_sdfptr, lamp_post, lamp_post_counter );
  msg.set_sdf(lamp_post_sdfptr->ToString());
  msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(v_xyz, quat));
  this->factoryPub->Publish(msg, true);
  total_models++;
}

void parking_lot::multiple_parking_spaces(ignition::math::Vector3d center_xyz, int num_spaces, ignition::math::Quaterniond new_quat)
{
  for (int i = 0; i < num_spaces; i++) {
    ignition::math::Pose3d pose = ignition::math::Pose3d(
      ignition::math::Vector3d(0, (i - num_spaces / 2.0)*.3, 0) - center_xyz,
      new_quat);
    place_parking_space(pose);
    if (rand() % 10 < PROB_SPAWN_CAR) {
      spawn_car(ignition::math::Vector3d(0,  (i - num_spaces / 2.0)*.3, 0) - center_xyz,
        new_quat);
    }
  }
}

void parking_lot::multiple_lamp_posts(ignition::math::Vector3d center_xyz, int num_spaces, ignition::math::Quaterniond new_quat, int mod_lamp)
{
  for (int i = 0; i < num_spaces; i++) {
    if(i % 3 == mod_lamp)
    {
      spawn_lamp_post(ignition::math::Vector3d(0,  (i - num_spaces / 2.0)*.3, 0) - center_xyz,
        new_quat);
    }
  }
}

void parking_lot::place_parking_space(ignition::math::Pose3d pose)
{
  msgs::Factory msg;
  change_sdf_model_name(parking_space_sdfptr, parking_space_model, parking_space_counter );
  // msg.set_sdf_filename("model://parking_space");
  msg.set_sdf(parking_space_sdfptr->ToString());
  msgs::Set(msg.mutable_pose(), pose);
  this->factoryPub->Publish(msg, true);
  total_models++;
}
