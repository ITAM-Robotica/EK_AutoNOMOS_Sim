/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
 /*
 * Desc: ROS interface to a Position2d controller for an Ackermann drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */
 /*
 * Adapted as a ROS gazebo plugin for car-like vehicles using
 * Ackermann steering.
 *
 * Copyright (C) 2011, Nicu Stiurca, Jack O'Quin
 */
 /*
  * Adapted for Lego NXT
  * Copyright (C) 2011, Alexandr Buyval
  */

#include <algorithm>
#include <assert.h>

#include <ackermann_plugin.h>

#include "common/Events.hh"
#include "physics/physics.h"

using namespace gazebo;


enum
{
  RIGHT, LEFT, STEER
};

const double TAU = 6.28318530717958647693;  // 2 * pi

// Constructor
AckermannPlugin::AckermannPlugin()
{

}

// Destructor
AckermannPlugin::~AckermannPlugin()
{
  event::Events::DisconnectWorldUpdateStart(updateConnection);
  delete transform_broadcaster_;
  rosnode_->shutdown();
  callback_queue_thread_.join();
  delete rosnode_;


}

// Load the controller
void AckermannPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  
   ROS_INFO("AckermannPlugin::Load");
  
  world = _model->GetWorld();
  
    // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    robotNamespace.clear();
  else
    robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
  
   if (!_sdf->HasElement("topicName"))
    topicName = "cmd_vel";
  else
    topicName = _sdf->GetElement("topicName")->GetValueString();
  
  
  if (!_sdf->HasElement("wheelSeparationWidth"))
    wheelSepWidth = 0.16;
  else
    wheelSepWidth = _sdf->GetElement("wheelSeparationWidth")->GetValueDouble();

  if (!_sdf->HasElement("wheelSeparationLength"))
    wheelSepLength = 0.32;
  else
    wheelSepLength = _sdf->GetElement("wheelSeparationLength")->GetValueDouble();

  if (!_sdf->HasElement("wheelDiameter"))
    wheelDiam = 0.04;
  else
    wheelDiam = _sdf->GetElement("wheelDiameter")->GetValueDouble();
  
  if (!_sdf->HasElement("driveTorque"))
    driveTorque = 10.0;
  else
    driveTorque = _sdf->GetElement("driveTorque")->GetValueDouble();  

  if (!_sdf->HasElement("steerTorque"))
    steerTorque = 10.0;
  else
    steerTorque = _sdf->GetElement("steerTorque")->GetValueDouble();


  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    linkName = link->GetName();
  }
  else {
    linkName = _sdf->GetElement("bodyName")->GetValueString();
    link = _model->GetLink(linkName);
  }

  // assert that the body by linkName exists
  if (!link)
  {
    ROS_FATAL("AckermannPlugin error: bodyName: %s does not exist\n", linkName.c_str());
    return;
  }

  if (_sdf->HasElement("leftJoint"))  joints[LEFT]  = _model->GetJoint(_sdf->GetElement("leftJoint")->GetValueString());
  if (_sdf->HasElement("rightJoint"))  joints[RIGHT]  = _model->GetJoint(_sdf->GetElement("rightJoint")->GetValueString());
  if (_sdf->HasElement("steerJoint"))  joints[STEER]  = _model->GetJoint(_sdf->GetElement("steerJoint")->GetValueString());

  //ROS_INFO("Info --- Load parametres");
  
  if (!joints[LEFT])  ROS_FATAL("AckermannPlugin error: The controller couldn't get left joint");
  if (!joints[RIGHT]) ROS_FATAL("AckermannPlugin error: The controller couldn't get right joint");
  if (!joints[STEER]) ROS_FATAL("AckermannPlugin error: The controller couldn't get steer joint");
  

  enableMotors = true;

  wheelSpeed[RIGHT] = 0;
  wheelSpeed[LEFT] = 0;
  steerAngle = 0;

  prevUpdateTime = world->GetSimTime();


  // Initialize the ROS node and subscribe to car_drive


  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"ackermann_plugin",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);


  }

  rosnode_ = new ros::NodeHandle(robotNamespace);

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();


  // ROS: Subscribe to the CarDrive command topic (usually "car_drive")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<art_msgs::CarDriveStamped>(topicName, 1,
                                                               boost::bind(&AckermannPlugin::carDriveCallback, this, _1),
                                                               ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);
  
  callback_queue_thread_ = boost::thread(boost::bind(&AckermannPlugin::QueueThread, this));
  
  Reset();
  
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&AckermannPlugin::Update, this));

}

// Initialize the controller
void AckermannPlugin::Init()
{
  callback_queue_thread_ = boost::thread(boost::bind(&AckermannPlugin::QueueThread, this));
}


// Reset
void AckermannPlugin::Reset()
{
  
  prevUpdateTime = world->GetSimTime();
  alive_ = true;

}

// Update the controller
void AckermannPlugin::Update()
{
  
  double a;
  common::Time stepTime;

  GetPositionCmd();

  stepTime = world->GetSimTime() - prevUpdateTime;
  prevUpdateTime = world->GetSimTime();

  
  // Get steering angle
  a = joints[STEER]->GetAngle(0).GetAsRadian();
 
  //ROS_INFO("Current anlge:%f Command angle:%f", a, steerAngle);
  
  double ang_vel = ((steerAngle - a) / stepTime.Double())/10;
  
  if (ang_vel>10) ang_vel=10; 
  if (ang_vel<-10) ang_vel=-10; 
  
  //ROS_INFO("Angular speed:%f", ang_vel);
  
  if (enableMotors)
  {
    joints[LEFT]->SetVelocity(0, wheelSpeed[LEFT] / (wheelDiam / 2.0));

    joints[RIGHT]->SetVelocity(0, wheelSpeed[RIGHT] / (wheelDiam / 2.0));

    
    // FIXME: come up with something nicer for doing position control
    // than this simple proportional controller
    joints[STEER]->SetVelocity(0, ang_vel);

	
    joints[LEFT]->SetMaxForce(0, driveTorque);
    joints[RIGHT]->SetMaxForce(0,driveTorque);
    joints[STEER]->SetMaxForce(0,steerTorque);

  }

  publish_odometry();



}


// NEW: Now uses the target velocities from the ROS message, not the Iface 
void AckermannPlugin::GetPositionCmd()
{
  lock.lock();

  // Changed motors to be always on, which is probably what we want anyway
  enableMotors = true; 

  wheelSpeed[LEFT] = speed_;
  wheelSpeed[RIGHT] = speed_;
  steerAngle = angle_;

  lock.unlock();


}

// NEW: Store the velocities from the ROS message
void AckermannPlugin::carDriveCallback(const art_msgs::CarDriveStamped::ConstPtr& cmd_msg)
{


  lock.lock();

  speed_ = cmd_msg->control.speed;
  angle_ = cmd_msg->control.steering_angle;
  
  // FIXME: take acceleration and jerk into account

  lock.unlock();

}

// NEW: custom callback queue thread
void AckermannPlugin::QueueThread()
{

  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    //    std::cout << "CALLING STUFF\n";
    
    queue_.callAvailable(ros::WallDuration(timeout));
  }

}

// NEW: Update this to publish odometry topic
void AckermannPlugin::publish_odometry()
{	

  // get current time
  ros::Time current_time_((world->GetSimTime()).sec, (world->GetSimTime()).nsec);

  // getting data for base_footprint to odom transform
  math::Pose pose = link->GetWorldPose();
  math::Vector3 velocity = link->GetWorldLinearVel();
  math::Vector3 angular_velocity = link->GetWorldAngularVel();

  

  tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);
  tf::Transform base_footprint_to_odom(qt, vt);


  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                            current_time_,
                                                            "odom",
                                                            "base_footprint"));


  // publish odom topic
  odom_.pose.pose.position.x = pose.pos.x;
  odom_.pose.pose.position.y = pose.pos.y;

  odom_.pose.pose.orientation.x = pose.rot.x;
  odom_.pose.pose.orientation.y = pose.rot.y;
  odom_.pose.pose.orientation.z = pose.rot.z;
  odom_.pose.pose.orientation.w = pose.rot.w;

  odom_.twist.twist.linear.x = velocity.x;
  odom_.twist.twist.linear.y = velocity.y;
  odom_.twist.twist.angular.z = angular_velocity.z;

  odom_.header.frame_id = tf::resolve(tf_prefix_, "odom");
  odom_.child_frame_id = "base_footprint";
  odom_.header.stamp = current_time_;

  pub_.publish(odom_);

}

GZ_REGISTER_MODEL_PLUGIN(AckermannPlugin)
