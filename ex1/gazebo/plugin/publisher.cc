/*
 * File name: pubisher.cc
 * Date:      Wed Sep 19 2018 15:41:48 GMT+0200 (CEST) 
 * Author:    Miroslav Kulich
 */

#include <iostream>

#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

using namespace gazebo;

int main(int _argc, char *_argv[])
{
  msgs::PoseAnimation msg;

  msg.set_model_name("box");
  msgs::Pose *p = msg.add_pose();
  msgs::Set(p, ignition::math::Pose3d(5, 5, 0, 0, 0, 0));
  p = msg.add_pose();
  msgs::Set(p, ignition::math::Pose3d(5, -5, 0, 0, 0, 0));
  p = msg.add_pose();
  msgs::Set(p, ignition::math::Pose3d(0, 0, 0, 0, 0, 0));

  transport::init();
  transport::run();
  transport::NodePtr node(new gazebo::transport::Node());
  node->Init("default");

  // modelmove_world is the name of the testing world
  const std::string topicName = "/gazebo/modelmove_world/" + msg.model_name()
    + "/model_move";
  gazebo::transport::PublisherPtr pathPub =
    node->Advertise<msgs::PoseAnimation>(topicName);

  std::cout << "Waiting for connection in " << topicName << std::endl;
  pathPub->WaitForConnection();
  pathPub->Publish(msg);

  std::cout << "Path published!" << std::endl;

  gazebo::transport::fini();
  return 0;
}
