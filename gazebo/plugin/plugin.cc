/*
 * File name: plugin.cc
 * Date:      Wed Sep 19 2018 14:24:33 GMT+0200 (CEST) 
 * Author:    Miroslav Kulich
 */

#include "plugin.h"
#include <ignition/math/Vector2.hh>

using namespace imr;

const int NUM_POSES_PER_SECOND=10;

bool TurtleBotPlugin::capabilitiesService(robot_messages::RobotCapabilities &out) {
  gzmsg << "Robot capabilities service called." << std::endl;
	out.clear_capabilities();
	robot_messages::RobotCapabilities_Capability * cap = out.add_capabilities();
	cap->set_datatype(robot_messages::RobotDataType::ROBOT_DATA_LASER);
	cap->set_count(1);
	cap->set_label("");

  return true;
}
void TurtleBotPlugin::controlVelocitiesService(const robot_messages::MotionControlMsg &msg) {
/*    
  static const double l = 0.23;// distance between wheels taken from kobuki.sdf 
  static const double KV = 75;
  static const double KW = 2000;
  double v = KV*msg.forwardvelocity();
  double w = KW*msg.angularvelocity();
  double v_r = (v + (w*l)/2); 
  double v_l = (v - (w*l)/2); 

gazebo::physics::JointPtr leftWheel = model->GetJoint("kobuki_standalone::wheel_left_joint");
// std::cout << "Joint: " <<  model->GetJoints()[0]->GetName() << std::endl;
// std::cout << "Joint: " <<  leftWheel->GetName() << std::endl;

gazebo::physics::JointPtr rightWheel = model->GetJoint("kobuki_standalone::wheel_right_joint");
leftWheel->SetVelocity(0, v_l);
rightWheel->SetVelocity(0, v_r);

  gzmsg << "Control velocities service called: fwd: " << msg.forwardvelocity() << " ang: " <<  msg.angularvelocity() << "  left: " << v_l << " right: " << v_r << std::endl;
*/
    gazebo::physics::ModelState state = gazebo::physics::ModelState(model);
    ignition::math::Pose3d position = state.Pose();
	double dx = msg.forwardvelocity()*cos(msg.angularvelocity()+position.Rot().Yaw());
	double dy = msg.forwardvelocity()*sin(msg.angularvelocity()+position.Rot().Yaw());
    model->SetLinearVel(ignition::math::Vector3d(dx, dy, 0));
    model->SetAngularVel(ignition::math::Vector3d(0, 0, msg.angularvelocity()));


  // return true;
}


void TurtleBotPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) {
  std::string topicPrefix = "/turtlebot";
  index = 0;
  model = parent;
  updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&TurtleBotPlugin::OnUpdate, this));

  ignition::transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(NUM_POSES_PER_SECOND);
  publisher = node.Advertise<robot_messages::OdometryMsg>(topicPrefix + "/odometry");
  if (!publisher)
  {
    std::cerr << "Error advertising topic [" << topicPrefix + "/odometry" << "]" << std::endl;
  }

  if (!node.Subscribe(topicPrefix + "/control_velocities", &TurtleBotPlugin::OnVelocityMessage, this)) {
    gzthrow("Error advertising topic [" << topicPrefix + "/control_velocities" << "]");
  }


  if (!node.Advertise(topicPrefix + "/capabilities", &TurtleBotPlugin::capabilitiesService,this)) {
    gzthrow("Error advertising service [" << topicPrefix + "/capabilities" << "]");
  }

  if (!node.Advertise<TurtleBotPlugin,robot_messages::MotionControlMsg>(topicPrefix + "/control_velocities", &TurtleBotPlugin::controlVelocitiesService,this)) {
    gzthrow("Error advertising service [" << topicPrefix + "/control_velocities" << "]");
  }


  gzmsg << ("TurtleBot plugin loaded.") << std::endl;

}

/*
void TurtleBotPlugin::OnVelocityMessage(const robot_messages::MotionControlMsg &msg) {
  std::cout << "Velocity received: " <<  msg.forwardvelocity() << "  angular: "<< msg.angularvelocity() << std::endl;
  model->SetLinearVel(ignition::math::Vector3d(msg.forwardvelocity(), msg.angularvelocity(), 0));
}*/


void TurtleBotPlugin::OnVelocityMessage(const robot_messages::MotionControlMsg &msg) {
    std::cout << "Velocity received: " << msg.forwardvelocity() << "  angular: " << msg.angularvelocity() << std::endl;
    gazebo::physics::ModelState state = gazebo::physics::ModelState(model);
    ignition::math::Pose3d position = state.Pose();
	double dx = msg.forwardvelocity()*cos(msg.angularvelocity()+position.Rot().Yaw());
	double dy = msg.forwardvelocity()*sin(msg.angularvelocity()+position.Rot().Yaw());
    model->SetLinearVel(ignition::math::Vector3d(dx, dy, 0));
    model->SetAngularVel(ignition::math::Vector3d(0, 0, msg.angularvelocity()));
}

void TurtleBotPlugin::OnUpdate() {
  static int i = 0;
  if (++i == 100) {
	  gazebo::physics::ModelState state = gazebo::physics::ModelState(model);
	  ignition::math::Pose3d position = state.Pose();
	  auto pos = position.Pos();
	  auto rot = position.Rot();	

  robot_messages::OdometryMsg msg;
  msg.set_index(index);;
  msg.set_x(pos.X());
  msg.set_y(pos.Y());
  msg.set_heading(rot.Yaw());
//   std::cout << "Publishing " << pos.X() << " " << pos.Y() << " " << rot.Yaw() << std::endl; 
  publisher.Publish(msg);

    i = 0;
  }
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TurtleBotPlugin)
