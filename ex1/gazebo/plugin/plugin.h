/*
 * File name: plugin.h
 * Date:      Wed Sep 19 2018 15:08:45 GMT+0200 (CEST) 
 * Author:    Miroslav Kulich
 */

#ifndef __PLUGIN__
#define __PLUGIN__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "msgs/robot_messages.pb.h"





namespace imr {

class TurtleBotPlugin : public gazebo::ModelPlugin {
    public: 
//        void Init();
        void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);
        void OnVelocityMessage(const robot_messages::MotionControlMsg &msg);
        void OnUpdate();

    private: 
        gazebo::physics::ModelPtr model;
        gazebo::event::ConnectionPtr updateConnection;
        gazebo::event::ConnectionPtr velocityConnection;
        ignition::transport::Node node;
        ignition::transport::Node::Publisher publisher;
        int index;
        bool capabilitiesService(robot_messages::RobotCapabilities &out);
        void controlVelocitiesService(const robot_messages::MotionControlMsg &msg);

  };
} //end namespace imr

#endif

/* end of plugin.h */
