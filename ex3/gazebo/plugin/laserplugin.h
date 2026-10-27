/*
 * File name: laserplugin.h
 * Date:      Fri Sep 21 2018 22:17:39 GMT+0200 (CEST) 
 * Author:    Miroslav Kulich (kulich@cvut.cz)
 */


#ifndef __LASERPLUGIN__
#define __LASERPLUGIN__

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "msgs/robot_messages.pb.h"

namespace imr {

using namespace gazebo;

class CLaserPlugin : public SensorPlugin {
    public:
        CLaserPlugin();
        virtual ~CLaserPlugin();
        void OnNewLaserScans();
        void OnUpdate();
        void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf);

    protected: 
        physics::WorldPtr world;

    private: 
        sensors::RaySensorPtr sensor;
        gazebo::event::ConnectionPtr connection;
        ignition::transport::Node node;
        ignition::transport::Node::Publisher publisher;
        std::string topicPrefix;
        // physics::MultiRayShapePtr  sensor;
        bool settingsService(const robot_messages::ConfigurationRequest &request, robot_messages::LaserCfgMsg &out);
};

} //end namespace imr

#endif

/* end of laserplugin.h */
