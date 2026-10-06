/*
 * File name: laserplugin.cc
 * Date:      Fri Sep 21 2018 22:17:55 GMT+0200 (CEST) 
 * Author:    Miroslav Kulich (kulich@cvut.cz)
 */


#include "laserplugin.h"
#include "gazebo/physics/physics.hh"

using namespace imr;

GZ_REGISTER_SENSOR_PLUGIN(CLaserPlugin)

const int NUM_SCANS_PER_SECOND = 10;


//CLaserPlugin constructor 
CLaserPlugin::CLaserPlugin() {
  
};


CLaserPlugin::~CLaserPlugin() {
  sensor.reset();
  world.reset();
}


bool CLaserPlugin::settingsService(const robot_messages::ConfigurationRequest &request, robot_messages::LaserCfgMsg &out) {
  gzmsg << "Laser configuration servive called." << std::endl;
  out.set_index(0);//TODO: magic constant
  out.set_maxrange(sensor->RangeMax());
  out.set_angularresolution(sensor->AngleResolution());
  out.set_minangle(sensor->AngleMin().Radian());
  out.set_scansamplecount(sensor->RayCount());
  out.set_frequency(NUM_SCANS_PER_SECOND);
  out.mutable_position()->set_x(0);
  out.mutable_position()->set_y(0);
	out.mutable_position()->set_z(0);

  return true;
}


void CLaserPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr /*_sdf*/) {
  topicPrefix = "/turtlebot";
  sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!sensor)
    gzthrow("RayPlugin requires a Ray Sensor as its parent");

   world = physics::get_world(sensor->WorldName());
  connection = sensor->LaserShape()->ConnectNewLaserScans(std::bind(&CLaserPlugin::OnNewLaserScans, this));

  ignition::transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(NUM_SCANS_PER_SECOND);

  publisher = node.Advertise<robot_messages::LaserScanMsg>(topicPrefix + "/laser",opts);
  if (!publisher)
  {
    gzthrow("Error advertising topic [" << topicPrefix +  "/laser" << "]");
  }

  if (!node.Advertise(topicPrefix + "/laser_configuration_request", &CLaserPlugin::settingsService,this))
  {
    gzthrow("Error advertising service [" << topicPrefix + "/laser_configuration_request" << "]");
  }
  
 // gzmsg << sensor->AngleMin() << " " << sensor->AngleMax() << " " << sensor->RayCount() << "    " << sensor->IsActive() << " " << sensor->RangeMax() <<  std::endl;
  gzmsg << ("LaserPlugin loaded.") << std::endl;
}


void CLaserPlugin::OnNewLaserScans() {
  double max = sensor->RangeMax();
  std::vector<double>	ranges;
  sensor->Ranges(ranges);
  robot_messages::LaserScanMsg msg;
  msg.set_index(0); //TODO:magic constant
  msg.clear_range();
  for(auto r: ranges) {
     msg.add_range(r<max ? r : max);
  }
  publisher.Publish(msg);
}

/* end of laserplugin.cc */
