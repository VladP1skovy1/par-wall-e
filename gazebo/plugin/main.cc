#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "msgs/messages.pb.h"

static std::atomic<bool> g_terminatePub(false);


void signal_handler(int _signal) {
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}


void poseUpdate(const PoseMsg &msg) {
  std::cout << "Position: [" << msg.x() << ", " << msg.y() << ", " << msg.phi() << "]" << std::endl << std::endl;
}

void laserUpdate(const LaserMsg &msg) {
  std::cout << "Msg: " << msg.ranges_size() << " rays received."<< std::endl;
}




int main(int argc, char **argv) {
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);
  ignition::transport::Node node;

// get laser settings
  LaserSettingsMsg settings;
  bool result;
  unsigned int timeout = 5000;
  bool executed = node.Request("/settings", timeout,settings, result);

  std:: cout << "Laser settings " << (executed ? "YES" : "NO") << "  " << (result ? "YES" : "NO") << std::endl;
  std:: cout << "Min angle: " << settings.angle_min() << std::endl;
  std:: cout << "Max angle: " << settings.angle_max() << std::endl;
  std:: cout << "Angle step: " << settings.angle_step() << std::endl;
  std:: cout << "Range min: " << settings.range_min() << std::endl;
  std:: cout << "Range max: " << settings.range_max() << std::endl;
  std:: cout << "Count: " <<  settings.count() << std::endl;



//
  std::string topic = "/velo";
  auto pub = node.Advertise<VelocityMsg>(topic);
  if (!pub)
  {
    std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
    return -1;
  }

  // Subscribe to a pose.
  if (!node.Subscribe("/odom", poseUpdate))
  {
    std::cerr << "Error subscribing to topic [" << "/pose" << "]" << std::endl;
    return -1;
  }


  // Subscribe to laser data.
  if (!node.Subscribe("/laser", laserUpdate))
  {
    std::cerr << "Error subscribing to topic [" << "/laser" << "]" << std::endl;
    return -1;
  }



  // Publish control commands.
  VelocityMsg msg;
  msg.set_forward(0.1);
  msg.set_angular(0.0);
  // Publish messages at 1Hz.
  while (!g_terminatePub)
  {
    if (!pub.Publish(msg))
      break;
    std::cout << "Publishing on topic [" << topic << "]" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return 0;
}