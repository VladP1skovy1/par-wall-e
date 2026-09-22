/*
 * File name: main.cc
 * Date:      2018/10/02 23:52
 * Author:    Miroslav Kulich
 */

#include <stdio.h>

#include "robot_client/robot_client.h"

CRobotClient robot;
CLaserSensorConfiguration lacfg;


int main(int argc, char **argv) {
  CRobotClientConfiguration cfg;
  cfg.waitForInitialization = true;
  robot.enableRequestResponseServices();
  if (!robot.initialize(&cfg)) {
    std::cerr << "Failed to initialize robot!" << std::endl;
    return 1;
  }
  std::cerr << "Robot initialized." << std::endl;

  bool res = robot.getLaserSensorConfiguration(lacfg);

  std::cout << "Res " << res << std::endl;
  std::cout << "Range " << lacfg.maxRange << std::endl;
  std::cout << "Angle resolution " << lacfg.angularResolution << std::endl;
  std::cout << "Min angle " << lacfg.minAngle << std::endl;
  std::cout << "Count " << lacfg.scanSampleCount << std::endl;
  std::cout << "Frequency " << lacfg.frequency << std::endl;

  CPose pose;
  CLaserScan scan;

  while (1) {
    bool fresh = false;
    if (robot.isFresh(ROBOT_DATA_ODOMETRY)) {
      if (robot.getOdometry(pose)) {
        std::cout <<  "odometry: [" << pose.x <<", " << pose.y << ", " << pose.heading << "]" << std::endl;
      }
      fresh = true;
    }
    if (robot.isFresh(ROBOT_DATA_LASER)) {
      if (robot.getLaserSensorData(scan)) {
	std::cout << "scan " << scan.range.size() << std::endl;
      }
      fresh = true;
    }
    if (!fresh) {
      usleep(100000);
    }
  }

  return 0;
}
