/*
 * File name: main.cc
 * Date:      2018/10/02 23:52
 * Author:    Miroslav Kulich
 */

#include <stdio.h>
#include <vector>
#include <math.h>
#include "robot_client/robot_client.h"

#define _USE_MATH_DEFINES


enum Sectors{
  RIGHT,
  FRONT,
  LEFT
};


constexpr double kFullSpeedThreshold = 4.0;
constexpr double kStopTreshold = 0.5;
constexpr double kInactiveTreshold = 0.05;


CRobotClient robot;
CLaserSensorConfiguration laserConfiguration;

void performAction(CLaserSensorConfiguration laserConfiguration, CPose& pose, CLaserScan& scan);
Sectors getSector(CLaserSensorConfiguration laserConfiguration, int rayIndex);

int main(int argc, char **argv) {
  CRobotClientConfiguration cfg;
  cfg.waitForInitialization = true;
  robot.enableRequestResponseServices();
  if (!robot.initialize(&cfg)) {
    std::cerr << "Failed to initialize robot!" << std::endl;
    return 1;
  }
  std::cerr << "Robot initialized." << std::endl;

  bool res = robot.getLaserSensorConfiguration(laserConfiguration);

  std::cout << "Res " << res << std::endl;
  std::cout << "Range " << laserConfiguration.maxRange << std::endl;
  std::cout << "Angle resolution " << laserConfiguration.angularResolution << std::endl;
  std::cout << "Min angle " << laserConfiguration.minAngle << std::endl;
  std::cout << "Count " << laserConfiguration.scanSampleCount << std::endl;
  std::cout << "Frequency " << laserConfiguration.frequency << std::endl;

  CPose pose;
  CLaserScan scan;


  while (true) {
    bool fresh = false;
    if (robot.isFresh(ROBOT_DATA_ODOMETRY)) {
      if (robot.getOdometry(pose)) {
        // std::cout <<  "odometry: [" << pose.x <<", " << pose.y << ", " << pose.heading << "]" << std::endl;
      }
      fresh = true;
    }
    if (robot.isFresh(ROBOT_DATA_LASER)) {
      if (robot.getLaserSensorData(scan)) {
	      // std::cout << "scan " << scan.range.size() << std::endl;
      }
      fresh = true;
    }
    if (!fresh) {
      usleep(100000);
    }


    performAction(laserConfiguration, pose, scan);

  }

  return 0;
}


void performAction(CLaserSensorConfiguration laserConfiguration, CPose& pose, CLaserScan& scan){  
  double minRightDist = INFINITY;
  double minFrontDist = INFINITY;
  double minLeftDist = INFINITY;

    for (int i = 0; i < scan.range.size(); i++){
      getSector(laserConfiguration, i);
    }
}

Sectors getSector(CLaserSensorConfiguration laserConfiguration, int rayIndex){
    double angle = (laserConfiguration.minAngle + (rayIndex * laserConfiguration.angularResolution))* (180.0/M_PI);

    if (angle <= -45.0) {
      return RIGHT;
    } else if (angle > -45.0 && angle < 45.0) {
        return FRONT;
    } else {
        return LEFT;
    }
}
