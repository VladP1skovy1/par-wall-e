/*
 * File name: main.cc
 * Date:      2018/10/02 23:52
 * Author:    Miroslav Kulich
 */
#define _USE_MATH_DEFINES

#include <stdio.h>
#include <vector>
#include <math.h>
#include "robot_client/robot_client.h"



enum Sectors{
    RIGHT,
    FRONT,
    LEFT
};


constexpr double kDroneWidth = 0.4;

constexpr double kDroneMaxSpeed = 2.0;
constexpr double kDroneMaxTurn = 2.0;

constexpr double kFullSpeedThreshold = 2.5;
constexpr double kStopThreshold = 0.4;
constexpr double kInactiveThreshold = 0.05;


CRobotClient robot;
CLaserSensorConfiguration laserConfiguration;

void avoidObstacles(CLaserSensorConfiguration laserConfiguration, CLaserScan& scan);
Sectors getSector(CLaserSensorConfiguration laserConfiguration, int rayIndex, double distance);

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
        } else {
            avoidObstacles(laserConfiguration, scan);
        }
    }

    return 0;
}


void avoidObstacles(CLaserSensorConfiguration laserConfiguration, CLaserScan& scan){  
    double minRightDist = INFINITY;
    double minFrontDist = INFINITY;
    double minLeftDist = INFINITY;

    for (int i = 0; i < scan.range.size(); i++){
        double rayDistance = scan.range[i];
        if (rayDistance <= kInactiveThreshold) continue;

        Sectors sector = getSector(laserConfiguration, i, rayDistance);

        if (sector == LEFT && rayDistance < minLeftDist){
            minLeftDist = rayDistance;
        }else if (sector == FRONT && rayDistance < minFrontDist){
            minFrontDist = rayDistance;
        }else if (sector == RIGHT && rayDistance < minRightDist){
            minRightDist = rayDistance;
        }
    }

    double speedMultiplier = std::clamp((minFrontDist - kStopThreshold) / (kFullSpeedThreshold - kStopThreshold), 0.0, 1.0);
    double speed = kDroneMaxSpeed * speedMultiplier;

    static int turnDirection = 0;
    double turn = 0.0;
    if (speedMultiplier < 1.0) {
        if (turnDirection == 0) turnDirection = (minLeftDist > minRightDist) ? +1 : -1;
        turn = turnDirection * kDroneMaxTurn * (1.0 - speedMultiplier);
    } else {
        turnDirection = 0;
    }

    robot.setVelocities(speed, turn);
}


Sectors getSector(CLaserSensorConfiguration laserConfiguration, int rayIndex, double distance){
    double angle = (laserConfiguration.minAngle + (rayIndex * laserConfiguration.angularResolution));

    double x = distance * std::cos(angle);
    double y = distance * std::sin(angle);

    if (x > 0.0 && std::fabs(y) < kDroneWidth) return FRONT;
    return (angle > 0.0) ? LEFT : RIGHT;
}
