/*
 * File name: main.cc
 * Date:      2018/10/02 23:52
 * Author:    Miroslav Kulich
 */
#define _USE_MATH_DEFINES

#include <algorithm>
#include <numbers>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <climits>
#include <iostream>
#include <unistd.h>
#include "robot_client/robot_client.h"


enum Sectors {
    RIGHT,
    FRONT,
    LEFT,
};

enum class Behaviours {
    AVOID_OBSTACLES,
    FOLLOW,
};


constexpr Behaviours kBehaviour = Behaviours::FOLLOW;
constexpr bool kIsSimulation = true;
constexpr int kIgnoredLaserCount = 150;

constexpr double kDroneWidth = 0.4;

constexpr double kDroneMaxSpeed = 2.0;
constexpr double kDroneMaxTurn = 2.0;

constexpr double kMaxFollowingAngle = 60.0;
constexpr double kFullAngularThreshold = 45.0;
constexpr double kStopAngularThreshold = 3.0;

constexpr double kFullSpeedThreshold = 2.5;
constexpr double kStopThreshold = 0.4;
constexpr double kInactiveThreshold = 0.05;


constexpr double kMaxTargetDistance = 5.0;
constexpr double kDesiredDistanceToTarget = 1.0;


CRobotClient robot;
CLaserSensorConfiguration laserConfiguration;

static void avoidObstacles(CLaserSensorConfiguration laserConfiguration, CLaserScan &scan);

static void followTarget(CLaserSensorConfiguration laserConfiguration, CLaserScan &scan);

static Sectors getSector(CLaserSensorConfiguration laserConfiguration, int rayIndex, double distance);

static double getAngle(CLaserSensorConfiguration laserConfiguration, int rayIndex);

static double calculateFollowingVelocity(double distanceToTarget);

double calculateAngularVelocity(double angle);


template<typename T>
constexpr T clamp(T value, T lo, T hi);

template<typename T>
constexpr int sign(T x);

double constexpr deg2rad(double deg);

double constexpr rad2deg(double rad);

int main(int argc, char **argv) {
    CRobotClientConfiguration cfg;
    cfg.waitForInitialization = true;
    if (kIsSimulation) {
        robot.enableRequestResponseServices();
    } else {
        robot.setAsyncMode();
    }
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
            continue;
        }

        switch (kBehaviour) {
            case Behaviours::AVOID_OBSTACLES:
                avoidObstacles(laserConfiguration, scan);
                break;
            case Behaviours::FOLLOW:
                followTarget(laserConfiguration, scan);
                break;
            default:
                avoidObstacles(laserConfiguration, scan);
                break;
        }
    }

    return 0;
}

void avoidObstacles(CLaserSensorConfiguration laserConfiguration, CLaserScan &scan) {
    double minRightDist = INFINITY;
    double minFrontDist = INFINITY;
    double minLeftDist = INFINITY;

    for (int i = kIgnoredLaserCount; i < scan.range.size() - kIgnoredLaserCount; i++) {
        double rayDistance = scan.range[i];
        if (rayDistance <= kInactiveThreshold) continue;

        Sectors sector = getSector(laserConfiguration, i, rayDistance);

        if (sector == LEFT && rayDistance < minLeftDist) {
            minLeftDist = rayDistance;
        } else if (sector == FRONT && rayDistance < minFrontDist) {
            minFrontDist = rayDistance;
        } else if (sector == RIGHT && rayDistance < minRightDist) {
            minRightDist = rayDistance;
        }
    }

    double speedMultiplier = clamp((minFrontDist - kStopThreshold) / (kFullSpeedThreshold - kStopThreshold), 0.0,
                                   1.0);
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

double calculateFollowingVelocity(double distanceToTarget) {
    double distanceDifference = distanceToTarget - kDesiredDistanceToTarget;

    if (distanceDifference > 0.0) { // Moving forward
        double speedMultiplier = clamp(
            (distanceDifference - kStopThreshold) / (kFullSpeedThreshold - kStopThreshold), 0.0, 1.0);
        return kDroneMaxSpeed * speedMultiplier;
    } else { // Moving backward
        double absDiff = std::abs(distanceDifference);
        double maxBackwardError = kDesiredDistanceToTarget;
        double denominator = maxBackwardError - kStopThreshold;
        if (denominator <= 0.0) return 0.0;

        double speedMultiplier = clamp((absDiff - kStopThreshold) / denominator, 0.0, 1.0);
        return -kDroneMaxSpeed * speedMultiplier;
    }
}

double calculateAngularVelocity(double angle) {
    double angularMultiplier = clamp(
        ((std::abs(angle) - deg2rad(kStopAngularThreshold)) / (deg2rad(kFullAngularThreshold) - deg2rad(kStopAngularThreshold))), 0.0, 1.0);
    return kDroneMaxTurn * angularMultiplier * sign(angle);
}

void followTarget(CLaserSensorConfiguration laserConfiguration, CLaserScan &scan) {
    double minDistance = std::numeric_limits<double>::max();
    int targetDirectionIndex = -1;

    for (int i = kIgnoredLaserCount; i < scan.range.size() - kIgnoredLaserCount; i++) {
        double rayDistance = scan.range[i];
        if (rayDistance <= kInactiveThreshold || rayDistance >= kMaxTargetDistance || rayDistance >= minDistance)
            continue;
        minDistance = rayDistance;
        targetDirectionIndex = i;
    }

    if (targetDirectionIndex == -1) {
        robot.setVelocities(0.0, 0.0);
        return;
    }

    double angle = getAngle(laserConfiguration, targetDirectionIndex);
    double forwardVelocity = (std::abs(angle) > deg2rad(kMaxFollowingAngle)) ? 0.0 : calculateFollowingVelocity(minDistance);
    double angularVelocity = calculateAngularVelocity(angle);
    robot.setVelocities(forwardVelocity, angularVelocity);
}

double getAngle(CLaserSensorConfiguration laserConfiguration, int rayIndex) {
    return laserConfiguration.minAngle + rayIndex * laserConfiguration.angularResolution;
}

Sectors getSector(CLaserSensorConfiguration laserConfiguration, int rayIndex, double distance) {
    double angle = getAngle(laserConfiguration, rayIndex);

    double x = distance * std::cos(angle);
    double y = distance * std::sin(angle);

    if (x > 0.0 && std::fabs(y) < kDroneWidth) return FRONT;
    return (angle > 0.0) ? LEFT : RIGHT;
}


template<typename T>
constexpr T clamp(T value, T lo, T hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}

template<typename T>
constexpr int sign(T x) {
    return (T(0) < x) - (x < T(0));
}

double constexpr deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

double constexpr rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}