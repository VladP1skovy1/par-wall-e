/*
 * File name: dummy_robot.cc
 * Date:      2018/09/17 15:32
 * Author:    Jan Chudoba
 */

#include <stdio.h>
#include <unistd.h>

#include "dummy_robot/dummy_robot.h"

CDummyRobot::CDummyRobot()
{
}

CDummyRobot::~CDummyRobot()
{
	threadInterrupt = true;
	pthread_join(thread, 0);
}

bool CDummyRobot::initialize(CRobotConfiguration * config)
{
	threadInterrupt = false;
	pthread_create(&thread, 0, thread_body, this);
	return true;
}

bool CDummyRobot::setVelocities(double forwardVelocity, double angularVelocity)
{
	fprintf(stderr, "CDummyRobot::setVelocities(%.3f, %.3f)\n", forwardVelocity, angularVelocity);
	return true;
}

bool CDummyRobot::getOdometry(CPose & odoPose)
{
	fprintf(stderr, "CDummyRobot::getOdometry()\n");
	odoPose.x = 0.1;
	odoPose.y = 0.2;
	odoPose.heading = 3.14;
	return true;
}

double CDummyRobot::getBatteryLevel()
{
	return 0.666;
}

uint8_t CDummyRobot::getFrontBumperState()
{
	return 0;
}

bool CDummyRobot::getChargerState()
{
	return false;
}

bool CDummyRobot::getDockingSensorData(std::vector<uint8_t> & dockingData)
{
	return false;
}

bool CDummyRobot::getMotorCurrents(std::vector<double> & currents)
{
	currents.clear();
	currents.push_back(1e-3);
	currents.push_back(1e-3);
	return true;
}

void CDummyRobot::threadBody()
{
	fprintf(stderr, "dummy robot thread started\n");
	while (!threadInterrupt) {
		usleep(500000);
		callDataCallback(ROBOT_DATA_ODOMETRY, 0);
	}
	fprintf(stderr, "dummy robot thread interrupted\n");
}

int CDummyRobot::robotProvidesDataInternal(TRobotDataType dataType)
{
	switch (dataType) {
		case ROBOT_DATA_NONE: return 0;
		case ROBOT_DATA_ODOMETRY: return 1;
		case ROBOT_DATA_BATTERY: return 1;
		case ROBOT_DATA_BUMPER: return 0;
		case ROBOT_DATA_CHARGER: return 0;
		case ROBOT_DATA_DOCKING_SENSOR: return 0;
		case ROBOT_DATA_LASER: return getLaserSensorCount();
		case ROBOT_DATA_LASER_CFG: return getLaserSensorCount();
		case ROBOT_DATA_MOTOR_CURRENTS: return 1;
		case ROBOT_DATA_CLIFF: return 1;
		case ROBOT_DATA_NUMBER: return 0;
	}
	return 0;
}



/* end of dummy_robot.cc */
