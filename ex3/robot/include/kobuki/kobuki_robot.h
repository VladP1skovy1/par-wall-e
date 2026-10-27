 /*
 * File name: kobuki_robot.h
 * Date:      2018/09/04 15:07
 * Author:    Jan Chudoba
 */

#ifndef __KOBUKI_ROBOT_H__
#define __KOBUKI_ROBOT_H__

#include <string>

#include "robot/robot_interface.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#include "kobuki_driver/kobuki.hpp"
#pragma GCC diagnostic pop

class CKobukiRobotConfiguration : public CRobotConfiguration
{
public:
	std::string deviceName;
};

#define BUMPER_LEFT 0x04
#define BUMPER_CENTER 0x02
#define BUMPER_RIGHT 0x01

class CKobukiRobot : public CRobot
{
public:
	CKobukiRobot();
	virtual ~CKobukiRobot();

	virtual bool initialize(CRobotConfiguration * config = NULL);

	virtual bool setVelocities(double forwardVelocity, double angularVelocity);

	virtual bool getRawOdometry(double & left, double & right);
	virtual bool getOdometry(CPose & odoPose);

	virtual double getBatteryLevel();
	virtual uint8_t getFrontBumperState();
	virtual bool getChargerState();
	virtual bool getDockingSensorData(std::vector<uint8_t> & dockingData);
	virtual bool getMotorCurrents(std::vector<double> & currents);

protected:
	virtual int robotProvidesDataInternal(TRobotDataType dataType);

protected:
	void processStreamData();

private:

	kobuki::Kobuki kobuki;
	kobuki::Parameters parameters;
	ecl::Slot<> slot_stream_data;
	ecl::LegacyPose2D<double> pose;
	bool poseValid;
	uint16_t lastOdoLeft, lastOdoRight;
	bool lastOdoValid;
	double odoLeft, odoRight;
	bool odoValid;
};

#endif

/* end of kobuki_robot.h */
