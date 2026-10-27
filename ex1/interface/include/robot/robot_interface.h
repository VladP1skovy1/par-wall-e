/*
 * File name: robot_interface.h
 * Date:      2018/09/04 14:28
 * Author:    Jan Chudoba
 */

#ifndef __ROBOT_INTERFACE_H__
#define __ROBOT_INTERFACE_H__

#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <vector>
#include <map>

#include "robot/pose.h"
#include "robot/laser_sensor.h"
#include "robot/common.h"

class CRobotConfiguration
{
public:
	bool waitForInitialization;

	CRobotConfiguration() :
		waitForInitialization(false)
	{}
};

enum TRobotDataType
{
	ROBOT_DATA_NONE,
	ROBOT_DATA_ODOMETRY,
	ROBOT_DATA_BATTERY,
	ROBOT_DATA_BUMPER,
	ROBOT_DATA_CHARGER,
	ROBOT_DATA_DOCKING_SENSOR,
	ROBOT_DATA_LASER,
	ROBOT_DATA_LASER_CFG,
	ROBOT_DATA_MOTOR_CURRENTS,
	ROBOT_DATA_CLIFF,
	ROBOT_DATA_NUMBER // last
	// keep this in sync with robot_messages / RobotDataType
};

typedef void (*TRobotDataCallback)(TRobotDataType, int, void*);

struct SRobotDeviceId
{
public:
	TRobotDataType dataType;
	int index;
	SRobotDeviceId(TRobotDataType dt, int i) : dataType(dt), index(i) {}
};

bool operator<(const SRobotDeviceId& l, const SRobotDeviceId& r );

class CRobot
{
public:
	CRobot();
	virtual ~CRobot();

	virtual bool initialize(CRobotConfiguration * config = NULL) = 0;

	bool waitForData(TRobotDataType dataType, int index = 0, int timeout_ms = 0);
	virtual bool isFresh(TRobotDataType dataType, int index = 0) { return internalIsFresh(dataType, index); }
	void unFresh(TRobotDataType dataType, int index = 0);

	virtual bool setVelocities(double forwardVelocity, double angularVelocity) = 0;

	virtual bool getRawOdometry(double & left, double & right) { return false; }
	virtual bool getOdometry(CPose & odoPose) = 0;

	int getLaserSensorCount() { return (int) laserSensors.size(); }
	bool getLaserSensorConfiguration(CLaserSensorConfiguration & cfg, int sensorIndex = 0) { return (sensorIndex < getLaserSensorCount()) ? laserSensors[sensorIndex]->getConfiguration(cfg) : false; }
	bool isLaserSensorFresh(int sensorIndex = 0) { return laserSensors[sensorIndex]->isFresh(); }
	bool getLaserSensorData(CLaserScan & scan, int sensorIndex = 0) { return (sensorIndex < getLaserSensorCount()) ? laserSensors[sensorIndex]->getLastScan(scan) : false; }

	virtual double getBatteryLevel() { return NAN; }

	virtual uint8_t getFrontBumperState() { return false; }

	virtual bool getChargerState() { return false; }

	virtual bool getDockingSensorData(std::vector<uint8_t> & dockingData) { return false; } // docking sensor provides binary data for left/mid/right sensor

	virtual bool getMotorCurrents(std::vector<double> & currents) { return false; }

	// TODO: cliff sensor interface
	// TODO: gyro interface

	bool robotProvidesData(TRobotDataType dataType) { return (robotProvidesDataInternal(dataType) > 0); }
	int getRobotSensorCount(TRobotDataType dataType) { return robotProvidesDataInternal(dataType); }

	void setDataCallback(TRobotDataCallback cb, void * context) { dataCallback = cb; dataCallbackContext = context; }

	bool registerLaserSensor(CLaserSensorInterface * laser);

protected:
	// robotProvidesDataInternal: return value is number of devices
	virtual int robotProvidesDataInternal(TRobotDataType dataType) { return 0; }

	void callDataCallback(TRobotDataType dataType, int index);

	bool deviceEventExist(TRobotDataType dataType, int index) { return deviceEvents.find(SRobotDeviceId(dataType, index)) != deviceEvents.end(); }
	CEvent & getDeviceEvent(TRobotDataType dataType, int index) { return deviceEvents[SRobotDeviceId(dataType, index)]; }

	bool internalIsFresh(TRobotDataType dataType, int index);
	void setFresh(TRobotDataType dataType, int index);

	static void laser_data_callback(void * context, int index, CLaserScan & scan) { ((CRobot*)context)->laserDataCallback(index, scan); }
	void laserDataCallback(int index, CLaserScan & scan);

protected:
	std::vector<CLaserSensorInterface*> laserSensors;
	TRobotDataCallback dataCallback;
	void * dataCallbackContext;
	std::map<SRobotDeviceId, CEvent> deviceEvents;
	std::map<SRobotDeviceId, bool> deviceFresh;

};

#endif

/* end of robot_interface.h */
