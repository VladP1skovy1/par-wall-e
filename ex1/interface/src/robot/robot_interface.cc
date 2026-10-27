/*
 * File name: robot_interface.cc
 * Date:      2018/09/04 14:56
 * Author:    Jan Chudoba
 */

#include "robot/robot_interface.h"

bool operator<(const SRobotDeviceId& l, const SRobotDeviceId& r )
{
   return ( l.dataType < r.dataType ) ||
          (( l.dataType == r.dataType) && ( l.index < r.index));
}


CRobot::CRobot() :
	dataCallback(NULL)
{
}

CRobot::~CRobot()
{
	for (std::vector<CLaserSensorInterface*>::iterator it = laserSensors.begin(); it != laserSensors.end(); it++) {
		delete *it;
	}
	laserSensors.clear();
}

bool CRobot::waitForData(TRobotDataType dataType, int index, int timeout_ms)
{
	CEvent & event = getDeviceEvent(dataType, index);
	bool result;
	event.lock();
	result = isFresh(dataType, index);
	if (!result) {
		if (timeout_ms > 0) {
			event.waitNoLock(timeout_ms);
		} else {
			event.waitNoLock();
		}
		result = isFresh(dataType, index);
	}
	event.unlock();
	return result;
}

bool CRobot::registerLaserSensor(CLaserSensorInterface * laser)
{
	int laserIndex = (int) laserSensors.size();
	laserSensors.push_back(laser);
	laser->setDataCallback(laser_data_callback, this, laserIndex);
	return true;
}

void CRobot::callDataCallback(TRobotDataType dataType, int index)
{
	if (dataCallback) {
		dataCallback(dataType, index, dataCallbackContext);
	}
}

bool CRobot::internalIsFresh(TRobotDataType dataType, int index)
{
	if (dataType == ROBOT_DATA_LASER) {
		return isLaserSensorFresh(index);
	}
	bool result = deviceFresh[SRobotDeviceId(dataType, index)];
	return result;
}

void CRobot::setFresh(TRobotDataType dataType, int index)
{
	deviceFresh[SRobotDeviceId(dataType, index)] = true;
	if (!deviceEventExist(dataType, index)) return;
	CEvent & event = getDeviceEvent(dataType, index);
	event.notify();
}

void CRobot::unFresh(TRobotDataType dataType, int index)
{
	deviceFresh[SRobotDeviceId(dataType, index)] = false;
}

void CRobot::laserDataCallback(int index, CLaserScan & scan)
{
	//setFresh(ROBOT_DATA_LASER, index);
	callDataCallback(ROBOT_DATA_LASER, index);
}

/* end of robot_interface.cc */
