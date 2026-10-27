/*
 * File name: robot_client.h
 * Date:      2018/09/18 09:09
 * Author:    Jan Chudoba
 */

#ifndef __ROBOT_CLIENT_H__
#define __ROBOT_CLIENT_H__

#include <string>
#include <map>

#include <ignition/transport.hh>

#include "robot/robot_interface.h"
#include "robot_messages/robot_messages.h"
#include "robot/common.h"

class CRobotClientConfiguration : public CRobotConfiguration
{
public:
	std::string robotName; // prefix for ignition-transport topic names
};

class CRobotClientLaser : public CLaserSensorInterface
{
public:
	void updateLaserConfiguration(const robot_messages::LaserCfgMsg & cfgMsg);
	void updateLaserScan(const robot_messages::LaserScanMsg & scanMsg);
};

class CRobotClient : public CRobot
{
public:
	CRobotClient();
	virtual ~CRobotClient();

	virtual bool initialize(CRobotConfiguration * config = NULL);

	virtual bool setVelocities(double forwardVelocity, double angularVelocity);

	virtual bool getOdometry(CPose & odoPose);

	virtual double getBatteryLevel();

	virtual uint8_t getFrontBumperState();

	virtual bool getChargerState();

	virtual bool getDockingSensorData(std::vector<uint8_t> & dockingData);

	virtual bool getMotorCurrents(std::vector<double> & currents);

	void setAsyncMode(bool async = true) { asyncMode = async; }
	void enableRequestResponseServices() { useRequestResponseServices = true; }

	// bool getLaserConfiguration(CLaserSensorConfiguration & cfg);

protected:
	virtual int robotProvidesDataInternal(TRobotDataType dataType);

protected:
	bool isLaserSensorPresent(int index) { return (laserSensorMap.find(index) != laserSensorMap.end()); }
	bool checkLaserSensorPresent(int index, bool registerNew = true);
	CRobotClientLaser & getLaserSensor(int index) { return *(laserSensorMap[index]); }

private:
	void cbCapabilities(const robot_messages::RobotCapabilities & msg, const ignition::transport::MessageInfo & info);
	void cbOdometry(const robot_messages::OdometryMsg & msg, const ignition::transport::MessageInfo & info);
	void cbBattery(const robot_messages::BatteryMsg & msg, const ignition::transport::MessageInfo & info);
	void cbBumper(const robot_messages::BumperMsg & msg, const ignition::transport::MessageInfo & info);
	void cbCharger(const robot_messages::ChargerMsg & msg, const ignition::transport::MessageInfo & info);
	void cbDockingSensor(const robot_messages::DockingSensorMsg & msg, const ignition::transport::MessageInfo & info);
	void cbLaserScan(const robot_messages::LaserScanMsg & msg, const ignition::transport::MessageInfo & info);
	void cbLaserConfiguration(const robot_messages::LaserCfgMsg & msg, const ignition::transport::MessageInfo & info);
	void cbMotorCurrents(const robot_messages::MotorCurrentMsg & msg, const ignition::transport::MessageInfo & info);
	void cbCliff(const robot_messages::CliffMsg & msg, const ignition::transport::MessageInfo & info);
	//void cbX(const robot_messages::X & msg, const ignition::transport::MessageInfo & info);

	void lockRequests() { requestsEvent.lock(); }
	void unlockRequests() { requestsEvent.unlock(); }
	void waitForRequests() { requestsEvent.waitNoLock(1000); }
	void notifyRequests() { requestsEvent.notifyNoLock(); }
	void deleteRequest(TRobotDataType dataType, int index);
	bool sendRequest(TRobotDataType dataType, int index);

	bool sendRobotCapabilitiesRequest();
	bool sendLaserCfgRequest(int index);

private:
	std::string robotName;
	std::string topicPrefix;
	ignition::transport::Node node;
	ignition::transport::Node::Publisher clientRequestPublisher;
	ignition::transport::Node::Publisher controlVelocitiesPublisher;

	CPose lastPose;
	bool lastPoseValid;
	bool lastPoseFresh;

	double batteryLevel;
	bool batteryLevelFresh;
	uint8_t frontBumperState;
	bool frontBumperStateFresh;
	bool chargerState;
	bool chargerStateFresh;
	std::vector<uint8_t> dockingSensorData;
	bool dockingSensorDataValid;
	bool dockingSensorDataFresh;
	std::vector<double> motorCurrents;
	bool motorCurrentsValid;
	bool motorCurrentsFresh;
	std::map<int, CRobotClientLaser*> laserSensorMap;
	std::map<std::pair<TRobotDataType, int>, int> requestedConfiguration;
	std::map<TRobotDataType, int> capabilities;
	CEvent requestsEvent;
	bool asyncMode;
	bool useRequestResponseServices;
	bool verbose;
};

#endif

/* end of robot_client.h */
