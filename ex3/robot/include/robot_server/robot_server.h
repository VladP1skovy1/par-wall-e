/*
 * File name: robot_server.h
 * Date:      2018/09/14 15:16
 * Author:    Jan Chudoba
 */

#ifndef __ROBOT_SERVER_H__
#define __ROBOT_SERVER_H__

#include <stdint.h>

#include <map>

#include <ignition/transport.hh>

#include "robot/robot_interface.h"
#include "robot_messages/robot_messages.pb.h"

class CRobotServer
{
public:
	CRobotServer(CRobot * r, const char * name);
	~CRobotServer();

protected:
	static void robot_data_callback(TRobotDataType dataType, int index, void * context) { ((CRobotServer*)context)->robotDataCallback(dataType, index); }
	void robotDataCallback(TRobotDataType dataType, int index);
	ignition::transport::Node::Publisher * getPublisher(TRobotDataType dataType);

	void publishRobotOdometry(ignition::transport::Node::Publisher * publisher, int index = 0);
	void publishRobotRawOdometry(ignition::transport::Node::Publisher * publisher, int index = 0);
	void publishRobotBattery(ignition::transport::Node::Publisher * publisher, int index = 0);
	void publishRobotBumper(ignition::transport::Node::Publisher * publisher, int index = 0);
	void publishRobotCharger(ignition::transport::Node::Publisher * publisher, int index = 0);
	void publishRobotDockingSensor(ignition::transport::Node::Publisher * publisher, int index = 0);
	void publishRobotLaser(ignition::transport::Node::Publisher * publisher, int index = 0);
	bool publishRobotLaserCfg(ignition::transport::Node::Publisher * publisher, int index = 0);
	void publishRobotMotorCurrents(ignition::transport::Node::Publisher * publisher, int index = 0);
	void publishRobotCliff(ignition::transport::Node::Publisher * publisher, int index = 0);

	bool prepareLaserSensorConfigurationMessage(robot_messages::LaserCfgMsg & cfgMsg, int index);
	void prepareRobotCapabilitiesResponse(robot_messages::RobotCapabilities & capMsg);

	void configurationRequestServer(const robot_messages::ConfigurationRequest & msg); // result is returned by topic
	void cbClientRequests(const robot_messages::ConfigurationRequest & msg, const ignition::transport::MessageInfo & info);
	void cbControlVelocities(const robot_messages::MotionControlMsg & msg, const ignition::transport::MessageInfo & info);
	void velocitiesServer(const robot_messages::MotionControlMsg & msg);
	bool capabilitiesRequestServer(robot_messages::RobotCapabilities & msg); // service with response (for Mirek)
	bool laserConfigurationRequestServer(const robot_messages::ConfigurationRequest & request, robot_messages::LaserCfgMsg & respones); // service with response (for Mirek)

	void checkBatteryStatus(int index);

private:
	static void * thread_body(void* arg) { ((CRobotServer*) arg)->threadBody(); return 0; }
	void threadBody();

	CRobot * robot;
	std::string robotName;
	ignition::transport::Node node;
	std::map<TRobotDataType, ignition::transport::Node::Publisher> publishers;

	pthread_t thread;
	bool threadInterrupt;
	bool threadStarted;

	bool verbose;

	std::map<TRobotDataType, int> robotDataUpdated;
	uint32_t lastControlCommandTime;
	bool lastControlCommandTimeValid;
	uint32_t lastBatteryReportTime;
};


#endif

/* end of robot_server.h */
