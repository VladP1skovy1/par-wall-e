/*
 * File name: robot_server.cc
 * Date:      2018/09/14 15:39
 * Author:    Jan Chudoba
 */

#include <stdio.h>
#include <math.h>

#include "robot_server/robot_server.h"
#include "robot_messages/robot_messages.h"
#include "robot/common.h"

#define SERVER_THREAD_PERIOD_MS 1000

#ifndef isnan
#define isnan(x) std::isnan(x)
#endif

#define CONTROL_TIMEOUT_MS 1000
#define BATTERY_REPORT_PERIOD 30000

CRobotServer::CRobotServer(CRobot * r, const char * name) :
	robot(r),
	robotName(name),
	threadStarted(false),
	verbose(false),
	lastControlCommandTimeValid(false)
{
	robot_messages::robot_messages_check();

	lastBatteryReportTime = CClock::getTimeMs();

	std::string topicPrefix = std::string("/");
	if (name) topicPrefix += name;
	else topicPrefix += "turtlebot";
	topicPrefix += "/";

	// enumerate robot capabilities and initialize publishers
	for (TRobotDataType dataType = ROBOT_DATA_NONE; dataType < ROBOT_DATA_NUMBER; dataType = (TRobotDataType) (dataType+1)) {
		robotDataUpdated[dataType] = -1;
		if (robot->robotProvidesData(dataType)) {
			std::string topicName;
			switch (dataType) {
			case ROBOT_DATA_NONE:
				break;
			case ROBOT_DATA_ODOMETRY:
				topicName = topicPrefix + "odometry";
				publishers[dataType] = node.Advertise<robot_messages::OdometryMsg>(topicName);
				break;
			case ROBOT_DATA_BATTERY:
				topicName = topicPrefix + "battery";
				publishers[dataType] = node.Advertise<robot_messages::BatteryMsg>(topicName);
				break;
			case ROBOT_DATA_BUMPER:
				topicName = topicPrefix + "bumper";
				publishers[dataType] = node.Advertise<robot_messages::BumperMsg>(topicName);
				break;
			case ROBOT_DATA_CHARGER:
				topicName = topicPrefix + "charger";
				publishers[dataType] = node.Advertise<robot_messages::ChargerMsg>(topicName);
				break;
			case ROBOT_DATA_DOCKING_SENSOR:
				topicName = topicPrefix + "docking_sensor";
				publishers[dataType] = node.Advertise<robot_messages::DockingSensorMsg>(topicName);
				break;
			case ROBOT_DATA_LASER:
				topicName = topicPrefix + "laser";
				publishers[dataType] = node.Advertise<robot_messages::LaserScanMsg>(topicName);
				break;
			case ROBOT_DATA_LASER_CFG:
				topicName = topicPrefix + "laser_cfg";
				publishers[dataType] = node.Advertise<robot_messages::LaserCfgMsg>(topicName);
				break;
			case ROBOT_DATA_MOTOR_CURRENTS:
				topicName = topicPrefix + "motor_currents";
				publishers[dataType] = node.Advertise<robot_messages::MotorCurrentMsg>(topicName);
				break;
			case ROBOT_DATA_CLIFF:
				topicName = topicPrefix + "cliff";
				publishers[dataType] = node.Advertise<robot_messages::CliffMsg>(topicName);
				break;
			case ROBOT_DATA_NUMBER:
				break;
			}
			if (!topicPrefix.empty()) {
				fprintf(stderr, "advertised topic %s\n", topicName.c_str());
				robotDataUpdated[dataType] = 0;
			}
		}
	}

	// advertise capabilities publisher
	{
		// ROBOT_DATA_NUMBER is used to retrieve robot capabilities
		std::string topicName = topicPrefix + "capabilities";
		publishers[ROBOT_DATA_NUMBER] = node.Advertise<robot_messages::RobotCapabilities>(topicName);
	}

	// advertise services
	{
		std::string serviceName;
		serviceName = topicPrefix + "capabilities";
		if (node.Advertise<CRobotServer,robot_messages::RobotCapabilities>(serviceName, &CRobotServer::capabilitiesRequestServer, this)) {
			fprintf(stderr, "advertised service '%s'\n", serviceName.c_str());
		} else {
			fprintf(stderr, "ERROR: Failed to advertise service '%s'\n", serviceName.c_str());
		}
		serviceName = topicPrefix + "laser_configuration_request"; // service for synchronous (request-response call) request
		if (node.Advertise<CRobotServer,robot_messages::ConfigurationRequest,robot_messages::LaserCfgMsg>(serviceName, &CRobotServer::laserConfigurationRequestServer, this)) {
			fprintf(stderr, "advertised service '%s'\n", serviceName.c_str());
		} else {
			fprintf(stderr, "ERROR: Failed to advertise service '%s'\n", serviceName.c_str());
		}
		serviceName = topicPrefix + "configuration_request"; // service for asynchronous requests of any type
		if (node.Advertise<CRobotServer,robot_messages::ConfigurationRequest>(serviceName, &CRobotServer::configurationRequestServer, this)) {
			fprintf(stderr, "advertised service '%s'\n", serviceName.c_str());
		} else {
			fprintf(stderr, "ERROR: Failed to advertise service '%s'\n", serviceName.c_str());
		}
		serviceName = topicPrefix + "control_velocities";
		if (node.Advertise<CRobotServer,robot_messages::MotionControlMsg>(serviceName, &CRobotServer::velocitiesServer, this)) {
			fprintf(stderr, "advertised service '%s'\n", serviceName.c_str());
		} else {
			fprintf(stderr, "ERROR: Failed to advertise service '%s'\n", serviceName.c_str());
		}
	}

	// subscribe for client requests sent as published topics
	{
		if (node.Subscribe<CRobotServer>("/client_requests", &CRobotServer::cbClientRequests, this)) {
			fprintf(stderr, "subscribed for asynchronous client requests\n");
		} else {
			fprintf(stderr, "ERROR: failed to subscribe for asynchronous client requests\n");
		}

		std::string topicName;
		topicName = topicPrefix + "control_velocities";
		if (node.Subscribe<CRobotServer>(topicName, &CRobotServer::cbControlVelocities, this)) {
		}
	}

	robot->setDataCallback(robot_data_callback, this); // robotDataCallback method is called
	threadInterrupt = false;
	threadStarted = (pthread_create(&thread, 0, thread_body, this) == 0);
}

CRobotServer::~CRobotServer()
{
	if (threadStarted) {
		threadInterrupt = true;
		pthread_join(thread, 0);
	}
}

void CRobotServer::robotDataCallback(TRobotDataType dataType, int index)
{
	if (verbose) fprintf(stderr, "DEBUG: robot data: type=%d, index=%d %s\n", (int) dataType, index, robot_messages::strDataType(dataType));
	ignition::transport::Node::Publisher * pub = getPublisher(dataType);
	robotDataUpdated[dataType] = 0;
	if (pub) {
		switch (dataType) {
		case ROBOT_DATA_ODOMETRY:
			publishRobotOdometry(pub, index);
			break;
		case ROBOT_DATA_BATTERY:
			publishRobotBattery(pub, index);
			checkBatteryStatus(index);
			break;
		case ROBOT_DATA_BUMPER:
			publishRobotBumper(pub, index);
			break;
		case ROBOT_DATA_CHARGER:
			publishRobotCharger(pub, index);
			break;
		case ROBOT_DATA_DOCKING_SENSOR:
			publishRobotDockingSensor(pub, index);
			break;
		case ROBOT_DATA_LASER:
			publishRobotLaser(pub, index);
			break;
		case ROBOT_DATA_LASER_CFG:
			publishRobotLaserCfg(pub, index);
			break;
		case ROBOT_DATA_MOTOR_CURRENTS:
			publishRobotMotorCurrents(pub, index);
			break;
		case ROBOT_DATA_CLIFF:
			publishRobotCliff(pub, index);
			break;
		case ROBOT_DATA_NONE:
		case ROBOT_DATA_NUMBER:
			break;
		}
	}
}

ignition::transport::Node::Publisher * CRobotServer::getPublisher(TRobotDataType dataType)
{
	std::map<TRobotDataType, ignition::transport::Node::Publisher>::iterator it = publishers.find(dataType);
	if (it == publishers.end()) return NULL;
	return &(it->second);
}

void CRobotServer::publishRobotOdometry(ignition::transport::Node::Publisher * publisher, int index)
{
	CPose pose;
	if (robot->getOdometry(pose)) {
		robot_messages::OdometryMsg odoMsg;
		odoMsg.set_index(index);
		odoMsg.set_x(pose.x);
		odoMsg.set_y(pose.y);
		odoMsg.set_heading(pose.heading);
		//fprintf(stderr, "publish odometry\n");
		publisher->Publish(odoMsg);
	}
}

void CRobotServer::publishRobotRawOdometry(ignition::transport::Node::Publisher * publisher, int index)
{
	double left, right;
	if (robot->getRawOdometry(left, right)) {
		robot_messages::RawOdometryMsg odoMsg;
		odoMsg.set_index(index);
		odoMsg.set_left(left);
		odoMsg.set_right(right);
		//fprintf(stderr, "publish raw odometry\n");
		publisher->Publish(odoMsg);
	}
}

void CRobotServer::publishRobotBattery(ignition::transport::Node::Publisher * publisher, int index)
{
	double level = robot->getBatteryLevel();
	if (!isnan(level)) {
		robot_messages::BatteryMsg batMsg;
		batMsg.set_index(index);
		batMsg.set_batterychargelevel(level);
		//fprintf(stderr, "publish battery\n");
		publisher->Publish(batMsg);
	}
}

void CRobotServer::publishRobotBumper(ignition::transport::Node::Publisher * publisher, int index)
{
	robot_messages::BumperMsg bumperMsg;
	bumperMsg.set_index(index);
	bumperMsg.set_state(robot->getFrontBumperState());
	//fprintf(stderr, "publish bumper\n");
	publisher->Publish(bumperMsg);
}

void CRobotServer::publishRobotCharger(ignition::transport::Node::Publisher * publisher, int index)
{
	robot_messages::ChargerMsg chgMsg;
	chgMsg.set_index(index);
	chgMsg.set_state(robot->getChargerState() ? 1 : 0);
	//fprintf(stderr, "publish charger\n");
	publisher->Publish(chgMsg);
}

void CRobotServer::publishRobotDockingSensor(ignition::transport::Node::Publisher * publisher, int index)
{
	std::vector<uint8_t> data;
	if (robot->getDockingSensorData(data)) {
		robot_messages::DockingSensorMsg dsMsg;
		dsMsg.set_index(index);
		for (unsigned int i = 0; i < data.size(); i++) {
			dsMsg.add_state(data[i]);
		}
		//fprintf(stderr, "publish docking sensor\n");
		publisher->Publish(dsMsg);
	}
}

void CRobotServer::publishRobotLaser(ignition::transport::Node::Publisher * publisher, int index)
{
	CLaserScan scan;
	if (robot->getLaserSensorData(scan, index)) {
		robot_messages::LaserScanMsg laserMsg;
		laserMsg.set_index(index);
		for (unsigned int i = 0; i < scan.range.size(); i++) {
			laserMsg.add_range(scan.range[i]);
			if (!scan.intensity.empty()) {
				laserMsg.add_intensity(scan.intensity[i]);
			}
		}
		if (verbose) fprintf(stderr, "publish laser %d\n", index);
		publisher->Publish(laserMsg);
	} else {
		fprintf(stderr, "ERROR: publishRobotLaser() failed to get laser(%d) data\n", index);
	}
}

bool CRobotServer::publishRobotLaserCfg(ignition::transport::Node::Publisher * publisher, int index)
{
	bool result;
	robot_messages::LaserCfgMsg cfgMsg;
	result = prepareLaserSensorConfigurationMessage(cfgMsg, index);
	if (result) {
		if (!publisher) {
			publisher = getPublisher(ROBOT_DATA_LASER_CFG);
			if (!publisher) {
				fprintf(stderr, "ERROR: no publisher for laser configuration\n");
				return false;
			}
		}
		fprintf(stderr, "publish laser %d cfg\n", index);
		publisher->Publish(cfgMsg);
	}
	return result;
}

void CRobotServer::publishRobotMotorCurrents(ignition::transport::Node::Publisher * publisher, int index)
{
	std::vector<double> currents;
	if (robot->getMotorCurrents(currents)) {
		robot_messages::MotorCurrentMsg mcMsg;
		mcMsg.set_index(index);
		mcMsg.set_left(currents[0]);
		mcMsg.set_right(currents[1]);
		//fprintf(stderr, "publish motor currents\n");
		publisher->Publish(mcMsg);
	}
}

void CRobotServer::publishRobotCliff(ignition::transport::Node::Publisher * publisher, int index)
{
	// TODO publishRobotCliff
}

bool CRobotServer::prepareLaserSensorConfigurationMessage(robot_messages::LaserCfgMsg & cfgMsg, int index)
{
	CLaserSensorConfiguration cfg;
	bool result = robot->getLaserSensorConfiguration(cfg, index);
	if (result) {
		cfgMsg.set_index(index);
		cfgMsg.set_maxrange(cfg.maxRange);
		cfgMsg.set_angularresolution(cfg.angularResolution);
		cfgMsg.set_minangle(cfg.minAngle);
		cfgMsg.set_scansamplecount(cfg.scanSampleCount);
		cfgMsg.set_frequency(cfg.frequency);
		cfgMsg.mutable_position()->set_x(cfg.x);
		cfgMsg.mutable_position()->set_y(cfg.y);
		cfgMsg.mutable_position()->set_z(cfg.z);
	} else {
		fprintf(stderr, "ERROR: can not obtain laser %d cfg\n", index);
		fprintf(stderr, "DEBUG: laser count %d\n", robot->getLaserSensorCount());
	}
	return result;
}

void CRobotServer::prepareRobotCapabilitiesResponse(robot_messages::RobotCapabilities & capMsg)
{
	capMsg.clear_capabilities();
	for (TRobotDataType dt = (TRobotDataType) (ROBOT_DATA_NONE+1); dt < ROBOT_DATA_NUMBER; dt = (TRobotDataType) (dt + 1)) {
		int dcount = robot->getRobotSensorCount(dt);
		if (dcount > 0) {
			robot_messages::RobotCapabilities_Capability * cap = capMsg.add_capabilities();
			cap->set_datatype(robot_messages::convertDataType(dt));
			cap->set_count(dcount);
			cap->set_label("");
		}
	}
}

void CRobotServer::configurationRequestServer(const robot_messages::ConfigurationRequest & msg)
{
	TRobotDataType dataType = convertDataType(msg.datatype());
	int index = msg.index();
	fprintf(stderr, "DEBUG: configuration request (%u, %u)\n", dataType, index);
	bool handled = false;
	switch (dataType) {
	case ROBOT_DATA_LASER:
		if (index < robot->getLaserSensorCount()) {
			CLaserSensorConfiguration laserCfg;
			handled = publishRobotLaserCfg(NULL, index);
		} else {
			fprintf(stderr, "WARNING: request configuration of invalid laser (index %d; laser count %d)\n", index, robot->getLaserSensorCount());
		}
		break;
	case ROBOT_DATA_NUMBER:
		// send all capabilities
		{
			ignition::transport::Node::Publisher * pub = getPublisher(dataType);
			if (pub) {
				robot_messages::RobotCapabilities capMsg;
				prepareRobotCapabilitiesResponse(capMsg);
				fprintf(stderr, "DEBUG: sending robot capabilities (%d)\n", (int) capMsg.capabilities_size());
				pub->Publish(capMsg);
				handled = true;
			} else {
				fprintf(stderr, "ERROR: no publisher to send robot capabilities\n");
			}
		}
		break;
	default: ;
	}
	if (!handled) {
		fprintf(stderr, "WARNING: unhandled configuration request (%u, %u)!\n", msg.datatype(), index);
	}
}

void CRobotServer::cbClientRequests(const robot_messages::ConfigurationRequest & msg, const ignition::transport::MessageInfo & info)
{
	if (msg.has_robotname()) {
		if (msg.robotname() == robotName) {
			configurationRequestServer(msg);
		}
	}
}

void CRobotServer::cbControlVelocities(const robot_messages::MotionControlMsg & msg, const ignition::transport::MessageInfo & info)
{
	velocitiesServer(msg);
}

void CRobotServer::velocitiesServer(const robot_messages::MotionControlMsg & msg)
{
	fprintf(stderr, "DEBUG: CRobotServer::velocitiesServer(%.3f, %.3f)\n", msg.forwardvelocity(), msg.angularvelocity());
	robot->setVelocities(msg.forwardvelocity(), msg.angularvelocity());
	lastControlCommandTime = CClock::getTimeMs();
	lastControlCommandTimeValid = true;
	// TODO: estimate average command rate, warn if rate too high
}

bool CRobotServer::capabilitiesRequestServer(robot_messages::RobotCapabilities & msg)
{
	// NOTE: this server is used only in synchronous request-response operation
	prepareRobotCapabilitiesResponse(msg);
	return true;
}

bool CRobotServer::laserConfigurationRequestServer(const robot_messages::ConfigurationRequest & request, robot_messages::LaserCfgMsg & response)
{
	// NOTE: this server is used only in synchronous request-response operation
	prepareLaserSensorConfigurationMessage(response, request.index());
	return true;
}

void CRobotServer::checkBatteryStatus(int index)
{
	double level = robot->getBatteryLevel();
	if (!isnan(level)) {
		uint32_t t = CClock::getTimeMs();
		uint32_t dt = t - lastBatteryReportTime;
		if (dt > BATTERY_REPORT_PERIOD) {
			fprintf(stderr, "BATTERY: %.1f%%\n", level);
			lastBatteryReportTime = t;
		}
	}
}

void CRobotServer::threadBody()
{
	while (!threadInterrupt) {
		usleep(SERVER_THREAD_PERIOD_MS * 1000);

		// read robot values not updated by callback
		for (TRobotDataType dataType = ROBOT_DATA_NONE; dataType < ROBOT_DATA_NUMBER; dataType = (TRobotDataType) (dataType+1)) {
			if (robotDataUpdated[dataType] < 0) continue;
			if (robotDataUpdated[dataType]*SERVER_THREAD_PERIOD_MS >= 2000) {
				ignition::transport::Node::Publisher * pub = getPublisher(dataType);
				if (pub) {
					switch (dataType) {
					case ROBOT_DATA_ODOMETRY:
						publishRobotOdometry(pub);
						break;
					case ROBOT_DATA_BATTERY:
						publishRobotBattery(pub);
						break;
					case ROBOT_DATA_BUMPER:
						publishRobotBumper(pub);
						break;
					case ROBOT_DATA_CHARGER:
						publishRobotCharger(pub);
						break;
					case ROBOT_DATA_DOCKING_SENSOR:
						publishRobotDockingSensor(pub);
						break;
					case ROBOT_DATA_LASER:
						// do nothing, laser data sending should be always event-driven
						break;
					case ROBOT_DATA_LASER_CFG:
						// do nothing
						break;
					case ROBOT_DATA_MOTOR_CURRENTS:
						publishRobotMotorCurrents(pub);
						break;
					case ROBOT_DATA_CLIFF:
						publishRobotCliff(pub);
						break;
					case ROBOT_DATA_NONE:
					case ROBOT_DATA_NUMBER:
						break;
					}
				}
			} else {
				robotDataUpdated[dataType]++;
			}
		}

		// control failsafe
		if (lastControlCommandTimeValid) {
			uint32_t t = CClock::getTimeMs();
			uint32_t tFromLastControlCommand = t - lastControlCommandTime;
			if (tFromLastControlCommand > CONTROL_TIMEOUT_MS) {
				robot->setVelocities(0, 0);
				lastControlCommandTimeValid = false;
			}
		}
	}
}

/* end of robot_server.cc */
