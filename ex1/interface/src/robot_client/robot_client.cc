/*
 * File name: robot_client.cc
 * Date:      2018/09/18 09:09
 * Author:    Jan Chudoba
 */

#include "robot_client/robot_client.h"

CRobotClient::CRobotClient() :
	lastPoseValid(false),
	lastPoseFresh(false),
	batteryLevel(NAN),
	batteryLevelFresh(false),
	frontBumperState(0),
	frontBumperStateFresh(false),
	chargerState(false),
	chargerStateFresh(false),
	dockingSensorDataValid(false),
	dockingSensorDataFresh(false),
	motorCurrentsValid(false),
	motorCurrentsFresh(false),
	asyncMode(false),
	useRequestResponseServices(false),
	verbose(false)
{
	robot_messages::robot_messages_check();

	char * envAsync = getenv("IGN_ASYNC");
	if (envAsync && envAsync[0] == '1') {
		fprintf(stderr, "DEBUG: using async mode for robot configuration subscribtions\n");
		setAsyncMode();
	}
}

CRobotClient::~CRobotClient()
{
}

bool CRobotClient::initialize(CRobotConfiguration * config)
{
	CRobotClientConfiguration * cfg = (CRobotClientConfiguration*) config;
	if (cfg && !cfg->robotName.empty()) {
		robotName = cfg->robotName;
	} else {
		robotName = "turtlebot";
	}
	topicPrefix = std::string("/") + robotName + "/";

	bool result = true;

	result &= node.Subscribe<CRobotClient>(topicPrefix + "capabilities", &CRobotClient::cbCapabilities, this);
	result &= node.Subscribe<CRobotClient>(topicPrefix + "odometry", &CRobotClient::cbOdometry, this);
	result &= node.Subscribe<CRobotClient>(topicPrefix + "battery", &CRobotClient::cbBattery, this);
	result &= node.Subscribe<CRobotClient>(topicPrefix + "bumper", &CRobotClient::cbBumper, this);
	result &= node.Subscribe<CRobotClient>(topicPrefix + "charger", &CRobotClient::cbCharger, this);
	result &= node.Subscribe<CRobotClient>(topicPrefix + "docking_sensor", &CRobotClient::cbDockingSensor, this);
	result &= node.Subscribe<CRobotClient>(topicPrefix + "laser", &CRobotClient::cbLaserScan, this);
	result &= node.Subscribe<CRobotClient>(topicPrefix + "laser_cfg", &CRobotClient::cbLaserConfiguration, this);
	result &= node.Subscribe<CRobotClient>(topicPrefix + "motor_currents", &CRobotClient::cbMotorCurrents, this);
	result &= node.Subscribe<CRobotClient>(topicPrefix + "cliff", &CRobotClient::cbCliff, this);
	//result &= node.Subscribe<CRobotClient>(topicPrefix + "X", &CRobotClient::cbX, this);
	
	clientRequestPublisher = node.Advertise<robot_messages::ConfigurationRequest>("/client_requests");
	controlVelocitiesPublisher = node.Advertise<robot_messages::MotionControlMsg>(topicPrefix + "control_velocities");
	
    
	sendRequest(ROBOT_DATA_NUMBER, 0); // request for all capabilities

	if (config && config->waitForInitialization && !useRequestResponseServices) {
		requestedConfiguration[std::pair<TRobotDataType, int>(ROBOT_DATA_NUMBER, 0)] = 1; // request for all capabilities

		CClock clk;
		lockRequests();
		while (!requestedConfiguration.empty()) {
			if (clk.getTotalTimeDifMs() > 5000) {
				fprintf(stderr, "ERROR: Failed to enumerate robot capabiliites or retrieve all sensor configuration!\n");
				break;
			}
			waitForRequests(); // wait is timeouted
			if (clk.getTimeDifMs() > 1000) {
				// re-send cfg requests after timeout
				for (std::map<std::pair<TRobotDataType, int>, int>::iterator it = requestedConfiguration.begin(); it != requestedConfiguration.end(); it++) {
					TRobotDataType dt = it->first.first;
					if (dt > ROBOT_DATA_NONE && dt < ROBOT_DATA_NUMBER) {
						sendRequest(dt, it->first.second);
					} else if (dt == ROBOT_DATA_NUMBER) {
						sendRequest(ROBOT_DATA_NUMBER, 0);
					}
				}
				clk.start();
			}
		}
		unlockRequests();
	}
	//fprintf(stderr, "client initialization done\n");

	return result;
}

bool CRobotClient::setVelocities(double forwardVelocity, double angularVelocity)
{
	robot_messages::MotionControlMsg msg;
	msg.set_index(0);
	msg.set_forwardvelocity(forwardVelocity);
	msg.set_angularvelocity(angularVelocity);
	bool executed;
	if (!asyncMode) {
		// sync mode (over service)
		executed = node.Request(topicPrefix + "control_velocities", msg);
	} else {
		// async mode
		if (controlVelocitiesPublisher) {
			executed = controlVelocitiesPublisher.Publish(msg);
		} else {
			executed = false;
		}
	}
	if (!executed) {
		fprintf(stderr, "ERROR: service 'control_velocities' call failed\n");
	}
	return executed;
}

bool CRobotClient::getOdometry(CPose & odoPose)
{
	if (lastPoseValid) {
		odoPose = lastPose;
		unFresh(ROBOT_DATA_ODOMETRY);
	}
	return lastPoseValid;
}

double CRobotClient::getBatteryLevel()
{
	return batteryLevel;
}

uint8_t CRobotClient::getFrontBumperState()
{
	return frontBumperState;
}

bool CRobotClient::getChargerState()
{
	return chargerState;
}

bool CRobotClient::getDockingSensorData(std::vector<uint8_t> & dockingData)
{
	dockingData = dockingSensorData;
	return dockingSensorDataValid;
}

bool CRobotClient::getMotorCurrents(std::vector<double> & currents)
{
	currents = motorCurrents;
	return motorCurrentsValid;
}

int CRobotClient::robotProvidesDataInternal(TRobotDataType dataType)
{
	std::map<TRobotDataType, int>::iterator it = capabilities.find(dataType);
	if (it == capabilities.end()) return 0;
	return it->second;
}

bool CRobotClient::checkLaserSensorPresent(int index, bool registerNew)
{
	bool result = isLaserSensorPresent(index);
	if (!result) {
		if (registerNew) {
			fprintf(stderr, "registering new laser sensor with index %d\n", index);
			if (getLaserSensorCount() != index) {
				fprintf(stderr, "FATAL ERROR, index mismatch!\n");
				// TODO: solve index mismatch - need option to register laser with specific ID ...
			}
			CRobotClientLaser * newLaser = new CRobotClientLaser();
			registerLaserSensor(newLaser);
			laserSensorMap[index] = newLaser;
		}
	}
	return result;
}

void CRobotClient::cbCapabilities(const robot_messages::RobotCapabilities & msg, const ignition::transport::MessageInfo & info)
{
	if (verbose) fprintf(stderr, "DEBUG: CRobotClient::cbCapabilities(%d)\n", msg.capabilities_size());
	fprintf(stderr, "robot capabilities:\n");
	capabilities.clear();
	lockRequests();
	deleteRequest(ROBOT_DATA_NUMBER, 0);
	unlockRequests();
	for (int i = 0; i < msg.capabilities_size(); i++) {
		const robot_messages::RobotCapabilities::Capability & cap = msg.capabilities(i);
		TRobotDataType dataType = convertDataType(cap.datatype());
		fprintf(stderr, " %s (%d) %s\n", robot_messages::strDataType(dataType), cap.count(), cap.label().c_str());
		int index_offset = capabilities[dataType];
		capabilities[dataType] += cap.count();
		if (dataType == ROBOT_DATA_LASER) {
			for (unsigned int index = 0; index < cap.count(); index++) {
				int realIndex = index_offset + index;
				// request laser configuration
				if (sendRequest(dataType, realIndex)) {
					if (!useRequestResponseServices) {
						lockRequests();
						requestedConfiguration[std::pair<TRobotDataType, int>(ROBOT_DATA_LASER, realIndex)] = 1;
						unlockRequests();
					}
				}
			}
		}
	}
	lockRequests();
	notifyRequests();
	unlockRequests();
}

void CRobotClient::cbOdometry(const robot_messages::OdometryMsg & msg, const ignition::transport::MessageInfo & info)
{
	if (verbose) fprintf(stderr, "DEBUG: CRobotClient::cbOdometry(%.3f, %.3f, %.3f)\n", msg.x(), msg.y(), msg.heading());
	lastPose.x = msg.x();
	lastPose.y = msg.y();
	lastPose.heading = msg.heading();
	lastPoseValid = true;
	lastPoseFresh = true;
	setFresh(ROBOT_DATA_ODOMETRY, msg.index());
	callDataCallback(ROBOT_DATA_ODOMETRY, msg.index());
}

void CRobotClient::cbBattery(const robot_messages::BatteryMsg & msg, const ignition::transport::MessageInfo & info)
{
	if (verbose) fprintf(stderr, "DEBUG: CRobotClient::cbBattery(%.3f)\n", msg.batterychargelevel());
	batteryLevel = msg.batterychargelevel();
	batteryLevelFresh = true;
	setFresh(ROBOT_DATA_BATTERY, msg.index());
	callDataCallback(ROBOT_DATA_BATTERY, msg.index());
}

void CRobotClient::cbBumper(const robot_messages::BumperMsg & msg, const ignition::transport::MessageInfo & info)
{
	frontBumperState = msg.state();
	frontBumperStateFresh = true;
	setFresh(ROBOT_DATA_BUMPER, msg.index());
	callDataCallback(ROBOT_DATA_BUMPER, msg.index());
}

void CRobotClient::cbCharger(const robot_messages::ChargerMsg & msg, const ignition::transport::MessageInfo & info)
{
	chargerState = msg.state();
	chargerStateFresh = true;
	setFresh(ROBOT_DATA_CHARGER, msg.index());
	callDataCallback(ROBOT_DATA_CHARGER, msg.index());
}

void CRobotClient::cbDockingSensor(const robot_messages::DockingSensorMsg & msg, const ignition::transport::MessageInfo & info)
{
	dockingSensorData.resize(msg.state_size());
	for (int i = 0; i < msg.state_size(); i++) {
		dockingSensorData[i] = msg.state(i);
	}
	dockingSensorDataValid = true;
	dockingSensorDataFresh = true;
	setFresh(ROBOT_DATA_DOCKING_SENSOR, msg.index());
	callDataCallback(ROBOT_DATA_DOCKING_SENSOR, msg.index());
}

void CRobotClient::cbLaserScan(const robot_messages::LaserScanMsg & msg, const ignition::transport::MessageInfo & info)
{
	if (verbose) fprintf(stderr, "got laser(%d) scan, laser count=%d\n", msg.index(), getLaserSensorCount());
	checkLaserSensorPresent(msg.index());
	CRobotClientLaser & laser = getLaserSensor(msg.index());
	laser.updateLaserScan(msg);
}

void CRobotClient::cbLaserConfiguration(const robot_messages::LaserCfgMsg & msg, const ignition::transport::MessageInfo & info)
{
	checkLaserSensorPresent(msg.index());
	CRobotClientLaser & laser = getLaserSensor(msg.index());
	if (verbose) fprintf(stderr, "got laser(%d) configuration\n", msg.index());
	laser.updateLaserConfiguration(msg);
	lockRequests();
	deleteRequest(ROBOT_DATA_LASER, msg.index());
	notifyRequests();
	unlockRequests();
	setFresh(ROBOT_DATA_LASER_CFG, msg.index());
	callDataCallback(ROBOT_DATA_LASER_CFG, msg.index());
	fprintf(stderr, "laser %d configuration %s\n", msg.index(), laser.isConfigurationValid() ? "valid":"INVALID");
}

void CRobotClient::cbMotorCurrents(const robot_messages::MotorCurrentMsg & msg, const ignition::transport::MessageInfo & info)
{
	motorCurrents.resize(2);
	motorCurrents[0] = msg.left();
	motorCurrents[1] = msg.right();
	motorCurrentsValid = true;
	motorCurrentsFresh = true;
	setFresh(ROBOT_DATA_MOTOR_CURRENTS, msg.index());
	callDataCallback(ROBOT_DATA_MOTOR_CURRENTS, msg.index());
}

void CRobotClient::cbCliff(const robot_messages::CliffMsg & msg, const ignition::transport::MessageInfo & info)
{
	// TODO .. cbCliff
	// setFresh(ROBOT_DATA_CLIFF, msg.index());
	// callDataCallback(ROBOT_DATA_CLIFF, msg.index());
}

void CRobotClient::deleteRequest(TRobotDataType dataType, int index)
{
	std::map<std::pair<TRobotDataType, int>, int>::iterator it = requestedConfiguration.find(std::pair<TRobotDataType,int>(dataType, index));
	if (it != requestedConfiguration.end()) {
		requestedConfiguration.erase(it);
	}
}

bool CRobotClient::sendRequest(TRobotDataType dataType, int index)
{
	// TODO: option to send request using service call
	const char * strDataType = (dataType != ROBOT_DATA_NUMBER) ? robot_messages::strDataType(dataType) : "capabilities";
	fprintf(stderr, "DEBUG: send request for configuration (%s, %d)\n", strDataType, index);
	robot_messages::ConfigurationRequest req;
	req.set_datatype(robot_messages::convertDataType(dataType));
	req.set_index(index);
	if (!asyncMode) {
		if (!useRequestResponseServices) {
			return (node.Request(topicPrefix + "configuration_request", req));
		} else {
			switch (dataType) {
			case ROBOT_DATA_LASER:
				return sendLaserCfgRequest(index);
				break;
			case ROBOT_DATA_NUMBER:
				return sendRobotCapabilitiesRequest();
				break;
			default:
				fprintf(stderr, "ERROR: can not send '%s' configuration request using request-response service (service does not exist!)\n", robot_messages::strDataType(dataType));
				return false;
			}
		}
	} else {
		req.set_robotname(robotName);
		if (!clientRequestPublisher) return false;
		return clientRequestPublisher.Publish(req);
	}
}

bool CRobotClient::sendRobotCapabilitiesRequest()
{
	robot_messages::RobotCapabilities response;
	std::string serviceName = topicPrefix + "capabilities";
	bool result;
	if (node.Request(serviceName, 5000, response, result)) {
		if (result) {
			ignition::transport::MessageInfo info;
			cbCapabilities(response, info);
		} else {
			fprintf(stderr, "ERROR: service '%s' call failed\n", serviceName.c_str());
		}
	} else {
		fprintf(stderr, "ERROR: service '%s' call timeout\n", serviceName.c_str());
		result = false;
	}
	return result;
}

bool CRobotClient::sendLaserCfgRequest(int index)
{
	robot_messages::ConfigurationRequest request;
	request.set_datatype(robot_messages::ROBOT_DATA_LASER);
	request.set_index(index);
	robot_messages::LaserCfgMsg response;
	std::string serviceName = topicPrefix + "laser_configuration_request";
	bool result;
	if (node.Request(serviceName, request, 5000, response, result)) {
		if (result) {
			ignition::transport::MessageInfo info;
			cbLaserConfiguration(response, info);
		} else {
			fprintf(stderr, "ERROR: service '%s' call failed\n", serviceName.c_str());
		}
	} else {
		fprintf(stderr, "ERROR: service '%s' call timeout\n", serviceName.c_str());
		result = false;
	}
	return result;
}


/*
bool CRobotClient::getLaserConfiguration(CLaserSensorConfiguration & cfg)
{
	std::cout << "sendLaserCfgRequest " << std::endl;
	robot_messages::LaserCfgMsg msg;
	std::string serviceName = topicPrefix + "laser_configuration_request";
	bool result;
	node.Request(serviceName, 5000,msg, result);
	cfg.maxRange = msg.maxrange();
	cfg.angularResolution = msg.angularresolution();
	cfg.minAngle = msg.minangle();
	cfg.scanSampleCount = msg.scansamplecount();
	cfg.frequency = msg.frequency();

	return result;
}
*/
/* end of robot_client.cc */
