/*
 * File name: kobuki_robot.cc
 * Date:      2018/09/04 15:11
 * Author:    Jan Chudoba
 */

#include "kobuki/kobuki_robot.h"

#define ODOMETRY_CONSTANT 1e-3 // TODO !!!

CKobukiRobot::CKobukiRobot() :
	slot_stream_data(&CKobukiRobot::processStreamData, *this),
	poseValid(false),
	lastOdoValid(false),
	odoLeft(0),
	odoRight(0),
	odoValid(false)
{
}

CKobukiRobot::~CKobukiRobot()
{
}

bool CKobukiRobot::initialize(CRobotConfiguration * config)
{
	CKobukiRobotConfiguration * cfg = (CKobukiRobotConfiguration*) config;
	parameters.device_port = cfg->deviceName.empty() ? "/dev/ttyUSB0" : cfg->deviceName;
	parameters.sigslots_namespace = "/kobuki";
	parameters.enable_acceleration_limiter = false;
	parameters.battery_capacity = 16.5;
	parameters.battery_low = 14.0;
	parameters.battery_dangerous = 13.2;

	bool result;

	try {
		kobuki.init(parameters);
		result = true;
	} catch ( ecl::StandardException &e ) {
		std::cout << e.what();
		result = false;
	}

	if (result) {
		slot_stream_data.connect("/kobuki/stream_data");
		usleep(100000);

		CClock clk;
		if (config && config->waitForInitialization) {
			// wait until robot connection is acknowledged
			while (!kobuki.isAlive()) {
				if (clk.getTotalTimeDifMs() > 3000) {
					fprintf(stderr, "ERROR: Failed to connect to robot\n");
					result = false;
					break;
				}
				usleep(10000);
			}
		}
	}

	return result;
}

bool CKobukiRobot::setVelocities(double forwardVelocity, double angularVelocity)
{
	kobuki.setBaseControl(forwardVelocity, angularVelocity);
	return true;
}

bool CKobukiRobot::getRawOdometry(double & left, double & right)
{
	if (odoValid) {
		left = odoLeft;
		right = odoRight;
	}
	return odoValid;
}

bool CKobukiRobot::getOdometry(CPose & odoPose)
{
	odoPose.x = pose.x();
	odoPose.y = pose.y();
	odoPose.heading = pose.heading();
	return poseValid;
}

double CKobukiRobot::getBatteryLevel()
{
	return kobuki.batteryStatus().percent() / 100.0;
}

uint8_t CKobukiRobot::getFrontBumperState()
{
	kobuki.lockDataAccess();
	kobuki::CoreSensors::Data coreSensors = kobuki.getCoreSensorData();
	kobuki.unlockDataAccess();
	return coreSensors.bumper;
}

bool CKobukiRobot::getChargerState()
{
	kobuki::Battery::State state = kobuki.batteryStatus().charging_state;
	return (state == kobuki::Battery::State::Charging || state == kobuki::Battery::State::Charged);
}

bool CKobukiRobot::getDockingSensorData(std::vector<uint8_t> & dockingData)
{
	kobuki.lockDataAccess();
	kobuki::DockIR::Data data = kobuki.getDockIRData();
	kobuki.unlockDataAccess();
	dockingData.clear();
	if (data.docking.size() > 0) {
		for (int i = 0; i < (int) data.docking.size(); i++) {
			dockingData.push_back(data.docking[i]);
		}
		return true;
	} else {
		return false;
	}
}

bool CKobukiRobot::getMotorCurrents(std::vector<double> & currents)
{
	kobuki.lockDataAccess();
	kobuki::Current::Data data = kobuki.getCurrentData();
	kobuki.unlockDataAccess();
	currents.clear();
	if (data.current.size() >= 2) {
		currents.push_back((double) data.current[0] / 255.0);
		currents.push_back((double) data.current[1] / 255.0);
		return true;
	} else {
		return false;
	}
}

void CKobukiRobot::processStreamData()
{
	ecl::LegacyPose2D<double> pose_update;
	ecl::linear_algebra::Vector3d pose_update_rates;
	kobuki.updateOdometry(pose_update, pose_update_rates);
	pose *= pose_update;
	poseValid = true;
	//dx += pose_update.x();
	//dth += pose_update.heading();
	//std::cout << dx << ", " << dth << std::endl;
	//std::cout << kobuki.getHeading() << ", " << pose.heading() << std::endl;
	//std::cout << "[" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
	
	uint16_t odoLeft16 = kobuki.getCoreSensorData().left_encoder;
	uint16_t odoRight16 = kobuki.getCoreSensorData().right_encoder;
	if (lastOdoValid) {
		int16_t dLeft = odoLeft16 - lastOdoLeft;
		int16_t dRight = odoRight16 - lastOdoRight;
		odoLeft += ODOMETRY_CONSTANT * (double)dLeft;
		odoRight += ODOMETRY_CONSTANT * (double)dRight;
		odoValid = true;
	}
	lastOdoLeft = odoLeft16;
	lastOdoRight = odoRight16;
	lastOdoValid = true;

	callDataCallback(ROBOT_DATA_ODOMETRY, 0);
	callDataCallback(ROBOT_DATA_BUMPER, 0);
	callDataCallback(ROBOT_DATA_BATTERY, 0);
	callDataCallback(ROBOT_DATA_CHARGER, 0);
	callDataCallback(ROBOT_DATA_CLIFF, 0);
	//callDataCallback(ROBOT_DATA_DOCKING_SENSOR, 0);
	//callDataCallback(ROBOT_DATA_MOTOR_CURRENTS, 0);
	// TODO: what other data are updated here?
	// see http://docs.ros.org/groovy/api/kobuki_driver/html/enSigslotsGuide.html
	// see https://yujinrobot.github.io/kobuki/enAppendixProtocolSpecification.html
}

int CKobukiRobot::robotProvidesDataInternal(TRobotDataType dataType)
{
	switch (dataType) {
		case ROBOT_DATA_NONE: return 0;
		case ROBOT_DATA_ODOMETRY: return 1;
		case ROBOT_DATA_BATTERY: return 1;
		case ROBOT_DATA_BUMPER: return 1;
		case ROBOT_DATA_CHARGER: return 1;
		case ROBOT_DATA_DOCKING_SENSOR: return 1;
		case ROBOT_DATA_LASER: return getLaserSensorCount();
		case ROBOT_DATA_LASER_CFG: return getLaserSensorCount();
		case ROBOT_DATA_MOTOR_CURRENTS: return 1;
		case ROBOT_DATA_CLIFF: return 1;
		case ROBOT_DATA_NUMBER: return 0;
	}
	return 0;
}

/* end of kobuki_robot.cc */
