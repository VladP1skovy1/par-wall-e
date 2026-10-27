/*
 * File name: robot_messages.cc
 * Date:      2018/09/19 14:58
 * Author:    Jan Chudoba
 */

#include <stdio.h>

#include "robot_messages/robot_messages.h"

const char * robot_messages::strDataType(TRobotDataType dataType)
{
	switch (dataType) {
	case ::ROBOT_DATA_NONE: return "<none>";
	case ::ROBOT_DATA_ODOMETRY: return "odometry";
	case ::ROBOT_DATA_BATTERY: return "battery";
	case ::ROBOT_DATA_BUMPER: return "bumper";
	case ::ROBOT_DATA_CHARGER: return "charger";
	case ::ROBOT_DATA_DOCKING_SENSOR: return "docking_sensor";
	case ::ROBOT_DATA_LASER: return "laser";
	case ::ROBOT_DATA_LASER_CFG: return "laser_cfg";
	case ::ROBOT_DATA_MOTOR_CURRENTS: return "motor_current";
	case ::ROBOT_DATA_CLIFF: return "cliff";
	case ::ROBOT_DATA_NUMBER: return "<invalid>";
	}
	if (dataType > ::ROBOT_DATA_NONE && dataType < ::ROBOT_DATA_NUMBER) {
		fprintf(stderr, "FATAL ERROR: forgot to add type (%d) to robot_messages::strDataType() !!!\n", dataType);
	}
	return "<invalid>";
}

TRobotDataType robot_messages::convertDataType(robot_messages::RobotDataType dataType)
{
	return (TRobotDataType) dataType;
}

robot_messages::RobotDataType robot_messages::convertDataType(TRobotDataType dataType)
{
	return (robot_messages::RobotDataType) dataType;
}

void robot_messages::robot_messages_check()
{
	if ((int)::ROBOT_DATA_NUMBER != (int)robot_messages::ROBOT_DATA_NUMBER) {
		fprintf(stderr, "FATAL ERROR: TRobotDataType and robot_messages::RobotDataType mismatch !!!\n");
		abort();
	}
}


/* end of robot_messages.cc */
