/*
 * File name: robot_messages.h
 * Date:      2018/09/19 13:43
 * Author:    Jan Chudoba
 */

#ifndef __ROBOT_MESSAGES_H__
#define __ROBOT_MESSAGES_H__

#include "msgs/robot_messages.pb.h"
//TODO: should be #include "robot_messages/robot_messages.pb.h" !
#include "robot/robot_interface.h"

namespace robot_messages {

	const char * strDataType(TRobotDataType dataType);

	TRobotDataType convertDataType(robot_messages::RobotDataType dataType);
	robot_messages::RobotDataType convertDataType(TRobotDataType dataType);

	void robot_messages_check();

} // end of namespace robot_messages

#endif

/* end of robot_messages.h */
