/*
 * File name: battery.cc
 * Date:      2018/09/13 09:46
 * Author:    Jan Chudoba
 */

#include <stdio.h>

#include "robot/robot_interface.h"
#include "kobuki/kobuki_robot.h"

int main(int argc, char ** argv)
{
	CKobukiRobotConfiguration cfg;
	cfg.waitForInitialization = true;
	CKobukiRobot robot;
	if (!robot.initialize(&cfg)) {
		return 1;
	}
	fprintf(stderr, "battery %d%%\n", (int) (100*robot.getBatteryLevel()));
	return 0;
}

/* end of battery.cc */
