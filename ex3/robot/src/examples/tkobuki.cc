/*
 * File name: tkobuki.cc
 * Date:      2018/09/05 12:03
 * Author:    Jan Chudoba
 */

#include <stdio.h>

#include "robot/robot_interface.h"
#include "kobuki/kobuki_robot.h"

#define WITH_SICK

#ifdef WITH_SICK
#include "sick/laser_sick.h"
#endif

int main(int argc, char ** argv)
{
	fprintf(stderr, "Kobuki turtlebot demo\n");
	CKobukiRobotConfiguration cfg;
	CKobukiRobot robot;

#ifdef WITH_SICK
	CSickLaser * sick = new CSickLaser();
	sick->initialize("192.168.80.2");
	robot.registerLaserSensor(sick);
#endif //  WITH_SICK

	cfg.waitForInitialization = true;
	if (!robot.initialize(&cfg)) {
		return 1;
	}

	// TODO: detect successful connection

	fprintf(stderr, "battery %d%%\n", (int) (100*robot.getBatteryLevel()));

	CPose pose;
	robot.getOdometry(pose);
	fprintf(stderr, "odometry: %.3f, %.3f, %.3f\n", pose.x, pose.y, pose.heading);

	fprintf(stderr, "move ...\n");
	robot.setVelocities(0.1, 0.0);
	usleep(2000000);

	fprintf(stderr, "stop\n");
	robot.setVelocities(0.0, 0.0);

	robot.getOdometry(pose);
	fprintf(stderr, "odometry: %.3f, %.3f, %.3f\n", pose.x, pose.y, pose.heading);

	fprintf(stderr, "battery %d%%\n", (int) (100*robot.getBatteryLevel()));

	fprintf(stderr, "bumper %1X\n", robot.getFrontBumperState());

#ifdef WITH_SICK
	CLaserScan scan;
	if (robot.getLaserSensorData(scan)) {
		fprintf(stderr, "laser data: %d samples, middle %.3f m\n", (int) scan.range.size(), scan.range[scan.range.size()/2]);
	} else {
		fprintf(stderr, "ERROR: failed to get laser data\n");
	}
#endif

	return 0;
}

/* end of tkobuki.cc */
