/*
 * File name: trobot_server.cc
 * Date:      2018/10/09
 * Author:    Jan Chudoba
 */

#include <stdio.h>

#include "robot/robot_interface.h"
#include "kobuki/kobuki_robot.h"
#include "sick/laser_sick.h"
#include "robot_server/robot_server.h"

#define WITH_SICK

int main(int argc, char ** argv)
{
	bool withLaser = false;

	int a = 1;
	while (a < argc) {
		if (argv[a][0] != '-') break;
		switch(argv[a][1]) {
		case 'l':
			withLaser = true;
			break;
		default:
			fprintf(stderr, "invalid option\n");
			return 1;
		}
		a++;
	}

	fprintf(stderr, "trobot_server\n");
	if (withLaser) {
		fprintf(stderr, " with laser\n");
	}

	CKobukiRobotConfiguration cfg;
	CKobukiRobot robot;

#ifdef WITH_SICK
	CSickLaser * sick = new CSickLaser();
	sick->initialize("192.168.80.2"); // TODO: cfg file
	robot.registerLaserSensor(sick);
	usleep(1000000); // TODO: better way to wait for laser configuration!
	CLaserSensorConfiguration laserCfg;
	if (sick->getConfiguration(laserCfg)) {
		fprintf(stderr, "laser configuration is valid\n");
	} else {
		fprintf(stderr, "WARNING: laser configuration is invalid!\n");
	}
#endif //  WITH_SICK

	fprintf(stderr, "initialize robot ...\n");
	robot.initialize(&cfg);

	// TODO: detect successful connection

	fprintf(stderr, "battery %d%%\n", (int) (100*robot.getBatteryLevel()));

	fprintf(stderr, "start server ...\n");
	CRobotServer server(&robot, "turtlebot");

	fprintf(stderr, "wait ...\n");
	//usleep(10000000);
	fprintf(stderr, "Press ENTER to stop\n");
	getchar();
	fprintf(stderr, "done.\n");

	return 0;
}

/* end of trobot_server.cc */
