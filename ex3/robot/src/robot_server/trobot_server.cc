/*
 * File name: trobot_server.cc
 * Date:      2018/09/14 16:11
 * Author:    Jan Chudoba
 */

#include <stdio.h>

#include "robot/robot_interface.h"
#ifdef TEST_KOBUKI
#include "kobuki/kobuki_robot.h"
#endif
#include "dummy_robot/dummy_robot.h"
#include "dummy_robot/dummy_laser.h"
#include "robot_server/robot_server.h"

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

#ifdef TEST_KOBUKI
	CKobukiRobotConfiguration cfg;
	CKobukiRobot robot;
#else
	CDummyRobot robot;
	CRobotConfiguration cfg;
#endif

#ifdef WITH_SICK
	CSickLaser * sick = new CSickLaser();
	sick->initialize("192.168.80.2");
	robot.registerLaserSensor(sick);
#else
	if (withLaser) {
		CDummyLaser * laser = new CDummyLaser();
		laser->initialize(NULL);
		robot.registerLaserSensor(laser);
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
