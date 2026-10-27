/*
 * File name: tdock_drive.cc
 * Date:      2018/09/11 16:37
 * Author:    Jan Chudoba
 */

#include <stdio.h>

#include "robot/robot_interface.h"
#include "kobuki/kobuki_robot.h"
#include "robot_client/robot_client.h"

#include "kobuki_dock_drive/dock_drive.h"

#define PERIOD_MS 100

int main(int argc, char ** argv)
{
	fprintf(stderr, "kobuki dock demo\n");

#if 0
	CKobukiRobotConfiguration cfg;
	CKobukiRobot robot;
	robot.initialize(&cfg);
#else
	CRobotClient robot;
	CRobotClientConfiguration cfg;
	robot.initialize(&cfg);
#endif

	// TODO: detect successful connection

	fprintf(stderr, "battery %d%%\n", (int) (100*robot.getBatteryLevel()));

	DockDrive dockDrive;

	while (true) {

		CPose pose;
		robot.getOdometry(pose);
		//fprintf(stderr, "odometry: %.3f, %.3f, %.3f\n", pose.x, pose.y, pose.heading);

		bool charger = robot.getChargerState();
		uint8_t bumper = robot.getFrontBumperState();
		if (charger) {
			fprintf(stderr, "INFO: charger detected\n");
		}
		if (bumper != 0) {
			fprintf(stderr, "INFO: bumper %1X\n", bumper);
		}

		std::vector<uint8_t> dockSensor;
		if (robot.getDockingSensorData(dockSensor)) {
			fprintf(stderr, "DOCK %4u %4u %4u\n", dockSensor[0], dockSensor[1], dockSensor[2]);
		} else {
			fprintf(stderr, "ERROR: failed to get dock data\n");
		}

		dockDrive.update(dockSensor, bumper, charger, pose);
		fprintf(stderr, "DEBUG: %s\n", dockDrive.getDebugStream().c_str());

		// set velocities
		robot.setVelocities(dockDrive.getVX(), dockDrive.getWZ());

		if (dockDrive.getState() == RobotDockingState::DOCKED_IN ||
			dockDrive.getState() == RobotDockingState::BUMPED_DOCK
		) {
			break;
		}

		usleep(PERIOD_MS * 1000);

		fprintf(stderr, "battery %d%%\n", (int) (100*robot.getBatteryLevel()));
	}

	fprintf(stderr, "stop\n");
	robot.setVelocities(0,0);

	return 0;
}

/* end of tdock_drive.cc */
