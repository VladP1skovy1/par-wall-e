/*
 * File name: joy_drive.cc
 * Date:      2018/09/26 15:04
 * Author:    Jan Chudoba
 */

#include <stdio.h>

#include "robot_client/robot_client.h"
#include "joystick.h"

CJoystick joystick;

CRobotClient client;

void joystickCallback(CJoystick * joystick)
{
#if 0
	fprintf(stderr, "%5d %5d %5d %5d buttons",
		joystick->axis(0),
		joystick->axis(1),
		joystick->axis(2),
		joystick->axis(3)
	);
	for (int i = 0; i < joystick->getButtonCount(); i++) {
		fprintf(stderr, " %d", joystick->buttonPressed(i));
	}
	fprintf(stderr, "\n");
	if (joystick->getAxisCount() > 4) {
		for(int i = 4; i < joystick->getAxisCount(); i++) {
			fprintf(stderr, " %5d", joystick->axis(i));
		}
		fprintf(stderr, "\n");
	}
#endif

	double forward = 0.2 * (double)joystick->axis(3) / 0x8000;
	double angular = 0.2 * (double)joystick->axis(2) / 0x8000;

	fprintf(stderr, "setVelocities(%.3f, %.3f)\n", forward, angular);
	client.setVelocities(forward, angular);
}


int main(int argc, char ** argv)
{
	fprintf(stderr, "joystick drive test\n");

	const char * joystickDevice;

	int a = 1;
	if (a < argc) joystickDevice = argv[a];
	else joystickDevice = NULL;

	if (!joystick.initialize(joystickDevice)) return 1;

	fprintf(stderr, "Joystick initialized\n");
	
	joystick.registerJoystickCallback(joystickCallback);

	CRobotClientConfiguration cfg;
	//cfg.robotName = "turtlebot";
	if (!client.initialize(&cfg)) {
		fprintf(stderr, "Failed to initialize robot\n");
		return 1;
	}
	fprintf(stderr, "Robot initialized\n");

	fprintf(stderr, "Press ENTER to stop\n");
	getchar();
	fprintf(stderr, "DONE\n");

	return 0;
}

/* end of joy_drive.cc */
