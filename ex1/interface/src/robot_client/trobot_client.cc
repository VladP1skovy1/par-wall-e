/*
 * File name: trobot_client.cc
 * Date:      2018/09/18 09:28
 * Author:    Jan Chudoba
 */

#include <stdio.h>
#include <unistd.h>

#include "robot_client/robot_client.h"

int main(int argc, char ** argv)
{
	fprintf(stderr, "PAR Robot client test\n");

	bool async = false;
	bool reqresp = false;

	int a = 1;
	while (a < argc) {
		if (argv[a][0] != '-') break;
		switch (argv[a][1]) {
		case 'a':
			async = true;
			break;
		case 'r':
			reqresp = true;
			break;
		default:
			return 1;
		}
		a++;
	}
	
	CRobotClientConfiguration cfg;
	//cfg.robotName = "turtlebot";
	cfg.waitForInitialization = true;
	
	CRobotClient client;
	if (async) {
		if (reqresp) {
			fprintf(stderr, "ERROR: async and req-resp modes are mutually exclusive!\n");
			return 3;
		}
		fprintf(stderr, "Enable asynchronous mode\n");
		usleep(500000);
		client.setAsyncMode();
	}
	if (reqresp) {
		fprintf(stderr, "Enable request-response service usage\n");
		usleep(500000);
		client.enableRequestResponseServices();
	}

	if (!client.initialize(&cfg)) {
		fprintf(stderr, "Failed to initialize robot\n");
		return 1;
	}

	fprintf(stderr, "Robot initialized\n");

	fprintf(stderr, "Robot laser count %d\n", client.getLaserSensorCount());

	CLaserSensorConfiguration laserCfg;
	if (client.getLaserSensorConfiguration(laserCfg)) {
		fprintf(stderr, "LASER CFG OK: %d samples, angular range %.1f deg\n", laserCfg.scanSampleCount, (laserCfg.scanSampleCount-1) * laserCfg.angularResolution * 180.0 / M_PI);
	} else {
		fprintf(stderr, "Failed to get laser configuration from robot\n");
	}

	usleep(1000000);

	fprintf(stderr, "Send control command ...\n");
	if (client.setVelocities(0.1, 0.01)) {
		fprintf(stderr, "setVelocities OK\n");
	} else {
		fprintf(stderr, "setVelocities ERROR\n");
	}

	usleep(5000000);

	fprintf(stderr, "terminating test ...\n");

	return 0;
}

/* end of trobot_client.cc */
