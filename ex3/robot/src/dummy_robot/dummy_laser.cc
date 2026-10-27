/*
 * File name: dummy_laser.cc
 * Date:      2018/10/04 14:53
 * Author:    Jan Chudoba
 */

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "dummy_robot/dummy_laser.h"

CDummyLaser::CDummyLaser() :
	threadRunning(false)
{
}

CDummyLaser::~CDummyLaser()
{
	threadShouldInterrupt = true;
	if (threadRunning) {
		pthread_join(thread, NULL);
		threadRunning = false;
	}
}

bool CDummyLaser::initialize(const char * deviceName)
{
	configuration.maxRange = 10.0;
	configuration.angularResolution = M_PI/180.0;
	configuration.minAngle = -M_PI / 2;
	configuration.scanSampleCount = 181;
	configuration.frequency = 1.0;
	configuration.x = 0.01;
	configuration.y = 0;
	configuration.z = 0.1;
	configuration.valid = true;

	threadShouldInterrupt = false;
	threadRunning = (pthread_create(&thread, 0, thread_body, this) == 0);
	if (!threadRunning) {
		fprintf(stderr, "CDummyLaser ERROR: failed to start reading thread\n");
		return false;
	}
	return true;
}

void CDummyLaser::threadBody()
{
	fprintf(stderr, "CDummyLaser DEBUG: thread started\n");
	while (!threadShouldInterrupt) {
		usleep(1000000);

		CLaserScan newScan;
		for (int i = 0; i < configuration.scanSampleCount; i++) {
			newScan.range.push_back(1.0 + (i % 20) * 0.05);
		}
		newScan.valid = true;
		lock();
		lastScan = newScan;
		lastScanFresh = true;
		unlock();
		callScanCallback(newScan);
	}
}



/* end of dummy_laser.cc */
