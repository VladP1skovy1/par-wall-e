/*
 * File name: laser_sensor.cc
 * Date:      2018/09/04 14:52
 * Author:    Jan Chudoba
 */

#include <stdlib.h>

#include "robot/laser_sensor.h"

CLaserSensorInterface::CLaserSensorInterface() :
	lastScanFresh(false),
	laserScanCallback(NULL)
{
	pthread_mutex_init(&dataMutex, 0);
}

CLaserSensorInterface::~CLaserSensorInterface()
{
	pthread_mutex_destroy(&dataMutex);
}

void CLaserSensorInterface::lock()
{
	pthread_mutex_lock(&dataMutex);
}

void CLaserSensorInterface::unlock()
{
	pthread_mutex_unlock(&dataMutex);
}

void CLaserSensorInterface::callScanCallback(CLaserScan & scan)
{
	if (laserScanCallback) {
		laserScanCallback(laserScanCallbackContext, laserScanCallbackIndex, scan);
	}
}

/* end of laser_sensor.cc */
