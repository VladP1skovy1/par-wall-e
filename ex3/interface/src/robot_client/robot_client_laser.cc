/*
 * File name: robot_client_laser.cc
 * Date:      2018/09/18 12:22
 * Author:    Jan Chudoba
 */

#include "robot_client/robot_client.h"

void CRobotClientLaser::updateLaserConfiguration(const robot_messages::LaserCfgMsg & cfgMsg)
{
	lock();
	configuration.valid = true;
	configuration.maxRange = cfgMsg.maxrange();
	configuration.angularResolution = cfgMsg.angularresolution();
	configuration.minAngle = cfgMsg.minangle();
	configuration.scanSampleCount = cfgMsg.scansamplecount();
	configuration.frequency = cfgMsg.frequency();
	configuration.x = cfgMsg.position().x();
	configuration.y = cfgMsg.position().y();
	configuration.z = cfgMsg.position().z();
	unlock();
}

void CRobotClientLaser::updateLaserScan(const robot_messages::LaserScanMsg & scanMsg)
{
	lock();
	lastScan.range.resize(scanMsg.range_size());
	int intSize = (scanMsg.range_size() == scanMsg.intensity_size()) ? scanMsg.intensity_size() : 0;
	lastScan.intensity.resize(intSize);
	for (int i=0; i<scanMsg.range_size(); i++) {
		lastScan.range[i] = scanMsg.range(i);
		if (intSize != 0) {
			lastScan.intensity[i] = scanMsg.intensity(i);
		}
	}
	lastScan.valid = true;
	lastScanFresh = true;
	unlock();

	callScanCallback(lastScan);
}


/* end of robot_client_laser.cc */
