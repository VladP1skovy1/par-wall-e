/*
 * File name: laser_sick.cc
 * Date:      2018/09/06 17:52
 * Author:    Jan Chudoba
 */

#include <unistd.h>
#include <math.h>

#include "sick/laser_sick.h"

CSickLaser::CSickLaser() :
	upsidedown(false),
	threadRunning(false)
{
}

CSickLaser::~CSickLaser()
{
	threadShouldInterrupt = true;
	if (threadRunning) {
		pthread_join(thread, NULL);
		threadRunning = false;
	}
}

bool CSickLaser::initialize(const char * deviceName)
{
	ipAddress = deviceName;
	ipPort = 0;
	// TODO: [low] store / parse device name in form host:port
	
	threadShouldInterrupt = false;
	threadRunning = (pthread_create(&thread, 0, thread_body, this) == 0);
	if (!threadRunning) {
		fprintf(stderr, "CSickLaser ERROR: failed to start reading thread\n");
		return false;
	}
	return true;
}

#define DEG2RAD(x) ((x)*M_PI/180.0)
#define RAD2DEG(x) ((x)*180.0/M_PI)

void CSickLaser::threadBody()
{
	fprintf(stderr, "CSickLaser DEBUG: thread started\n");

	while (!threadShouldInterrupt) {

		//laserStatus = LASER_STATUS_INITIALIZING;

		// initialize
		if (ipPort > 0) {
			lms1xx.connect(ipAddress, ipPort);
		} else {
			lms1xx.connect(ipAddress);
		}

		if (!lms1xx.isConnected()) {
			if (threadShouldInterrupt) break;
			const int reconnectDelay = 5; // [s]
			fprintf(stderr, "CSickLaser ERROR: failed to connect to laser ip=%s, trying reconnect in %d seconds\n", ipAddress.c_str(), reconnectDelay);
			//laserStatus = LASER_STATUS_NOT_PRESENT;
			//return;
			usleep(reconnectDelay*1000*1000);
			fprintf(stderr, "CSickLaser DEBUG: reconnecting to laser ip=%s ...\n", ipAddress.c_str());
			continue;
		} else {
			fprintf(stderr, "CSickLaser DEBUG: connected to laser ip=%s\n", ipAddress.c_str());
		}

		double minAngle;
		double maxAngle;
		double angleIncrement;

		lms1xx::scanCfg laserCfg = lms1xx.getScanCfg();
		fprintf(stderr, "CSickLaser DEBUG: laser scanning frequency %.2f Hz\n", 0.01 * (double)laserCfg.scaningFrequency);
		minAngle = DEG2RAD((double)laserCfg.startAngle / 10000.0);
		maxAngle = DEG2RAD((double)laserCfg.stopAngle / 10000.0);
		angleIncrement = DEG2RAD((double)laserCfg.angleResolution / 10000.0);
		fprintf(stderr, "CSickLaser DEBUG: laser angle resolution %.3f deg\n", RAD2DEG(angleIncrement));
		fprintf(stderr, "CSickLaser DEBUG: laser min angle %.3f deg\n", RAD2DEG(minAngle));
		fprintf(stderr, "CSickLaser DEBUG: laser max angle %.3f deg\n", RAD2DEG(maxAngle));

		int expectedSampleCount = 1 + (int)round((maxAngle - minAngle) / angleIncrement);
		fprintf(stderr, "CSickLaser DEBUG: expected samples per scan: %d\n", expectedSampleCount);

		lms1xx.startMeas();
		lms1xx::status_t lmsStatus = lms1xx.queryStatus();
		const char * strLaserStatus = "invalid";
		switch (lmsStatus) {
			case lms1xx::undefined: strLaserStatus = "undefined"; break;
			case lms1xx::initialisation: strLaserStatus = "initialisation"; break;
			case lms1xx::configuration: strLaserStatus = "configuration"; break;
			case lms1xx::idle: strLaserStatus = "idle"; break;
			case lms1xx::rotated: strLaserStatus = "rotated"; break;
			case lms1xx::in_preparation: strLaserStatus = "in_preparation"; break;
			case lms1xx::ready: strLaserStatus = "ready"; break;
			case lms1xx::ready_for_measurement: strLaserStatus = "ready_for_measurement"; break;
		}
		fprintf(stderr, "CSickLaser DEBUG: laser status: %s\n", strLaserStatus);

		lms1xx.scanContinous(1);

		fprintf(stderr, "CSickLaser DEBUG: laser initialized\n");

		//laserStatus = LASER_STATUS_INITIALIZED;

		//int pollutionReportCount = 0;

		// data reading loop
		while (!threadShouldInterrupt) {
			lms1xx::scanData scanData;
			if (lms1xx.getData(scanData)) {
				//int pollutionErrorCount = 0;
				if (scanData.dist_len1 > 0) {
					//laserStatus = LASER_STATUS_OK;
					
					if (!configuration.valid) {
						configuration.maxRange = -1; // TODO assign range by sensor type
						configuration.minAngle = minAngle;
						configuration.angularResolution = angleIncrement;
						configuration.scanSampleCount = expectedSampleCount;
						configuration.frequency = laserCfg.scaningFrequency;
						configuration.valid = true;
					}

					CLaserScan scan;
					for (int i=0; i<scanData.dist_len1; i++) {
						int isrc = (upsidedown==false) ? i : (scanData.dist_len1 - 1 - i);
						int iRange = scanData.dist1[isrc];
						scan.range.push_back(0.001 * (double)iRange); // TODO: range resolution units
						//fprintf(stderr, " %d", iRange);
					}
					scan.valid = true;

					lock();
					lastScan = scan;
					lastScanFresh = true;
					unlock();

					callScanCallback(scan);

					/*
					CDataLaser laserData;
					laserData.timestamp = getTimeMs();
					laserData.minAngle = minAngle;
					laserData.maxAngle = maxAngle;
					laserData.angleIncrement = angleIncrement;
					laserData.laserId = laserId;
					if (scanData.dist_len1 != expectedSampleCount) {
						double angleRange = angleIncrement * (scanData.dist_len1 - 1);
						laserData.minAngle = - angleRange / 2.0; // TODO: this expects that samples are centered!
						laserData.maxAngle = + angleRange / 2.0;
						//fprintf(stderr, "CSickLaser WARNING: laser '%s' read %d samples in scan, while expecting %d !!! fix angle to range <%.1f; %.1f>\n", laserId, scanData.dist_len1, expectedSampleCount, RAD2DEG(laserData.minAngle), RAD2DEG(laserData.maxAngle));
					}
					//fprintf(stderr, "raw laser scan:");
					std::string dbgPollution = "";
					for (int i=0; i<scanData.dist_len1; i++) {
						int isrc = (upsidedown==false) ? i : (scanData.dist_len1 - 1 - i);
						int iRange = scanData.dist1[isrc];
						laserData.ranges.push_back(0.001 * (double)iRange); // TODO: range resolution units
						if (iRange < 16 && iRange > 0) { // TODO: LMS151 reports 28mm for dirt on glass (really??)
							// 0 = no meas. value detected, probably out of range, or object too dark
							// 1 = dazzled
							// 2 = implausible measurement
							// 3 = invalid value due to filter (echo/particle filter)
							// 4-15: reserved
							// >16 ... valid measurements
							//pollutionErrorCount++;
							dbgPollution += " [" + toStr(i) + "]=" + toStr(iRange);
						}
						//fprintf(stderr, " %d", iRange);
					}
					//fprintf(stderr, "\n");
					//
					// send data to system thru sending thread
					sendData(laserData);
					*/

					/*
					switch (scanData.status) {
						case lms1xx::SCANNER_STATUS_OK:
							laserStatus = LASER_STATUS_OK;
							break;
						case lms1xx::SCANNER_STATUS_POLLUTION_WARNING:
						case lms1xx::SCANNER_STATUS_POLLUTION_ERROR:
							laserStatus = LASER_STATUS_WARNING;
							break;
						case lms1xx::SCANNER_STATUS_ERROR:
							laserStatus = LASER_STATUS_ERROR;
							break;
						case lms1xx::SCANNER_STATUS_NUMBER:
						case lms1xx::SCANNER_STATUS_UNKNOWN:
							laserStatus = LASER_STATUS_ERROR;
							break;
					}
					*/
				}
				/*
				else if (laserStatus != LASER_STATUS_ERROR) {
					laserStatus = LASER_STATUS_ERROR;
				}
				*/
			} else {
				//laserStatus = LASER_STATUS_ERROR;
				fprintf(stderr, "CSickLaser WARNING: failed to get data\n");

				if (!lms1xx.isConnected()) {
					fprintf(stderr, "CSickLaser WARNING: reconnecting ...\n");
					// interrupt data read loop
					break;
				} else {
				}
			}
		} // while ! threadShouldInterrupt - end of data reading loop

		if (lms1xx.isConnected()) {
			lms1xx.scanContinous(0);
			lms1xx.stopMeas();
			lms1xx.disconnect();
		}
	}

	fprintf(stderr, "CSickLaser WARNING: thread interrupted\n");
}



/* end of laser_sick.cc */
