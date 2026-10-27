/*
 * File name: laser_sensor.h
 * Date:      2018/09/04 14:34
 * Author:    Jan Chudoba
 */

#ifndef __LASER_SENSOR_H__
#define __LASER_SENSOR_H__

#include <pthread.h>
#include <vector>

class CLaserSensorConfiguration
{
public:
	bool valid;
	double maxRange;
	double angularResolution;
	double minAngle;
	int scanSampleCount;
	double frequency;
	double x;
	double y;
	double z;

	CLaserSensorConfiguration() : valid(false) {}
};

class CLaserScan
{
public:
	bool valid;
	std::vector<double> range;
	std::vector<double> intensity;

	CLaserScan() : valid(false) {}
};

typedef void (*TLaserScanCallback)(void*, int, CLaserScan &);

class CLaserSensorInterface
{
public:
	CLaserSensorInterface();
	virtual ~CLaserSensorInterface();

	void setDataCallback(TLaserScanCallback cb, void * context = NULL, int index = 0) { laserScanCallback = cb; laserScanCallbackContext = context; }

	virtual bool initialize(const char * deviceName) { return false; };
	bool getConfiguration(CLaserSensorConfiguration & cfg) { lock(); cfg = configuration; unlock(); return cfg.valid; }
	virtual bool isFresh() { return lastScanFresh; }
	bool getLastScan(CLaserScan & scan) { lock(); scan = lastScan; bool result = scan.valid; lastScanFresh = false; unlock(); return result; }
	bool isConfigurationValid() { return configuration.valid; }

protected:
	void lock();
	void unlock();

	void callScanCallback(CLaserScan & scan);

protected:
	pthread_mutex_t dataMutex;
	CLaserSensorConfiguration configuration;
	CLaserScan lastScan;
	bool lastScanFresh;
	TLaserScanCallback laserScanCallback;
	void * laserScanCallbackContext;
	int laserScanCallbackIndex;
};

#endif

/* end of laser_sensor.h */
