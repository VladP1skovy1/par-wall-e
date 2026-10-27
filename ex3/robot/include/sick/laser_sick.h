/*
 * File name: laser_sick.h
 * Date:      2018/09/06 17:52
 * Author:    Jan Chudoba
 */

#ifndef __LASER_SICK_H__
#define __LASER_SICK_H__

#include <pthread.h>
#include <string>

#include "robot/laser_sensor.h"
#include "sick/LMS1xx.h"

class CSickLaser : public CLaserSensorInterface
{
public:
	CSickLaser();
	virtual ~CSickLaser();

	virtual bool initialize(const char * deviceName);

protected:
	static void * thread_body(void * arg) { ((CSickLaser*)arg)->threadBody(); return NULL; }
	virtual void threadBody();

private:
	lms1xx::LMS1xx lms1xx;
	std::string ipAddress;
	int ipPort;
	bool upsidedown;

	pthread_t thread;
	bool threadRunning;
	bool threadShouldInterrupt;
};

#endif

/* end of laser_sick.h */
