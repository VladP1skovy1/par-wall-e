/*
 * File name: dummy_laser.h
 * Date:      2018/10/04 14:53
 * Author:    Jan Chudoba
 */

#ifndef __DUMMY_LASER_H__
#define __DUMMY_LASER_H__

#include <pthread.h>
#include "robot/laser_sensor.h"

class CDummyLaser : public CLaserSensorInterface
{
public:
	CDummyLaser();
	virtual ~CDummyLaser();

	virtual bool initialize(const char * deviceName);

protected:
	static void * thread_body(void * arg) { ((CDummyLaser*)arg)->threadBody(); return NULL; }
	virtual void threadBody();

private:
	pthread_t thread;
	bool threadRunning;
	bool threadShouldInterrupt;
};


#endif

/* end of dummy_laser.h */
