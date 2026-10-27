/*
 * File name: dummy_robot.h
 * Date:      2018/09/17 15:32
 * Author:    Jan Chudoba
 */

#ifndef __DUMMY_ROBOT_H__
#define __DUMMY_ROBOT_H__

#include <pthread.h>

#include "robot/robot_interface.h"

class CDummyRobot : public CRobot
{
public:
	CDummyRobot();
	virtual ~CDummyRobot();

	virtual bool initialize(CRobotConfiguration * config = NULL);

	virtual bool setVelocities(double forwardVelocity, double angularVelocity);

	virtual bool getOdometry(CPose & odoPose);

	virtual double getBatteryLevel();
	virtual uint8_t getFrontBumperState();
	virtual bool getChargerState();
	virtual bool getDockingSensorData(std::vector<uint8_t> & dockingData);
	virtual bool getMotorCurrents(std::vector<double> & currents);

protected:
	virtual int robotProvidesDataInternal(TRobotDataType dataType);

private:
	static void * thread_body(void * arg) { ((CDummyRobot*)arg)->threadBody(); return 0; }
	void threadBody();

	pthread_t thread;
	bool threadInterrupt;

};


#endif

/* end of dummy_robot.h */
