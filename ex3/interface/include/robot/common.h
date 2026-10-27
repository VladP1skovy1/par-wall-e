/*
 * File name: common.h
 * Date:      2018/09/19 15:32
 * Author:    Jan Chudoba
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <pthread.h>

class CMutex
{
public:
	CMutex() { pthread_mutex_init(&mutex, 0); }
	~CMutex() { pthread_mutex_destroy(&mutex); }
	void lock() { pthread_mutex_lock(&mutex); }
	void unlock() { pthread_mutex_unlock(&mutex); }
private:
	pthread_mutex_t mutex;
};

class CEvent
{
public:
	CEvent() { pthread_mutex_init(&mutex, 0); pthread_cond_init(&cond, 0); }
	~CEvent() { pthread_cond_destroy(&cond), pthread_mutex_destroy(&mutex); }
	void lock() { pthread_mutex_lock(&mutex); }
	void unlock() { pthread_mutex_unlock(&mutex); }
	void wait() { lock(); waitNoLock(); unlock(); }
	void waitNoLock() { pthread_cond_wait(&cond, &mutex); }
	void notify() { lock(); notifyNoLock(); unlock(); }
	void notifyNoLock() { pthread_cond_signal(&cond); }
	void wait(int timeout_ms) { lock(); waitNoLock(timeout_ms); unlock(); }
	void waitNoLock(int timeout_ms);
private:
	pthread_mutex_t mutex;
	pthread_cond_t cond;
};

class CClock
{
public:
	CClock() { start(); firstStartTime = startTime; }
	void start() { startTime = getTimeMs(); }
	uint32_t getTimeDifMs() { return (getTimeMs() - startTime); }
	uint32_t getTotalTimeDifMs() { return (getTimeMs() - firstStartTime); }
	static uint32_t getTimeMs();
private:
	uint32_t startTime;
	uint32_t firstStartTime;
};

#endif

/* end of common.h */
