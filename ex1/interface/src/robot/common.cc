/*
 * File name: common.cc
 * Date:      2018/09/19 15:38
 * Author:    Jan Chudoba
 */

#include <sys/time.h>

#include "robot/common.h"

void CEvent::waitNoLock(int timeout_ms)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	int msec = ts.tv_nsec / 1000000 + timeout_ms;
	if (msec < 1000) {
		ts.tv_nsec = 1000000 * msec;
	} else {
		int sec = msec / 1000;
		msec -= sec * 1000;
		ts.tv_nsec = msec * 1000000;
		ts.tv_sec += sec;
	}
	pthread_cond_timedwait(&cond, &mutex, &ts);
}

uint32_t CClock::getTimeMs()
{
	uint32_t t;
	struct timeval now;
	gettimeofday(&now, NULL);
	t = now.tv_sec * 1000 + now.tv_usec / 1000;
	return t;
}


/* end of common.cc */
