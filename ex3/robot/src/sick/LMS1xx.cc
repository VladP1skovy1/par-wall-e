/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************
 *   Modified for imrnav2 by Jan Chudoba
 ***************************************************************************/

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "sick/LMS1xx.h"

using namespace lms1xx;

#define SICK_DATA_MAX_DELAY_MS 5000

#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_DEBUG 3

static int debugLevel = LOG_LEVEL_INFO;

LMS1xx::LMS1xx() :
	connected(false),
	sockDesc(-1)
{
}

LMS1xx::~LMS1xx()
{
	if (sockDesc >= 0) {
		::shutdown(sockDesc, SHUT_RDWR);
	}
	disconnect();
	if (sockDesc >= 0) {
		::close(sockDesc);
	}
}

void LMS1xx::connect(std::string host, int port) {
	if (!connected) {
		if (sockDesc < 0) {
			sockDesc = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
		}
		if (sockDesc >= 0) {
			struct sockaddr_in stSockAddr;
			stSockAddr.sin_family = PF_INET;
			stSockAddr.sin_port = htons(port);
			inet_pton(AF_INET, host.c_str(), &stSockAddr.sin_addr);
			int ret = ::connect(sockDesc, (struct sockaddr *) &stSockAddr, sizeof stSockAddr);
			if (ret == 0) {
				connected = true;
			}
		}
	}
}

void LMS1xx::disconnect() {
	if (connected) {
		close(sockDesc);
		connected = false;
		fprintf(stderr, "LMS1xx WARNING: socket %d closed\n", sockDesc);
		sockDesc = -1;
	}
}

bool LMS1xx::isConnected()
{
	return connected;
}

void LMS1xx::startMeas()
{
	char buf[100];
	sprintf(buf, "%c%s%c", 0x02, "sMN LMCstartmeas", 0x03);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	int len = read(sockDesc, buf, 100);
	if (buf[0] != 0x02)
		fprintf(stderr, "LMS1xx WARNING: invalid packet recieved\n");
	buf[len] = 0;
	if (debugLevel>=LOG_LEVEL_DEBUG) fprintf(stderr, "LMS1xx DEBUG: RX: %s\n", buf);
}

void LMS1xx::stopMeas()
{
	char buf[100];
	sprintf(buf, "%c%s%c", 0x02, "sMN LMCstopmeas", 0x03);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	int len = read(sockDesc, buf, 100);
	if (buf[0] != 0x02)
		fprintf(stderr, "LMS1xx WARNING: invalid packet recieved\n");
	buf[len] = 0;
	if (debugLevel>=LOG_LEVEL_DEBUG) fprintf(stderr, "LMS1xx DEBUG: RX: %s\n", buf);
}

status_t LMS1xx::queryStatus()
{
	char buf[100];
	sprintf(buf, "%c%s%c", 0x02, "sRN STlms", 0x03);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	int len = read(sockDesc, buf, 100);
	if (buf[0] != 0x02)
		fprintf(stderr, "LMS1xx WARNING: invalid packet recieved\n");
	buf[len] = 0;
	if (debugLevel>=LOG_LEVEL_DEBUG) fprintf(stderr, "LMS1xx DEBUG: RX: %s\n", buf);

	int ret;
	sscanf((buf + 10), "%d", &ret);

	return (status_t) ret;
}

void LMS1xx::login()
{
	char buf[100];
	int result;
	sprintf(buf, "%c%s%c", 0x02, "sMN SetAccessMode 03 F4724744", 0x03);
	
	fd_set readset;
	struct timeval timeout;


	do { //loop until data is available to read
	  timeout.tv_sec = 1;
	  timeout.tv_usec = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	  write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	  FD_ZERO(&readset);
	  FD_SET(sockDesc, &readset);
	  result = select(sockDesc + 1, &readset, NULL, NULL, &timeout);

	} while (result <= 0);

	int len = read(sockDesc, buf, 100);
	if (buf[0] != 0x02)
		fprintf(stderr, "LMS1xx WARNING: invalid packet recieved\n");
	buf[len] = 0;
	if (debugLevel>=LOG_LEVEL_DEBUG) fprintf(stderr, "LMS1xx DEBUG: RX: %s\n", buf);
}

scanCfg LMS1xx::getScanCfg() const
{
	scanCfg cfg;
	char buf[100];
	sprintf(buf, "%c%s%c", 0x02, "sRN LMPscancfg", 0x03);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	int len = read(sockDesc, buf, 100);
	if (buf[0] != 0x02)
		fprintf(stderr, "LMS1xx WARNING: invalid packet recieved\n");
	buf[len] = 0;
	if (debugLevel>=LOG_LEVEL_DEBUG) fprintf(stderr, "LMS1xx DEBUG: RX: %s\n", buf);

	sscanf(buf + 1, "%*s %*s %X %*d %X %X %X", &cfg.scaningFrequency, &cfg.angleResolution, &cfg.startAngle, &cfg.stopAngle);

	// internal range is -45 - 225, convert it to -135 - 135 deg.
	cfg.startAngle -= 90*10000;
	cfg.stopAngle -= 90*10000;
	return cfg;
}

void LMS1xx::setScanCfg(const scanCfg &cfg) {
	char buf[100];
	sprintf(buf, "%c%s %X +1 %X %X %X%c", 0x02, "sMN mLMPsetscancfg",
			cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle,
			cfg.stopAngle, 0x03);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	int len = read(sockDesc, buf, 100);

	buf[len - 1] = 0;
}

void LMS1xx::setScanDataCfg(const scanDataCfg &cfg)
{
#pragma GCC diagnostic pop
	char buf[100];
	sprintf(buf, "%c%s %02X 00 %d %d 0 %02X 00 %d %d 0 %d +%d%c", 0x02,
			"sWN LMDscandatacfg", cfg.outputChannel, cfg.remission ? 1 : 0,
			cfg.resolution, cfg.encoder, cfg.position ? 1 : 0,
			cfg.deviceName ? 1 : 0, cfg.timestamp ? 1 : 0, cfg.outputInterval, 0x03);
	if (debugLevel>=LOG_LEVEL_DEBUG) fprintf(stderr, "LMS1xx DEBUG: TX: %s\n", buf);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	int len = read(sockDesc, buf, 100);
	buf[len - 1] = 0;
}

scanOutputRange LMS1xx::getScanOutputRange() const
{
	scanOutputRange outputRange;
	char buf[100];
	sprintf(buf, "%c%s%c", 0x02, "sRN LMPoutputRange", 0x03);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	int len __attribute__((unused));
	len = read(sockDesc, buf, 100);

	sscanf(buf + 1, "%*s %*s %*d %X %X %X", &outputRange.angleResolution, &outputRange.startAngle, &outputRange.stopAngle);
	return outputRange;
}

void LMS1xx::scanContinous(int start)
{
	char buf[100];
	sprintf(buf, "%c%s %d%c", 0x02, "sEN LMDscandata", start, 0x03);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	int len = read(sockDesc, buf, 100);

	if (buf[0] != 0x02)
		fprintf(stderr, "LMS1xx ERROR: invalid packet recieved\n");

	buf[len] = 0;
	if (debugLevel>=LOG_LEVEL_DEBUG) fprintf(stderr, "LMS1xx DEBUG: RX: %s\n", buf);
}

bool LMS1xx::getData(scanData & data)
{
	char buf[20000];
	fd_set rfds;
	struct timeval tv;
	int retval, len;
	len = 0;

	data.status = SCANNER_STATUS_UNKNOWN;

	do {
		if (sockDesc == -1) return false;
		FD_ZERO(&rfds);
		FD_SET(sockDesc, &rfds);

		tv.tv_sec = 0;
		tv.tv_usec = 200000;
		retval = select(sockDesc + 1, &rfds, NULL, NULL, &tv);
		if (retval > 0) {
			int r = read(sockDesc, buf + len, 20000 - len);
			if (r > 0) {
				len += r;
			} else {
				if (r == 0) {
					fprintf(stderr, "LMS1xx WARNING: read returned %d (%s): END OF FILE!\n", r, strerror(errno));
					disconnect();
				} else {
					fprintf(stderr, "LMS1xx WARNING: read error %d: %s\n", r, strerror(errno));
				}
				return false;
			}
		} else {
			fprintf(stderr, "LMS1xx WARNING: select returned %d: %s\n", retval, strerror(errno));
		}
		// TODO: calculate total waiting time, when delay is too big, return error
	} while ((buf[0] != 0x02) || (buf[len - 1] != 0x03));

	// TODO: binary protocol parsing

	// if (debugLevel>=LOG_LEVEL_DEBUG) fprintf(stderr, "LMS1xx DEBUG: scan data recieved\n");
	buf[len - 1] = 0;

	if (!receivedScanPrinted) {
		receivedScanPrinted = true;
		fprintf(stderr, "LMS1xx DEBUG: received first scan data, message len=%d\n", len);
		fprintf(stderr, "LMS1xx::getData() message '");
		fputs(buf, stderr);
		fprintf(stderr, "\n");
	}

	// parsing of sSN LMDscandata message
	do {
		char * saveptr;
		char* tok = strtok_r(buf, " ", &saveptr); //Type of command ("sSN")
		tok = strtok_r(NULL, " ", &saveptr); //Command ("LMDscandata")
		tok = strtok_r(NULL, " ", &saveptr); //VersionNumber
		tok = strtok_r(NULL, " ", &saveptr); //DeviceNumber
		tok = strtok_r(NULL, " ", &saveptr); //Serial number
		tok = strtok_r(NULL, " ", &saveptr); //DeviceStatus
		if (tok) {
			int deviceStatus = atoi(tok);
			if (deviceStatus == 0) {
				data.status = SCANNER_STATUS_OK;
			} else {
				bool errClass = false;
				std::string errMsg = "";
				if ((deviceStatus & 0x02) != 0) {
					data.status = SCANNER_STATUS_POLLUTION_WARNING;
					errMsg += " pollution-warning";
				}
				if ((deviceStatus & 0x04) != 0) {
					data.status = SCANNER_STATUS_POLLUTION_ERROR;
					errMsg += " pollution-error";
					errClass = true;
				}
				if ((deviceStatus & 0x01) != 0) {
					data.status = SCANNER_STATUS_ERROR;
					errMsg += " error";
					errClass = true;
				}
				if ((deviceStatus & ~(int)0x07) != 0) {
					errMsg += " unknown-code";
				}
				if (errClass) {
					fprintf(stderr, "LMS1xx ERROR: laser device status %d: %s\n", deviceStatus, errMsg.c_str());
				} else {
					fprintf(stderr, "LMS1xx WARNING: laser device status %d: %s\n", deviceStatus, errMsg.c_str());
				}
			}
		} else {
			// can not parse status
			break;
		}
		tok = strtok_r(NULL, " ", &saveptr); //MessageCounter
		tok = strtok_r(NULL, " ", &saveptr); //ScanCounter
		tok = strtok_r(NULL, " ", &saveptr); //PowerUpDuration
		tok = strtok_r(NULL, " ", &saveptr); //TransmissionDuration
		tok = strtok_r(NULL, " ", &saveptr); //InputStatus
		tok = strtok_r(NULL, " ", &saveptr); //OutputStatus
		tok = strtok_r(NULL, " ", &saveptr); //ReservedByteA
		tok = strtok_r(NULL, " ", &saveptr); //ScanningFrequency
		tok = strtok_r(NULL, " ", &saveptr); //MeasurementFrequency
		tok = strtok_r(NULL, " ", &saveptr);
		tok = strtok_r(NULL, " ", &saveptr);
		tok = strtok_r(NULL, " ", &saveptr);
		tok = strtok_r(NULL, " ", &saveptr); //NumberEncoders
		if (tok == NULL) break;
		int NumberEncoders;
		sscanf(tok, "%d", &NumberEncoders);
		for (int i = 0; i < NumberEncoders; i++) {
			tok = strtok_r(NULL, " ", &saveptr); //EncoderPosition
			tok = strtok_r(NULL, " ", &saveptr); //EncoderSpeed
		}

		tok = strtok_r(NULL, " ", &saveptr); //NumberChannels16Bit
		int NumberChannels16Bit;
		if (tok == NULL) break;
		sscanf(tok, "%d", &NumberChannels16Bit);
		//fprintf(stderr, "LMS1xx DEBUG: NumberChannels16Bit : %d\n", NumberChannels16Bit);

		for (int i = 0; i < NumberChannels16Bit; i++) {
			char content[6];
			int * dataLengthPtr = NULL;
			uint16_t * dataArray = NULL;
			tok = strtok_r(NULL, " ", &saveptr); //MeasuredDataContent
			if (tok == NULL) break;
			sscanf(tok, "%s", content);
			if (!strcmp(content, "DIST1")) {
				dataLengthPtr = &data.dist_len1;
				dataArray = data.dist1;
			} else if (!strcmp(content, "DIST2")) {
				dataLengthPtr = &data.dist_len2;
				dataArray = data.dist2;
			} else if (!strcmp(content, "RSSI1")) {
				dataLengthPtr = &data.rssi_len1;
				dataArray = data.rssi1;
			} else if (!strcmp(content, "RSSI2")) {
				dataLengthPtr = &data.rssi_len2;
				dataArray = data.rssi2;
			} else {
				// invalid/unknown data type
			}
			tok = strtok_r(NULL, " ", &saveptr); //ScalingFactor
			tok = strtok_r(NULL, " ", &saveptr); //ScalingOffset
			tok = strtok_r(NULL, " ", &saveptr); //Starting angle
			tok = strtok_r(NULL, " ", &saveptr); //Angular step width
			tok = strtok_r(NULL, " ", &saveptr); //NumberData
			// TODO: these data should be reported to containing module and checked if they are correct
			int NumberData;
			if (tok == NULL) break;
			sscanf(tok, "%X", &NumberData);
			//fprintf(stderr, "LMS1xx DEBUG: NumberData : %d\n", NumberData);

			if (dataLengthPtr) {
				*dataLengthPtr = NumberData;
			}

			for (int i = 0; i < NumberData; i++) {
				int dat;
				tok = strtok_r(NULL, " ", &saveptr); //data
				if (tok == NULL) break;
				sscanf(tok, "%X", &dat);

				if (dataArray) dataArray[i] = dat;
			}
		}

		tok = strtok_r(NULL, " ", &saveptr); //NumberChannels8Bit
		int NumberChannels8Bit;
		if (tok == NULL) break;
		sscanf(tok, "%d", &NumberChannels8Bit);
		//fprintf(stderr, "LMS1xx DEBUG: NumberChannels8Bit : %d\n", NumberChannels8Bit);

		for (int i = 0; i < NumberChannels8Bit; i++) {
			int type = -1;
			char content[6];
			tok = strtok_r(NULL, " ", &saveptr); //MeasuredDataContent
			if (tok == NULL) break;
			sscanf(tok, "%s", content);
			if (!strcmp(content, "DIST1")) {
				type = 0;
			} else if (!strcmp(content, "DIST2")) {
				type = 1;
			} else if (!strcmp(content, "RSSI1")) {
				type = 2;
			} else if (!strcmp(content, "RSSI2")) {
				type = 3;
			}
			tok = strtok_r(NULL, " ", &saveptr); //ScalingFactor
			tok = strtok_r(NULL, " ", &saveptr); //ScalingOffset
			tok = strtok_r(NULL, " ", &saveptr); //Starting angle
			tok = strtok_r(NULL, " ", &saveptr); //Angular step width
			tok = strtok_r(NULL, " ", &saveptr); //NumberData
			int NumberData;
			if (tok == NULL) break;
			sscanf(tok, "%X", &NumberData);
			//fprintf(stderr, "LMS1xx DEBUG: NumberData : %d\n", NumberData);

			if (type == 0) {
				data.dist_len1 = NumberData;
			} else if (type == 1) {
				data.dist_len2 = NumberData;
			} else if (type == 2) {
				data.rssi_len1 = NumberData;
			} else if (type == 3) {
				data.rssi_len2 = NumberData;
			}
			for (int i = 0; i < NumberData; i++) {
				int dat;
				tok = strtok_r(NULL, " ", &saveptr); //data
				if (tok == NULL) break;
				sscanf(tok, "%X", &dat);

				if (type == 0) {
					data.dist1[i] = dat;
				} else if (type == 1) {
					data.dist2[i] = dat;
				} else if (type == 2) {
					data.rssi1[i] = dat;
				} else if (type == 3) {
					data.rssi2[i] = dat;
				}
			}
			if (tok == NULL) break;
		}
		if (tok == NULL) break;
		return true;
	} while (false);
	fprintf(stderr, "LMS1xx ERROR: Error parsing data: probably invalid message (length=%d)\n", len);
	for (int i=0; i<len-1; i++) if (buf[i] == 0) buf[i] = ' ';
	fprintf(stderr, "LMS1xx::getData() invalid message is '");
	fputs(buf, stderr);
	fprintf(stderr, "\n");
	return false;
}

void LMS1xx::saveConfig()
{
	char buf[100];
	sprintf(buf, "%c%s%c", 0x02, "sMN mEEwriteall", 0x03);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	int len = read(sockDesc, buf, 100);

	if (buf[0] != 0x02)
		fprintf(stderr, "LMS1xx WARNING: invalid packet recieved\n");
	buf[len] = 0;
	if (debugLevel >= LOG_LEVEL_DEBUG) fprintf(stderr, "LMS1xx DEBUG: RX: %s\n", buf);
}

void LMS1xx::startDevice()
{
	char buf[100];
	sprintf(buf, "%c%s%c", 0x02, "sMN Run", 0x03);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
	write(sockDesc, buf, strlen(buf));
#pragma GCC diagnostic pop

	int len = read(sockDesc, buf, 100);

	if (buf[0] != 0x02)
		fprintf(stderr, "LMS1xx WARNING: invalid packet recieved\n");
	buf[len] = 0;
	if (debugLevel >= LOG_LEVEL_DEBUG) fprintf(stderr, "LMS1xx DEBUG: RX: %s\n", buf);
}
