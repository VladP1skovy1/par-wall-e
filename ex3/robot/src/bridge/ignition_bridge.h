/*
 * File name: ignition_bridge.h
 * Date:      2018/09/24 09:26
 * Author:    Jan Chudoba
 */

#ifndef __IGNITION_BRIDGE_H__
#define __IGNITION_BRIDGE_H__

#include <stdint.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <string>
#include <map>

#include <ignition/transport.hh>

#include "robot/common.h"

#define IGNITION_BRIDGE_DEFAULT_PORT 44666

class CIgnitionBridge
{
public:
	CIgnitionBridge();
	~CIgnitionBridge();

	void initializeAuto();
	void initialize(const char * subscribeTopicFile = NULL, const char * advertiseTopicFile = NULL);

	void stop();

	bool startServer(int port = 0);
	bool startClient(const char * serverName, int port = 0);

	const char * getLastError() { return lastError; }

protected:
	bool createSocket();
	bool sendHandshake();
	bool sendMessage(uint8_t * data, int length);
	bool parseByte(uint8_t b);
	void checkMessageBufferSize();
	void processReceivedMessage(uint8_t * data, int length);
	void handshakeError();
	bool startThread();
	void receiveThreadBody();
	void topicDiscoveryThreadBody();
	static void * receive_thread_body(void * arg) { ((CIgnitionBridge*)arg)->receiveThreadBody(); return 0; }
	static bool resolveIpAddress(struct sockaddr * & addr, size_t & addrLen, const char * hostName, int port, int socktype = SOCK_STREAM, int family = AF_UNSPEC);
	static void * topic_discovery_thread_body(void * arg) { ((CIgnitionBridge*)arg)->topicDiscoveryThreadBody(); return 0; }

	// ignition stuff
	bool subscribeTopics();
	bool advertiseTopics();
	bool subscribeAllTopics();
	void cbTopicIn(const ::google::protobuf::Message & msg, const ignition::transport::MessageInfo & info);
	static void cbTopicInRaw(const char * msgData, const size_t dataSize, const ignition::transport::MessageInfo & info);

	static int packDatagram(uint8_t * datagram, unsigned int maxSize, const char * topicName, const char * typeName, const uint8_t * data, unsigned int dataSize);
	static bool unpackDatagram(const uint8_t * datagram, unsigned int datagramLength, std::string & topicName, std::string & typeName, std::string & data);

	static CIgnitionBridge * getInstance() { return instance; }
private:
	int sockfd;
	int commfd;
	bool serverMode;
	bool connected;
	bool handshake;
	pthread_t receiveThread;
	pthread_t topicDiscoveryThread;
	bool receiveThreadRunning;
	bool topicDiscoveryThreadRunning;
	bool receiveThreadInterrupt;
	const char * lastError;
	CMutex sendMutex;

	struct sockaddr_in connectAddress;

	// protocol parser
	int parserState;
	uint32_t parserMessageLength;
	uint32_t parserMessagePos;
	uint32_t messageBufferSize;
	uint8_t * messageBuffer;

	// ignition transport
	std::string subscribeTopicFileName;
	std::string advertiseTopicFileName;
	bool autoSubscribeTopics;
	ignition::transport::Node node;
	std::map<std::string, ignition::transport::Node::Publisher> publishers;
	std::map<std::string, bool> subscribedTopics;

	static CIgnitionBridge * instance;
};

#endif

/* end of ignition_bridge.h */
