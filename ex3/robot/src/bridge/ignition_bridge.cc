/*
 * File name: ignition_bridge.cc
 * Date:      2018/09/24 09:26
 * Author:    Jan Chudoba
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <netdb.h>

#include "ignition_bridge.h" // TODO: in subdir

#define TOPIC_DISCOVERY_PERIOD_MS 5000

CIgnitionBridge * CIgnitionBridge::instance = NULL;

CIgnitionBridge::CIgnitionBridge() :
	sockfd(-1),
	commfd(-1),
	serverMode(false),
	connected(false),
	handshake(false),
	receiveThreadRunning(false),
	topicDiscoveryThreadRunning(false),
	lastError(""),
	messageBufferSize(0),
	messageBuffer(NULL),
	subscribeTopicFileName("subscribe.txt"),
	advertiseTopicFileName("advertise_topics.txt")
{
	if (instance == NULL) {
		instance = this;
	}
}

CIgnitionBridge::~CIgnitionBridge()
{
	stop();
	if (instance == this) {
		instance = NULL;
	}
}

void CIgnitionBridge::initializeAuto()
{
	subscribeTopicFileName.clear();

	autoSubscribeTopics = true;

	advertiseTopics();
	subscribeTopics();
}

void CIgnitionBridge::initialize(const char * subscribeTopicFile, const char * advertiseTopicFile)
{
	if (subscribeTopicFile) {
		subscribeTopicFileName = subscribeTopicFile;
	}
	if (advertiseTopicFile) {
		advertiseTopicFileName = advertiseTopicFile;
	}
	// TODO: [low] advertising may not be needed, as advertisements are made automatically
	advertiseTopics();
	subscribeTopics();
}

void CIgnitionBridge::stop()
{
	receiveThreadInterrupt = true;

	if (commfd >= 0) {
		shutdown(commfd, SHUT_RDWR);
		::close(commfd);
		
		if (sockfd >= 0 && sockfd != commfd) {
			shutdown(sockfd, SHUT_RDWR);
			::close(sockfd);
		}
		commfd = -1;
		sockfd = -1;
	}
	connected = false;

	if (receiveThreadRunning) {
		pthread_join(receiveThread, 0);
		receiveThreadRunning = false;
	}
	if (topicDiscoveryThreadRunning) {
		pthread_join(topicDiscoveryThread, 0);
		topicDiscoveryThreadRunning = false;
	}
}

bool CIgnitionBridge::startServer(int port)
{
	stop();

	if (!createSocket()) return false;

	if (port <= 0) {
		port = IGNITION_BRIDGE_DEFAULT_PORT;
	}

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof addr);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(sockfd,(struct sockaddr *)&addr, sizeof addr) < 0) {
		lastError = strerror(errno);
		::close(sockfd);
		sockfd = -1;
		return false;
	}

	if(listen(sockfd, 2) < 0) {
		lastError = strerror(errno);
		::close(sockfd);
		sockfd = -1;
		return false;
	}

	serverMode = true;

	return startThread();
}

bool CIgnitionBridge::startClient(const char * serverName, int port)
{
	stop();

	if (!createSocket()) return false;

	if (port <= 0) {
		port = IGNITION_BRIDGE_DEFAULT_PORT;
	}

	memset(&connectAddress, 0, sizeof(connectAddress));

	struct sockaddr * pAddr = (struct sockaddr*) &connectAddress;
	size_t addrLen = sizeof(connectAddress);
	if (!resolveIpAddress(pAddr, addrLen, serverName, port)) {
		lastError = "address resolve error";
		stop();
		return false;
	}

	serverMode = false;

	return startThread();
}

bool CIgnitionBridge::createSocket()
{
	if (sockfd >= 0) stop();

	int fd;
	fd = ::socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (fd < 0) {
		lastError = strerror(errno);
		return false;
	}

	int so_reuse_addr_value = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &so_reuse_addr_value, sizeof(so_reuse_addr_value));
	sockfd = fd;

	return true;
}

bool CIgnitionBridge::sendHandshake()
{
	uint8_t header[8];
	header[0] = 0xAA;
	header[1] = 0xBB;
	header[2] = 0xCC;
	header[3] = 0xDD;
	header[4] = 0x00;
	header[5] = 0x00;
	header[6] = 0x00;
	header[7] = 0x00;

	return (::send(commfd, header, sizeof(header), 0) == sizeof(header));
}

bool CIgnitionBridge::sendMessage(uint8_t * data, int length)
{
	if (!connected || handshake) {
		return false;
	}

	uint8_t header[8];

	header[0] = 0xAA;
	header[1] = 0x00;
	header[2] = 0x00;
	header[3] = 0x00;
	header[4] = (length >> 24) & 0xFF;
	header[5] = (length >> 16) & 0xFF;
	header[6] = (length >> 8) & 0xFF;
	header[7] = (length) & 0xFF;

	bool result;
	sendMutex.lock();
	result = (::send(commfd, header, sizeof(header), 0) == sizeof(header));

	if (result) {
		result &= (::send(commfd, data, length, 0) == length);
	}

	sendMutex.unlock();

	return result;
}

bool CIgnitionBridge::parseByte(uint8_t b)
{
	// protocol description:
	// HEADER (8B) | DATA (variable length)
	// header:
	// AA 00 00 00 [ length 4-bytes, MSB first ]

	// parserState:
	// 0 = search for header byte 0, 1-3 search for header byte 1-3
	// 4-7 = read message length bytes
	// 8 = read data
	
	// handshake message: AA BB CC DD 00 00 00 00
	
	//if (handshake) {
		//// handshake debugging
		//fprintf(stderr, "DEBUG: handshake state=%d byte=%02X\n", parserState, b);
	//}

	switch (parserState) {
	case 0:
		if (b == 0xAA) parserState++;
		else if (handshake) {
			handshakeError();
		}
		break;
	case 1:
	case 2:
	case 3:
		if (handshake) {
			switch (parserState) {
			case 1: if (b == 0xBB) parserState++; else handshakeError(); break;
			case 2: if (b == 0xCC) parserState++; else handshakeError(); break;
			case 3: if (b == 0xDD) parserState++; else handshakeError(); break;
			}
		} else {
			if (b == 0x00) parserState++;
			else parserState = 0;
		}
		break;
	case 4:
		parserMessageLength = ((uint32_t)b) << 24;
		if (handshake && (parserMessageLength != 0)) handshakeError();
		else parserState++;
		break;
	case 5:
		parserMessageLength |= ((uint32_t)b) << 16;
		if (handshake && (parserMessageLength != 0)) handshakeError();
		else parserState++;
		break;
	case 6:
		parserMessageLength |= ((uint32_t)b) << 8;
		if (handshake && (parserMessageLength != 0)) handshakeError();
		else parserState++;
		break;
	case 7:
		parserMessageLength |= ((uint32_t)b);
		if (handshake) {
			if (parserMessageLength != 0) {
				handshakeError();
			} else {
				fprintf(stderr, "DEBUG: handshake successfull\n");
				handshake = false;
				parserState = 0;
			}
		} else {
			checkMessageBufferSize();
			parserMessagePos = 0;
			parserState++;
		}
		break;
	case 8:
		messageBuffer[parserMessagePos++] = b;
		if (parserMessagePos == parserMessageLength) {
			processReceivedMessage(messageBuffer, parserMessagePos);
			parserState = 0;
		}
		break;
	}
	return true;
}

void CIgnitionBridge::checkMessageBufferSize()
{
	if (messageBufferSize < parserMessageLength) {
		if (messageBuffer) {
			delete[] messageBuffer;
		}
		messageBufferSize = parserMessageLength;
		messageBuffer = new uint8_t[messageBufferSize];
	}
}

void CIgnitionBridge::processReceivedMessage(uint8_t * data, int length)
{
	std::string topicName;
	std::string typeName;
	std::string topicData;

	if (unpackDatagram(data, length, topicName, typeName, topicData)) {
		fprintf(stderr, "received message of length %d, topic '%s', type '%s', data length %d\n", length, topicName.c_str(), typeName.c_str(), (int)topicData.length());
		if (publishers.find(topicName) == publishers.end()) {
			// check if topic is not locally subscribed
			if (subscribedTopics.find(topicName) == subscribedTopics.end()) {
				// try advertise
				ignition::transport::Node::Publisher newPub = node.Advertise(topicName, typeName);
				if (newPub) {
					fprintf(stderr, "INFO: advertised topic '%s' (%s)\n", topicName.c_str(), typeName.c_str());
					publishers[topicName] = newPub;
				}
			}
		}
		if (publishers.find(topicName) != publishers.end()) {
			ignition::transport::Node::Publisher & pub = publishers[topicName];
			if (pub.PublishRaw(topicData, typeName)) {
			} else {
				fprintf(stderr, "WARNING: failed to publish (raw) topic '%s' message type '%s' length %d\n", topicName.c_str(), typeName.c_str(), (int)topicData.length());
			}
		} else {
			fprintf(stderr, "WARNING: no publisher for topic '%s' message type '%s' length %d\n", topicName.c_str(), typeName.c_str(), (int)topicData.length());
		}
	} else {
		fprintf(stderr, "ERROR: received message of length %d, failed to parse\n", length);
	}
}

void CIgnitionBridge::handshakeError()
{
	fprintf(stderr, "DEBUG: handshake error (parserState %d)\n", parserState);
	parserState = 255;
	::close(commfd);
	commfd = -1;
	connected = false;
}

bool CIgnitionBridge::startThread()
{
	receiveThreadInterrupt = false;
	if (pthread_create(&receiveThread, 0, receive_thread_body, this) != 0) {
		lastError = strerror(errno);
		::close(sockfd);
		sockfd = -1;
		return false;
	}
	receiveThreadRunning = true;

	if (autoSubscribeTopics) {
		if (pthread_create(&topicDiscoveryThread, 0, topic_discovery_thread_body, this) != 0) {
			fprintf(stderr, "ERROR: failed to start topic discovery thread\n");
		} else {
			topicDiscoveryThreadRunning = true;
		}
	}

	return true;
}

void CIgnitionBridge::receiveThreadBody()
{
	fprintf(stderr, "DEBUG: thread started\n");
	connected = false;
	while (!receiveThreadInterrupt) {
		if (!connected) {
			if (serverMode) {
				socklen_t clientAddrSize = 0;
				commfd = ::accept(sockfd, (struct sockaddr *) & connectAddress, &clientAddrSize);
				if (commfd >= 0) {
					handshake = true;
					connected = true;
					fprintf(stderr, "DEBUG: client connected\n");
				}
			} else {
				fprintf(stderr, "DEBUG: connecting to server ...\n");
				connected = (::connect(sockfd, (struct sockaddr *)&connectAddress, sizeof connectAddress) < 0);
				if (connected) {
					commfd = sockfd;
					fprintf(stderr, "DEBUG: connected to server\n");
					sendHandshake();
				}
			}
			if (connected) {
				parserState = 0;
			} else {
				usleep(1000000);
			}
		} // ! connected

		if (connected) {
			uint8_t buffer[256];
			int r = ::recv(commfd, buffer, sizeof(buffer), 0);
			if (r > 0) {
				for (int i = 0; i < r; i++) {
					parseByte(buffer[i]);
				}
			} else if (r <= 0) {
				if (serverMode) {
					::close(commfd);
					commfd = -1;
					fprintf(stderr, "DEBUG: client disconnected\n");
				} else {
					fprintf(stderr, "DEBUG: server closed connection\n");
					usleep(1000000);
				}
				connected = false;
			}
		} // connected
	} // while ! interrupt
	fprintf(stderr, "DEBUG: thread interrupted\n");
}

void CIgnitionBridge::topicDiscoveryThreadBody()
{
	while (!receiveThreadInterrupt) {
		usleep(TOPIC_DISCOVERY_PERIOD_MS * 1000);
		subscribeAllTopics();
	}
}

bool CIgnitionBridge::resolveIpAddress(struct sockaddr * & addr, size_t & addrLen, const char * hostName, int port, int socktype, int family)
{
	struct addrinfo hints;
	struct addrinfo *result, *rp;
	int s;

	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = family;
	hints.ai_socktype = socktype; /* Datagram socket */
	hints.ai_flags = 0;
	hints.ai_protocol = 0;	   /* Any protocol */

	s = getaddrinfo(hostName, NULL, &hints, &result);
	if (s != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
		return false;
	}

	/* getaddrinfo() returns a list of address structures.
		Try each address until we successfully connect(2).
		If socket(2) (or connect(2)) fails, we (close the socket
		and) try the next address. */

	for (rp = result; rp != NULL; rp = rp->ai_next) {
		fprintf(stderr, "getaddrinfo result family=%d\n", rp->ai_family);
		if (rp->ai_family != AF_INET) {
			// change if IPv6 is to be supported
			continue;
		}
		//sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		//if (sfd == -1) continue;

		//if (connect(sfd, rp->ai_addr, rp->ai_addrlen) != -1) break;		   /* Success */

		//close(sfd);

		if (addr == NULL) {
			// we need to allocate addr
			addr = (struct sockaddr *) new uint8_t[rp->ai_addrlen];
			memcpy(addr, rp->ai_addr, rp->ai_addrlen);
			if (rp->ai_family == AF_INET) {
				((struct sockaddr_in *)addr)->sin_port = htons(port);
			} else if (rp->ai_family == AF_INET6) {
				((struct sockaddr_in6 *)addr)->sin6_port = htons(port);
			} else {
				delete[] (uint8_t*)addr;
				addr = NULL;
			}
			if (addr != NULL) {
				addrLen = rp->ai_addrlen;
				break;
			}
		} else {
			// addr is already allocated
			if (rp->ai_addrlen <= addrLen) {
				memcpy(addr, rp->ai_addr, rp->ai_addrlen);
				if (rp->ai_family == AF_INET) {
					((struct sockaddr_in *)addr)->sin_port = htons(port);
				} else if (rp->ai_family == AF_INET6) {
					((struct sockaddr_in6 *)addr)->sin6_port = htons(port);
				} else {
					fprintf(stderr, "Invalid/unknown family: %d", rp->ai_family);
				}
			} else {
				fprintf(stderr, "Socket address structure too long %u > %u\n", rp->ai_addrlen, (unsigned) addrLen);
			}
			break;
		}
	}

	bool res;
	if (rp == NULL) {		   /* No address succeeded */
		fprintf(stderr, "Could not resolve address '%s'\n", hostName);
		res = false;
	} else {
		res = true;
	}

	freeaddrinfo(result);	   /* No longer needed */
	return res;
}


// =============================================================================
// ignition stuff
// =============================================================================

bool CIgnitionBridge::subscribeTopics()
{
	if (autoSubscribeTopics) {
		return subscribeAllTopics();
	}

	FILE * f = fopen(subscribeTopicFileName.c_str(), "r");
	if (!f) {
		return false;
	}
	bool result = true;
	char line[128];
	while (fgets(line, sizeof(line), f)) {
		char *c = line; while (*c != 0) { if (*c == '\n') { *c = 0; break; } c++; } // remove LF
		if (line[0] == 0) continue;
		//if (node.Subscribe<CIgnitionBridge>(line, &CIgnitionBridge::cbTopicIn, this))
		if (node.SubscribeRaw(line, &CIgnitionBridge::cbTopicInRaw)) {
			subscribedTopics[line] = true;
			fprintf(stderr, "INFO: subscribed topic '%s'\n", line);
		} else {
			fprintf(stderr, "ERROR: failed to subscribe topic '%s'\n", line);
			result = false;
		}
	}
	fclose(f);
	return result;
}

bool CIgnitionBridge::advertiseTopics()
{
	FILE * f = fopen(advertiseTopicFileName.c_str(), "r");
	if (!f) {
		return false;
	}
	bool result = true;
	char line[128];
	while (fgets(line, sizeof(line), f)) {
		char *c = line; while (*c != 0) { if (*c == '\n') { *c = 0; break; } c++; } // remove LF
		if (line[0] == 0) continue;
		std::string topicName;
		std::string typeName;
		char * saveptr;
		char * token;
		token = strtok_r(line, " ", &saveptr);
		if (!token) fprintf(stderr, "ERROR: invalid topic to advertise: '%s'\n", line);
		topicName = token;
		token = strtok_r(NULL, " ", &saveptr);
		if (!token) fprintf(stderr, "ERROR: invalid topic to advertise: '%s'\n", line);
		typeName = token;
		publishers[topicName] = node.Advertise(topicName, typeName);
		if (publishers[topicName]) {
			fprintf(stderr, "INFO: advertised topic '%s' (%s)\n", topicName.c_str(), typeName.c_str());
		} else {
			publishers.erase(topicName);
			fprintf(stderr, "ERROR: failed to advertise topic '%s' (%s)\n", topicName.c_str(), typeName.c_str());
			result = false;
		}
	}
	fclose(f);
	return result;
}

bool CIgnitionBridge::subscribeAllTopics()
{
	std::vector< std::string > topics;
	node.TopicList(topics);

	bool result = true;

	for (std::vector<std::string>::iterator it = topics.begin(); it != topics.end(); it++) {
		if (subscribedTopics.find(*it) == subscribedTopics.end()) {
			// not subscribed yet
			if (publishers.find(*it) == publishers.end()) {
				// not even published locally
				if (node.SubscribeRaw(*it, &CIgnitionBridge::cbTopicInRaw)) {
					subscribedTopics[*it] = true;
					fprintf(stderr, "INFO: auto subscribed topic '%s'\n", (*it).c_str());
				} else {
					fprintf(stderr, "ERROR: failed to auto-subscribe topic '%s'\n", (*it).c_str());
					result = false;
				}
			} else {
				fprintf(stderr, "INFO: auto subscribe: ignore locally published topic '%s'\n", (*it).c_str());
				// ignoring locally published topics
			}
		} else {
			// already subscribed
			fprintf(stderr, "INFO: auto subscribe: topic '%s' already subscribed\n", (*it).c_str());
		}
	}
	return result;
}

void CIgnitionBridge::cbTopicIn(const google::protobuf::Message & msg, const ignition::transport::MessageInfo & info)
{
	fprintf(stderr, "DEBUG: received subscribed topic '%s' (%s)\n", info.Topic().c_str(), info.Type().c_str());
}

void CIgnitionBridge::cbTopicInRaw(const char * msgData, const size_t dataSize, const ignition::transport::MessageInfo & info)
{
	fprintf(stderr, "DEBUG: received subscribed topic '%s' (%s) RAW data size %u\n", info.Topic().c_str(), info.Type().c_str(), (unsigned int) dataSize);
#if 0
	for (unsigned int i = 0; i < dataSize; i++) {
		fprintf(stderr, " %02X", (uint8_t)msgData[i]);
	}
	fprintf(stderr, "\n");
#endif

	CIgnitionBridge * bridge = CIgnitionBridge::getInstance();
	if (!bridge) return;

	uint8_t datagram[10*1024];
	int datagramSize = packDatagram(datagram, sizeof(datagram), info.Topic().c_str(), info.Type().c_str(), (const uint8_t*) msgData, dataSize);
	if (datagramSize > 0) {
		bridge->sendMessage(datagram, datagramSize);
	} else {
		fprintf(stderr, "ERROR: failed to pack message size %u topic '%s' type '%s'\n", (unsigned int)dataSize, info.Topic().c_str(), info.Type().c_str());
	}
}

int CIgnitionBridge::packDatagram(uint8_t * datagram, unsigned int maxSize, const char * topicName, const char * typeName, const uint8_t * data, unsigned int dataSize)
{
	if (dataSize > 0xFFFF) return -1;
	unsigned int topicNameLength = strlen(topicName);
	unsigned int typeNameLength = strlen(typeName);
	if (topicNameLength > 0xFF) return -1;
	if (typeNameLength > 0xFF) return -1;
	unsigned int neededSize = 4 + topicNameLength + typeNameLength + dataSize;
	if (neededSize > maxSize) return -1;
	unsigned int ptr = 0;
	datagram[ptr++] = topicNameLength;
	memcpy(datagram + ptr, topicName, topicNameLength);
	ptr += topicNameLength;
	datagram[ptr++] = typeNameLength;
	memcpy(datagram + ptr, typeName, typeNameLength);
	ptr += typeNameLength;
	datagram[ptr++] = (dataSize >> 8) & 0xFF;
	datagram[ptr++] = (dataSize) & 0xFF;
	memcpy(datagram + ptr, data, dataSize);
	ptr += dataSize;
	return ptr;
}

bool CIgnitionBridge::unpackDatagram(const uint8_t * datagram, unsigned int datagramLength, std::string & topicName, std::string & typeName, std::string & data)
{
	if (datagramLength < 4) return false;
	unsigned int ptr = 0;
	uint8_t topicNameLength = datagram[ptr++];
	if (ptr + topicNameLength > datagramLength) return false;
	topicName.assign((const char*) (datagram + ptr), topicNameLength);
	ptr += topicNameLength;
	uint8_t typeNameLength = datagram[ptr++];
	if (ptr + typeNameLength > datagramLength) return false;
	typeName.assign((const char *) (datagram + ptr), typeNameLength);
	ptr += typeNameLength;
	uint16_t dataLength = ((uint16_t) datagram[ptr++]) << 8;
	dataLength |= datagram[ptr++];
	if (ptr + dataLength > datagramLength) return false;
	if (ptr + dataLength < datagramLength) {
		fprintf(stderr, "WARNING: received datagram with extra data\n");
	}
	data.assign((const char *) (datagram + ptr), dataLength);
	return true;
}

void printHelp()
{
	FILE * f = stdout;
	fprintf(f, "Usage: ignition_bridge [-s] [-a]\n");
	fprintf(f, " -a    auto subscribe/advertise mode (recommended)\n");
	fprintf(f, " -s    server mode\n");
}

// =============================================================================
// main()
// =============================================================================

int main(int argc, char ** argv)
{
	bool serverMode = false;
	const char * hostName = NULL;
	const char * subscribeTopicFileName = NULL;
	const char * advertiseTopicFileName = NULL;
	bool autoMode = false;

	int a = 1;
	while (a < argc) {
		if (argv[a][0] == '-') {
			switch (argv[a][1]) {
			case 'h':
				printHelp();
				exit(0);
				break;
			case 's':
				serverMode = true;
				break;
			case 'a':
				autoMode = true;
				break;
			case '-':
				if (a+1 >= argc) {
					fprintf(stderr, "option parameter missing\n");
					exit(1);
				}
				if (strcmp(argv[a], "--subscribe") == 0) {
					subscribeTopicFileName = argv[++a];
					fprintf(stderr, "subscribe file %s\n", subscribeTopicFileName);
				} else
				if (strcmp(argv[a], "--advertise") == 0) {
					advertiseTopicFileName = argv[++a];
					fprintf(stderr, "advertise file %s\n", advertiseTopicFileName);
				} else {
					fprintf(stderr, "Invalid long option: %s\n", argv[a]);
					exit(1);
				}
				break;
			default:
				fprintf(stderr, "Invalid option: %s\n", argv[a]);
				exit(1);
			}
			a++;
		} else {
			break;
		}
	}

	fprintf(stderr, "IGNITION BRIDGE %s\n", serverMode ? "SERVER" : "CLIENT");

	if (!serverMode) {
		if (a < argc) {
			hostName = argv[a++];
		} else {
			fprintf(stderr, "Host name missing\n");
			exit(1);
		}
	}

	CIgnitionBridge bridge;

	if (autoMode) {
		bridge.initializeAuto();
	} else {
		bridge.initialize(subscribeTopicFileName, advertiseTopicFileName);
	}

	bool result;

	if (serverMode) {
		result = bridge.startServer();
	} else {
		result = bridge.startClient(hostName);
	}

	if (!result) {
		fprintf(stderr, "ERROR: %s\n", bridge.getLastError());
		exit(2);
	}

	fprintf(stderr, "press ENTER to exit\n");
	getchar();
	fprintf(stderr, "terminating ...\n");

	bridge.stop();

	return 0;
}

/* end of ignition_bridge.cc */
