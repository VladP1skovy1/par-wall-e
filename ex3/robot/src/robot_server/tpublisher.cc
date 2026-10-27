/*
 * File name: trobot_server.cc
 * Date:      2018/09/14 14:15
 * Author:    Jan Chudoba
 */

#include <stdio.h>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "robot_messages/robot_messages.pb.h"

int main(int argc, char ** argv)
{
	fprintf(stderr, "PAR robot server (test)\n");

	ignition::transport::Node node;
	std::string topic = "/turtlebot/odometry";

	auto pub = node.Advertise<robot_messages::OdometryMsg>(topic);
	if (!pub)
	{
		std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
		return -1;
	}

	// Prepare the message.
	//ignition::msgs::StringMsg msg;
	//msg.set_data("HELLO");

	robot_messages::OdometryMsg odoMsg;
	odoMsg.set_index(0);
	odoMsg.set_x(0.12);
	odoMsg.set_y(0.21);
	odoMsg.set_heading(3.1415927);

	// Publish messages at 1Hz.
	while (true)
	{
		fprintf(stderr, "odo: %.3f, %.3f, %.3f\n", odoMsg.x(), odoMsg.y(), odoMsg.heading());

		if (!pub.Publish(odoMsg))
			break;

		std::cout << "Publishing topic [" << topic << "]" << std::endl;
		usleep(1000000);
	}

	return 0;
}

/* end of trobot_server.cc */
