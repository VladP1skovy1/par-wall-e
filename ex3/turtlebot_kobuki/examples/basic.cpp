/*
 * File name: basic.cpp
 * Date:      2018/08/31 13:22
 * Author:    Jan Chudoba
 */

#include <kobuki_driver/kobuki.hpp>

kobuki::Kobuki * gKobuki = NULL;

class Manager
{
	public:
	ecl::LegacyPose2D<double> pose;
	void processStreamData() {
		ecl::LegacyPose2D<double> pose_update;
		ecl::linear_algebra::Vector3d pose_update_rates;
		gKobuki->updateOdometry(pose_update, pose_update_rates);
		pose *= pose_update;
		//dx += pose_update.x();
		//dth += pose_update.heading();
		//std::cout << dx << ", " << dth << std::endl;
		//std::cout << kobuki.getHeading() << ", " << pose.heading() << std::endl;
		//std::cout << "[" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
	}
};

int main(int, char **)
{
	kobuki::Kobuki kobuki;
	gKobuki = &kobuki;

	kobuki::Parameters parameters;
	// change the default device port from /dev/kobuki to /dev/ttyUSB0
	parameters.device_port = "/dev/ttyUSB0";
	// Other parameters are typically happy enough as defaults
	// namespaces all sigslot connection names under this value, only important if you want to
	parameters.sigslots_namespace = "/kobuki";
	// Most people will prefer to do their own velocity smoothing/acceleration limiting.
	// If you wish to utilise kobuki's minimal acceleration limiter, set to true
	parameters.enable_acceleration_limiter = false;
	// If your battery levels are showing significant variance from factory defaults, adjust thresholds.
	// This will affect the led on the front of the robot as well as when signals are emitted by the driver.
	parameters.battery_capacity = 16.5;
	parameters.battery_low = 14.0;
	parameters.battery_dangerous = 13.2;

	try {
		kobuki.init(parameters);
	} catch ( ecl::StandardException &e ) {
		std::cout << e.what();
	}

	Manager manager;
	ecl::Slot<> slot_stream_data(&Manager::processStreamData, manager);
	slot_stream_data.connect("/kobuki/stream_data");

	usleep(1000000);

	ecl::LegacyPose2D<double> pose;
	pose = manager.pose;
	std::cout << "current pose: [" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;

	kobuki.setBaseControl(0.1, 0.0);
	usleep(2000000);
	kobuki.setBaseControl(0.0, 0.0);

	usleep(1000000);

	//ecl::LegacyPose2D<double> pose;
	pose = manager.pose;
	std::cout << "current pose: [" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
}


/* end of basic.cpp */
