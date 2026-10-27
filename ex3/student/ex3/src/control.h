/*
 * File name: control.h
 * Date:      2018-11-27
 * Author:    Miroslav Kulich
 */

#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <vector>
#include <mutex>

#include "map_grid.h"
#include "robot_client/robot_client.h"

/**
    Class CControl provides communication with the robot (in simulator or the
    real one). Position and other sensor data are available here. Also the
    commands for SND driver (robot motion controller) are send from here.
*/

namespace imr {

class CControl {


    public:

        /**
            Constructor. Initialze all sensors and drivers. After initialization
            the variables are checked, if everything was successful.
        */
        CControl(CMapGrid &map);

        /**
            Destructor.
        */
        ~CControl();

        /**
            Request to stop the control loop. Flag quit is set to true. When this
            is recognized in navigation loop, it stops.
        */
        void stop(void);


        /**
            Thread execution method. Whenever the execution of the thread (robot)
            is called, this method is raised. In this case threadBody calls a
            navigation method where the robot is controlled.

            Main control loop. Position and sensors are read here and robot motion
            is driven from here.

            Implement YOUR own control law here.
            The main idea what happens in this method is following:
                - Read and update robot's position
                - Read laser data and update your map (occupancy grid)
                - The first two steps provide new data for planner
                - Read current plan (output from the planner)
                - Drive robot to next node from the plan
        */
        void main(void);


    private:
        bool quit;        // Flag of quit request
        CMapGrid &map;    // map used during exploration
        std::mutex mtx;         // mutex to lock the quit variable
        CRobotClient robot;
        CLaserSensorConfiguration lacfg;



};

} //end namespace imr

#endif

/* end of robot.h */
