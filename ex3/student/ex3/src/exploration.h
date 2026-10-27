/*
 * File name: exploration.h
 * Date:      2018-11-27
 * Author:    Miroslav Kulich
 */

#ifndef __EXPLORATION_H__
#define __EXPLORATION_H__

#include <mutex>

// #include "robot_types.h"
#include "map_grid.h"
#include "planner.h"
#include "control.h"

/**
    Class of exploration represents another thread, which is executed
    simultaneously with robot control and data acquisition. It is
    supposed to seek frontiers, plan new path for robot in dynamicaly
    changing map and give new objectives to the robot.
*/

namespace imr {


class CExploration  {

    public:
        /**
            Constructor for exploration class. The class needs a robot, which it
            will control, a map to which it will write new data (explored space)
            and basic configuration. Method assigns parameters to private variables

            @param cfg  exploration configuration
            @param gridMap  a clear map
            @param robot    instance of controlled robot
        */
        CExploration(CMapGrid& gridMap, CControl &control);
        ~CExploration();

        /**
            Stop the exploration. This method only locks the mutex (ScopedLock)
            and sets quit variable to true
        */
        void stop(void);

        /**
            Main method of this class for users. Here the controll loop is implemented.
            User shall seek the frontiers, make new plan and pass the new plan to
            the robot and many other useful stuff.
        */
        void main(void);

    private:
        CControl &control;            // robot controlled by the instance (allows acces to the robot to change the plan, read data etc.)
        CMapGrid& gridMap;        // map of the environment, explored world
        bool quit;                // quit flag - if set to true, threadBody is stopped
        std::mutex mtx;                // mutex for quit flag
};

} //namespace imr
#endif

/* end of exploration.h */
