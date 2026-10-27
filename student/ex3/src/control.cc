/*
 * File name: control.cc
 * Date:      2018-11-27
 * Author:    Miroslav Kulich
 */
#include "control.h"

#include <unistd.h> //usleep
#include <fstream>
#include <cmath>

#include "canvas.h"
#include "imr-h/logging.h"
#include "imr-h/imr_exceptions.h"
#include "snd.h"


using namespace imr;


/// ----------------------------------------------------------------------------
/// Class CControl

/// ----------------------------------------------------------------------------
CControl::CControl(CMapGrid &map) : quit(false), map(map)
{
}

/// ----------------------------------------------------------------------------
CControl::~CControl()
{
    stop();
}

/// ----------------------------------------------------------------------------
void CControl::stop(void)
{
    std::lock_guard<std::mutex> lk(mtx);
    quit = true;
}

/// - public method ---------------------------------------------------------
void CControl::main(void)
{
    CRobotClientConfiguration cfg;
    cfg.waitForInitialization = true;
    robot.enableRequestResponseServices();
    //robot.setAsyncMode();
    if (!robot.initialize(&cfg))
    {
        std::cerr << "Failed to initialize robot!" << std::endl;
        return;
    }
    std::cerr << "Robot initialized." << std::endl;

    bool res = robot.getLaserSensorConfiguration(lacfg);

    std::cout << "Res " << res << std::endl;
    std::cout << "Range " << lacfg.maxRange << std::endl;
    std::cout << "Angle resolution " << lacfg.angularResolution << std::endl;
    std::cout << "Min angle " << lacfg.minAngle << std::endl;
    std::cout << "Count " << lacfg.scanSampleCount << std::endl;
    std::cout << "Frequency " << lacfg.frequency << std::endl;

  imr::CCanvas &canvas = imr::CCanvas::getInstance();
  
  // draw a map
  canvas.draw(map);


    bool q = quit;

    CPose pose;
    CLaserScan scan;
    SNDNavigation snd(robot);
    
    // TODO: update map, retrieve nearest goal from planner, set goal to snd, check proximity to goal

    while (!q)
    {
        bool fresh = false;
        if (robot.isFresh(ROBOT_DATA_ODOMETRY))
        {
            if (robot.getOdometry(pose))
            {
                std::cout << "odometry: [" << pose.x << ", " << pose.y << ", " << pose.heading << "]" << std::endl;
                canvas.drawPixel(map.getX(pose.x),map.getY(pose.y),Color::Blue);
            }
            fresh = true;
        }
        if (robot.isFresh(ROBOT_DATA_LASER))
        {
            if (robot.getLaserSensorData(scan))
            {
                std::cout << "scan " << scan.range.size() << std::endl;
            }
            fresh = true;

            snd.GoTo(1,1,3.14/2,true);
            canvas.redraw();
        }
        if (!fresh)
        {
            usleep(100000);
        }

        std::lock_guard<std::mutex> lk(mtx);
        q = quit;
    }
}

/* end of robot.cc */
