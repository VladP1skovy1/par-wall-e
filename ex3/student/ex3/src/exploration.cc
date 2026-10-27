/*
* File name: exploration.cc
* Date:      2016-09-27
* Author:    Miroslav Kulich
*/

#include <queue>
#include <cmath>
#include <limits>
#include <fstream>
#include <unistd.h> //usleep


#include "exploration.h"
#include "logging.h"

using namespace imr;

/// ----------------------------------------------------------------------------
/// Class CExploration



/// ----------------------------------------------------------------------------
CExploration::CExploration(CMapGrid& gridMap, CControl &control) :
    control(control), gridMap(gridMap), quit(false)
{
}

/// ----------------------------------------------------------------------------
CExploration::~CExploration()
{
    stop();
}




/// ----------------------------------------------------------------------------
void CExploration::stop(void)
{
    std::lock_guard<std::mutex> lk(mtx);
    quit = true;
}



/// - public method ---------------------------------------------------------
void CExploration::main(void)
{
    bool q = quit;

	// TODO: inflate current map, detect frontiers, run dijkstra, pick (closest) frontier, plan and smooth path, update path for control

    while (!q) {
        // sleep 1500 ms
        usleep(1500000);
        INFO("EXPLORER");
        std::lock_guard<std::mutex> lk(mtx);
        q = quit;
    }
}

/* end of exploration.cc */
