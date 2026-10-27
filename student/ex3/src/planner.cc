/*
 * File name: planner.cc
 * Date:      2016-11-01
 * Author:    Miroslav Kulich, Lukáš Bertl
 */

#include "planner.h"

#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>

#include "imr-h/logging.h"

using namespace imr;
using namespace dijkstra;

CPlanner::CPlanner() {}

/**
    Non-member method. Swaps two integer values.
*/
void SWAP(int &x, int &y) {
  int p;
  p = x;
  x = y;
  y = p;
}

bool CPlanner::bresenham(int x0, int y0, int x1, int y1) {
  int dx = x1 - x0;
  int dy = y1 - y0;
  int steep = (abs(dy) >= abs(dx));
  if (steep) {
    SWAP(x0, y0);
    SWAP(x1, y1);
    // recompute Dx, Dy after swap
    dx = x1 - x0;
    dy = y1 - y0;
  }
  int xstep = 1;
  if (dx < 0) {
    xstep = -1;
    dx = -dx;
  }
  int ystep = 1;
  if (dy < 0) {
    ystep = -1;
    dy = -dy;
  }
  int twoDy = 2 * dy;
  int twoDyTwoDx = twoDy - 2 * dx;  // 2*Dy - 2*Dx
  int e = twoDy - dx;               // 2*Dy - Dx
  int y = y0;
  int xDraw, yDraw;
  for (int x = x0; x != x1; x += xstep) {
    if (steep) {
      xDraw = y;
      yDraw = x;
    } else {
      xDraw = x;
      yDraw = y;
    }
    //TODO: do stuff with (xDraw, yDraw) here
    if (e > 0) {
      e += twoDyTwoDx;  // E += 2*Dy - 2*Dx;
      y = y + ystep;
    } else {
      e += twoDy;  // E += 2*Dy;
    }
  }

  return true;
}


void CPlanner::setMap(CMapGrid &map) {
    //TODO:
}


void CPlanner::inflateMap(void) {
    //TODO:
}




bool CPlanner::dijkstra(int x, int y) {
  bool result = true;  
  //TODO:
  return result;
}

void CPlanner::smooth_path(void) {
 //TODO:
}

bool CPlanner::plan(int x0, int y0, int x1, int y1) {
  bool result = true;
  //TODO:
  return result;
}
