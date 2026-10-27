/*
 * File name: map.cc
 * Date:      2013/10/07 09:40
 * Author:    Miroslav Kulich
 */

#include "map_grid.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include "imr-h/imr_assert.h"
#include "imr-h/logging.h"

using namespace imr;

const double CMapGrid::FLOOR = 0;
const double CMapGrid::UNKNOWN = 0.5;
const double CMapGrid::WALL = 1;
const double CMapGrid::EMPTY = 0;
const double CMapGrid::OCCUPIED = 1;

CMapGrid::CMapGrid(int width, int height, double cellSize)
    : height(height), width(width), cellSize(cellSize) {
  data = new double[height * width];
  for (long int i = 0; i < height * width; i++) {
    data[i] = UNKNOWN;
  }
};

CMapGrid::CMapGrid(std::string name) {
  std::ifstream infile(name.c_str(), std::ios_base::in);
  std::string line;
  int i = 0;
  int ii = 0;
  INFO("LOADING MAP " << name);
  while (getline(infile, line, '\n')) {
    i++;

    if (i == 1) {
      std::stringstream strStream(line);
      strStream >> height;
      INFO("MAP HEIGHT " << height);
      continue;
    }

    if (i == 2) {
      width = line.size();
      INFO("MAP WIDTH " << width);
      data = new double[height * width];
    }

    for (unsigned int k = 0; k < line.size(); k++) {
      data[ii++] = line[k] == '.' ? (double)FLOOR : (double)WALL;
    }
  }

  INFO("Map loaded. Width " << width << " height " << height);
}

CMapGrid::~CMapGrid() { delete data; }

double CMapGrid::getCell(int x, int y) {
  assert_argument(x >= 0 && x < width, "getCell: X out of range");
  assert_argument(y >= 0 && y < height, "getCell: Y out of range");
  double cell = data[y * width + x];
  return cell;
};

void CMapGrid::setCell(int x, int y, double value) {
  assert_argument(x >= 0 && x < width, "setCell: X out of range");
  assert_argument(y >= 0 && y < height, "setCell: Y out of range");
  data[y * width + x] = value;
};

void CMapGrid::updateCell(int x, int y, double value) { 
  //do stuff here
  setCell(x, y, value); 
};

void CMapGrid::SWAP(int &x, int &y) {
  int p;
  p = x;
  x = y;
  y = p;
}

void CMapGrid::updateCells(int r_x, int r_y, int c_x, int c_y, double dist) {
  // cout << "upt cells" << endl;
  double p, d;
  int dx = c_x - r_x;
  int dy = c_y - r_y;
  int steep = (std::abs(dy) >= std::abs(dx));

  if (steep) {
    SWAP(r_x, r_y);
    SWAP(c_x, c_y);
    // recompute Dx, Dy after swap
    dx = c_x - r_x;
    dy = c_y - r_y;
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
  int y = r_y;
  int xDraw, yDraw;
  for (int x = r_x; x != c_x; x += xstep) {
    if (steep) {
      xDraw = y;
      yDraw = x;
    } else {
      xDraw = x;
      yDraw = y;
    }
    // do stuff with xDraw yDraw like updateCell...
    // updateCell(xDraw, yDraw, p);

    // next
    if (e > 0) {
      e += twoDyTwoDx;  // E += 2*Dy - 2*Dx;
      y = y + ystep;
    } else {
      e += twoDy;  // E += 2*Dy;
    }
  }
}

int CMapGrid::getHeight() { return height; };

int CMapGrid::getWidth() { return width; };

int CMapGrid::getX(double x) {
  int xx = x / cellSize;
  assert_argument(xx >= 0 && xx < width, "X out of range");
  return xx;
}

int CMapGrid::getY(double y) {
  int yy = y / cellSize;
  //int yy = (height - 1 - ((int)(y / cellSize)));
  assert_argument(yy >= 0 && yy < height, "Y out of range!");
  return yy;
}

double CMapGrid::getPosX(int x) { return (double)x * cellSize; }

double CMapGrid::getPosY(int y) {
  return (double) y * cellSize;
  //return (double)(height - 1 - y) * cellSize;
}

void CMapGrid::copyTo(double *grid) {
  std::copy(data, data + (width * height), grid);
}
