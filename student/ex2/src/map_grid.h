/*
 * File name: map_grid.h
 * Date:      2013/10/07 09:39
 * Author:    Miroslav Kulich
 */

#ifndef IMR_MAP_GRID
#define IMR_MAP_GRID

#include <string>

/**
    CMapGrid class represents the real environments as a grid.
    Cells have its size (cellSize) which coresponds to real arena dimensions.

    There are 2 predefined values - FLOOR and WALL. In further use (while
    calculating occupancy of the grid) it is recommended to keep this notation
    (1 is free space, 0 means obstacle) especialy because in this way the map
    is/will be handled while loading from file in planner.
    Of course it can be swapped...
*/

namespace imr {

class CMapGrid {
 public:
  /**
      Predefined values of the grid - wall and floor
  */
  static const double FLOOR;     //= 0;
  static const double UNKNOWN;   //= 0.5;
  static const double WALL;      //= 1;
  static const double EMPTY;     //= 0;
  static const double OCCUPIED;  //= 1;

  /**
      Simple constructor. Creates map grid with parameters given in the config

      @param cfg  Map configuration (width, height, cell size)
  */
  CMapGrid(int width, int height, double cellSize);

  /**
      Contructor, loads map from file where '.' denotes floor while other
      symbol (usually 0) denotes wall. Input file is specified by name.
      Size of the map in this case is stored at the beginning of the file.

      @param cfg  Map configuration
      @param name Name of the input map file
  */
  CMapGrid(std::string name);


  /**
      Destructor - free memory occupied by data array
  */
  ~CMapGrid();

  /**
      Get cell value at position [x,y]

      @param x    X coordinate of the cell
      @param y    Y coordinate of the cell
      @return cell value
  */
  double getCell(int x, int y);

  /**
      Set cell value at position [x,y]

      @param x        X coordinate of the cell
      @param y        Y coordinate of the cell
      @param value    new cell value
  */
  void setCell(int x, int y, double value);

  /**
      Update cell value - update the belief that cell is occupied
      based on the laser measurements.
      This method is supposed TBD by students

      @param x        X coordinate of the cell
      @param y        Y coordinate of the cell
      @param value    measurement value
  */
  void updateCell(int x, int y, double value);

  /**
      Get height of the grid (number of cells)

      @return height of the grid in cells
  */
  int getHeight();

  /**
      Get width of the grid (number of cells)

      @return width of the grid in cells
  */
  int getWidth();

  /**
      Get cell position X coordinate from X-direction distance
      from arena origin

      @param x    real distance [m]
      @return position of the cell in the grid
  */
  int getX(double x);

  /**
      Get cell position Y coordinate from Y-direction distance
      from arena origin

      @param y    real distance [m]
      @return position of the cell in the grid
  */
  int getY(double y);

  /**
      Get real world (arena) position X coordinate from X cell
      coordinate

      @param x    position of the cell in the grid
      @return real distance [m]
  */
  double getPosX(int x);

  /**
      Get real world (arena) position Y coordinate from Y cell
      coordinate

      @param y    position of the cell in the grid
      @return real distance [m]
  */
  double getPosY(int y);

  /**
      Update probability of each cell on the line from robot's position
      to the end point determined by a laser scanner. Needs to be modified by students

      @param r_x  Robot position X coordinate
      @param r_y  Robot position Y coordinate
      @param c_x  Endpoint of the measured scan X coordinate
      @param c_y  Endpoint of the measured scan Y coordinate
      @param dist Measured distance in [m]
  */
  void updateCells(int r_x, int r_y, int c_x, int c_y, double dist);

/** 
    Copies the underlaying grid structure into a a new array. The array has to be allocated already. 
 */
 
  void copyTo(double *grid);  

 private:
  int height;         // height of the grid
  int width;          // width of the grid
  double cellSize;    // corresponding size of the cell (in meters)
  double *data;       // data array

  /**
      Swap value in x with value in y
      @param x    first swapped value
      @param y    second swapped value
  */
  void SWAP(int& x, int& y);
};
}  // namespace imr
#endif
