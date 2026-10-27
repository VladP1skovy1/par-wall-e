/*
 * File name: point.cc
 * Date:      Wed Mar 28 2018 22:42:00 GMT+0200 (CEST) 
 * Author:    Miroslav Kulich (kulich@cvut.cz)
 */

#include <math.h>
#include "point.h"


using namespace imr;

//CCPoint constructor 
CPoint::CPoint(double x, double y): x(x), y(y) {
  
};

double CPoint::dist(CPoint &p) {
  double xx = x - p.x;
  double yy = y - p.y;
  return sqrt(xx*xx+yy*yy);
}
/* end of cpoint.cc */
