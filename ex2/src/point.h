/*
 * File name: point.h
 * Date:      Wed Mar 28 2018 22:41:28 GMT+0200 (CEST) 
 * Author:    Miroslav Kulich (kulich@cvut.cz)
 */


#ifndef __IMR_POINT__
#define __IMR_POINT__


namespace imr {

 
class CPoint {
    public:
        CPoint() {};
        CPoint(double x, double y);
        double dist(CPoint &p);
        
        double x;
        double y;
};

} //end namespace imr

#endif

/* end of point.h */
