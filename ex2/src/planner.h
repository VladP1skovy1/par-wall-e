/*
 * File name: planner.h
 * Date:      2016-09-27
 * Author:    Miroslav Kulich, Lukáš Bertl
 */

#ifndef IMR_PLANNER
#define IMR_PLANNER

#include <string>
#include <vector>


#include "map_grid.h"

constexpr int STRAIGHT_COST = 10;
constexpr int DIAGONAL_COST = 14;
/**
    Planner class is essential class for users. Here the Dijkstra's algorithm
    shall be implemented with all the necessary features to achieve final path
    plan. By default it does not contain many methods, because is shall be done
    by user.
*/

using RobotPath = std::vector<std::pair<int, int> >;

namespace imr {
    class CPlanner {
    private:
        CMapGrid *map; // map over which planner makes the plans
        void inflateMap(int radius);

        /**
            Dijkstra's algorithm. Plans over inflated map, therefore it
            is necessary to inflate it first.

            @return true if path exists, false otherwise
        */
        std::vector<std::pair<int, int> > dijkstra(std::pair<int, int> start, std::pair<int, int> finish);

        /**
            Takes the plan produced by Dijkstra and makes smooth path.
            Smooth path is stored back to plan, goal is first element,
            start is the last one
        */
        std::vector<std::pair<int, int> > smooth_path(const std::vector<std::pair<int, int> > &path);

    public:
        /**
            Contructor. Initializes the internal configuration cfg with values
            from cfg given as parameter.
        */
        CPlanner();


        /**
            Map setter. Assign new map to private map and
            create new infmap to fit the map's size
        */
        void setMap(CMapGrid &new_map);


        /**
            Method to be done by users. This method makes the plan with starting
            and goal position stored in the configuration cfg over the map.
            It should contain calls for map inflation, Dijkstra's algorithm and
            path smoothing.

            Implementation of this method and structure of the final plan is up to
            user himself. Hint: generaly good solution is to create a new class
            representing the plan. This class can contain many useful functions like
            get goal position, get next position etc.

            @return true if path was found, false otherwise (goal position is not
                    reachable)
        */
        std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>> plan(
            int x0, int y0, int x1, int y1, int robot_radius);

        /**
            Bresenham algorithm for drawing a straight line. Output of the method is
            not defined (it does not draw anywhere, just generates [X,Y] points of
            the line). There is TODO line in the lower part of the method for user to
            implement required functionality.

            EDIT: the method has to be adjusted for smoothing path purposes. It walks through
            points on a line between two points. If it hits a wall, it returns false
            as an indication that those points are not straight visible

            @param x0   X coordinate of the first (starting) point
            @param y0   Y coordinate of the first (starting) point
            @param x1   X coordinate of the second (goal) point
            @param y1   Y coordinate of the second (goal) point

            @return true if there is no wall between given point
        */
        bool bresenham(int x0, int y0, int x1, int y1);

        CMapGrid &getMap() { return *map; }
    };
}

#endif
