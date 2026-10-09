/*
 * File name: planner.cc
 * Date:      2016-11-01
 * Author:    Miroslav Kulich, Lukáš Bertl
 */

#include "planner.h"

#include <climits>
#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>

#include "imr-h/logging.h"

using namespace imr;


CPlanner::CPlanner() {
}

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
    const int endX = x1, endY = y1;
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
    int twoDyTwoDx = twoDy - 2 * dx; // 2*Dy - 2*Dx
    int e = twoDy - dx; // 2*Dy - Dx
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
        if (map->getCell(xDraw, yDraw) >= CMapGrid::WALL) return false;
        if (e > 0) {
            e += twoDyTwoDx;
            y = y + ystep;
        } else {
            e += twoDy;
        }
    }

    return map->getCell(endX, endY) < CMapGrid::WALL;
}


void CPlanner::setMap(CMapGrid &new_map) {
    this->map = new CMapGrid(new_map);
}



void CPlanner::inflateMap(int radius) {
    const int w = map->getWidth();
    const int h = map->getHeight();

    auto original_map = new CMapGrid(*map);

    RobotPath offsets;
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            if (dx * dx + dy * dy > radius * radius) continue;
            offsets.emplace_back(dx, dy);
        }
    }

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (original_map->getCell(x, y) < CMapGrid::WALL) continue;
            for (const auto &offset: offsets) {
                int nx = x + offset.first;
                int ny = y + offset.second;
                if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                    map->setCell(nx, ny, CMapGrid::WALL);
                }
            }
        }
    }
}


RobotPath CPlanner::dijkstra(std::pair<int, int> start, std::pair<int, int> finish) {
    const int width = map->getWidth();
    const int height = map->getHeight();
    const int max_int = std::numeric_limits<int>::max();

    auto inside = [&](int x, int y) { return x >= 0 && x < width && y >= 0 && y < height; };
    auto blocked = [&](int x, int y) { return map->getCell(x, y) >= CMapGrid::WALL; };

    RobotPath path;

    if (!inside(start.first, start.second) || !inside(finish.first, finish.second)) return path;
    if (blocked(start.first, start.second) || blocked(finish.first, finish.second)) return path;

    std::priority_queue<std::pair<int, int>, RobotPath, std::greater<std::pair<int, int> > > pq;

    std::vector<std::vector<int> > dist(width, std::vector<int>(height, max_int));
    std::vector<RobotPath > prev(width, RobotPath(height, {-1, -1}));

    dist[start.first][start.second] = 0;
    pq.emplace(0, start.second * width + start.first);

    while (!pq.empty()) {
        const int d = pq.top().first;
        const int node = pq.top().second;
        pq.pop();

        const int x = node % width;
        const int y = node / width;

        if (d > dist[x][y]) continue;
        if (x == finish.first && y == finish.second) break;

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                const int nx = x + dx;
                const int ny = y + dy;
                if (!inside(nx, ny) || blocked(nx, ny)) continue;

                const int nd = d + ((dx != 0 && dy != 0) ? DIAGONAL_COST : STRAIGHT_COST);
                if (nd < dist[nx][ny]) {
                    dist[nx][ny] = nd;
                    prev[nx][ny] = {x, y};
                    pq.emplace(nd, ny * width + nx);
                }
            }
        }
    }

    if (dist[finish.first][finish.second] == max_int) return path;


    for (std::pair<int, int> c = finish; c.first != -1; c = prev[c.first][c.second]) {
        path.push_back(c);
    }
    return path;
}

RobotPath CPlanner::smooth_path(const RobotPath &path) {
    if (path.size() <= 2) return path;

    RobotPath smooth;
    size_t anchor = 0;
    smooth.push_back(path[anchor]);

    for (size_t i = 1; i < path.size(); i++) {
        if (bresenham(path[anchor].first, path[anchor].second,
                      path[i].first, path[i].second) || i - 1 == anchor)
            continue;
        anchor = i - 1;
        smooth.push_back(path[anchor]);
    }
    smooth.push_back(path.back());
    return smooth;
}

std::pair<RobotPath, RobotPath> CPlanner::plan(int x0, int y0, int x1, int y1, int robot_radius) {
    inflateMap(robot_radius);
    auto path = dijkstra(std::make_pair(x0, y0), std::make_pair(x1, y1));
    auto smoothed_path = smooth_path(path);
    return std::make_pair(path, smoothed_path);
}
