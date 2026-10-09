/*
 * File name: main.cc
 * Date:      2019/09/30
 * Author:    Miroslav Kulich
 */

#include <gtkmm/application.h>
#include <gtkmm/window.h>
#include <fstream>
#include <iostream>
#include <thread>

#include "canvas.h"
#include "map_grid.h"
#include "planner.h"
#include "window.h"

const std::string PROGRAM_VERSION = "0.1";
constexpr int ROBOT_RADIUS = 5;

void drawPath(imr::CCanvas &canvas, const RobotPath &path, imr::Color color) {
    for (const auto &p : path) {
        canvas.drawPixel(p.first, p.second, color);
    }
}

void drawPathBetweenAnchors(imr::CCanvas &canvas, const RobotPath &path, imr::Color color) {
    for (size_t i = 0; i < path.size() - 1; i++) {
        canvas.drawLine(path[i].first, path[i].second, path[i + 1].first, path[i + 1].second, color);
    }
}

void planning(std::pair<int, int> start, std::pair<int, int> finish) {
    imr::CMapGrid gridMap("../maps/arena.txt");
    imr::CPlanner planner;
    imr::CCanvas &canvas = imr::CCanvas::getInstance();

    planner.setMap(gridMap);

    auto paths = planner.plan(start.first, start.second, finish.first, finish.second, ROBOT_RADIUS);
    canvas.draw(planner.getMap());
    canvas.drawLine(start.first, start.second, finish.first, finish.second, imr::Red);

    drawPath(canvas, paths.first, imr::Blue);
    drawPathBetweenAnchors(canvas, paths.second, imr::Green);

    canvas.redraw();
}

/// ----------------------------------------------------------------------------
/// Main PROGRAM
/// ----------------------------------------------------------------------------
int main(int argc, char **argv) {
    auto app = Gtk::Application::create("cz.cvut.ciirc.imr.par.ex2");

    imr::CWindow win;
    imr::CCanvas &canvas = imr::CCanvas::getInstance();

    win.set_title("PAR - path planner");
    win.set_default_size(1600, 1400);
    win.add(canvas);
    canvas.show();
    std::pair<int, int> start;
    std::pair<int, int> finish;

    start = std::make_pair(50, 50);
    finish = std::make_pair(150, 145);

    std::thread t1(planning, start, finish);
    app->run(win, argc, argv);
    t1.join();
}

/* end of main.cc */
