/*
 * File name: main.cc
 * Date:      2018/10/02 23:52
 * Author:    Miroslav Kulich
 */

#include <stdio.h>
#include <gtkmm/application.h>
#include <gtkmm/window.h>
#include <thread>

#include "map_grid.h"
#include "canvas.h"
#include "window.h"

#include "control.h"
#include "exploration.h"


int main(int argc, char **argv) {
  auto app = Gtk::Application::create("cz.cvut.ciirc.imr.par.ex3");

  imr::CWindow win;
  imr::CCanvas &canvas = imr::CCanvas::getInstance();

  win.set_title("PAR - path planner");
  win.set_default_size(600, 600);
  win.add(canvas);
  canvas.show();
  imr::CMapGrid grid(200,200,0.05);
  imr::CControl control(grid);
  imr::CExploration explorer(grid,control);  

  std::thread t1(&imr::CControl::main,&control);
  std::thread t2(&imr::CExploration::main,&explorer);
  app->run(win, argc, argv);
  control.stop();
  explorer.stop();
  t1.join();
  t2.join();


  return 0;
}
