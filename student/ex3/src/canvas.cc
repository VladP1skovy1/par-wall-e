/*
 * File name: canvas.cc
 * Date:      Sun Feb 25 2018 15:55:11 GMT+0100 (CET)
 * Author:    Miroslav Kulich (kulich@cvut.cz)
 */

#include <cairomm/context.h>
#include <gtkmm.h>
#include <cmath>
#include <iostream>

#include "canvas.h"
#include "window.h"

using namespace imr;

CCanvas* CCanvas::instance = NULL;

CCanvas& CCanvas::getInstance() {
  if (!instance) {
    instance = new CCanvas();
  }
  return *instance;
}

// CCanvas constructor
CCanvas::CCanvas()
    : scale(1), grab(0, 0), shift(0, 0), height(0), width(0), grid(0) {
  add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK |
             Gdk::SCROLL_MASK | Gdk::SMOOTH_SCROLL_MASK |
             Gdk::BUTTON1_MOTION_MASK);
  scale = 0.5;
  shift.x = 100;

  color[Red] = CColor(1, 0, 0);
  color[Cyan] = CColor(0, 1, 1);
  color[Blue] = CColor(0, 0, 1);
  color[DarkBlue] = CColor(0, 0, 0.627);
  color[LightBlue] = CColor(0.678, 0.847, 0.902);
  color[Purple] = CColor(0.5, 0, 0.5);
  color[Yellow] = CColor(1, 1, 0);
  color[Lime] = CColor(0, 1, 0);
  color[Magenta] = CColor(1, 0, 1);
  color[White] = CColor(1, 1, 1);
  color[Silver] = CColor(0.75, 0.75, 0.75);
  color[Gray] = CColor(0.5, 0.5, 0.5);
  color[Black] = CColor(0, 0, 0);
  color[Orange] = CColor(1, 0.647, 0);
  color[Brown] = CColor(0.647, 0.165, 0.165);
  color[Marron] = CColor(0.5, 0, 0);
  color[Green] = CColor(0, 0.5, 0);
  color[Olive] = CColor(0.5, 0.5, 0);

  //     Red #FF0000 White #FFFFFF Cyan #00FFFF Silver #C0C0C0 Blue #0000FF Gray
  //     or
  // Grey #808080 DarkBlue #0000A0 Black #000000 LightBlue #ADD8E6 Orange
  // #FFA500 Purple #800080 Brown #A52A2A Yellow #FFFF00 Maroon #800000 Lime
  // #00FF00 //Green #008000 Magenta #FF00FF Olive #808000
};

CCanvas::~CCanvas() {
  if (grid != 0) {
    delete[] grid;
  }
}

void CCanvas::draw(CMapGrid& map) {
  std::lock_guard<std::mutex> lk(mtx);
  height = map.getHeight();
  width = map.getWidth();
  if (grid != 0) {
    delete[] grid;
  }
  grid = new double[height * width];
  map.copyTo(grid);
}

bool CCanvas::on_scroll_event(GdkEventScroll* ev) {
  // Update scale according to mouse scroll
  scale -= ev->delta_y / 10.0;
  if (scale < 0.1) {
    scale = 0.1;
  }
  queue_draw();
  return true;
}

bool CCanvas::on_button_press_event(GdkEventButton* event) {
  if ((event->type == GDK_BUTTON_PRESS) && (event->button == 1)) {
    grab.x = event->x;
    grab.y = event->y;
  }

  return true;
}

bool CCanvas::on_button_release_event(GdkEventButton* event) {
  if ((event->type == GDK_BUTTON_RELEASE) && (event->button == 1)) {
    shift.x += event->x - grab.x;
    shift.y += event->y - grab.y;
    queue_draw();
  }

  return true;
}

bool CCanvas::on_motion_notify_event(GdkEventMotion* event) {
  shift.x += event->x - grab.x;
  shift.y += event->y - grab.y;
  grab.x = event->x;
  grab.y = event->y;
  queue_draw();
  // std::cout << "MOVE: " << event->x << " " << event->x << std::endl;
  return true;
}

bool CCanvas::on_draw(const Cairo::RefPtr<Cairo::Context>& cr) {
  std::lock_guard<std::mutex> lk(mtx);
  // This is where we draw on the window
  // Gtk::Allocation allocation = get_allocation();
  // const int ww = allocation.get_width();
  // const int hh = allocation.get_height();
  // const int lesser = MIN(ww, hh);
  cr->translate(shift.x, shift.y);
  cr->scale(scale, scale);
  // coordinates for the center of the window
  // int xc, yc;
  // xc = width / 2;
  // yc = height / 2;

  // background
  // cr->save();
  // cr->set_source_rgb(1.0, 1.0, 1.0);
  // cr->paint();
  // cr->restore();

  cr->save();
  const int CELL_SIZE = 10;
  double val;
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      val = grid[y * width + x];
      if (val >= 0 && val <= 1) {
        cr->set_source_rgb(1 - val, 1 - val, 1 - val);
      } else {
        cr->set_source_rgb(color[int(val)].r, color[int(val)].g,
                           color[int(val)].b);
      }
      cr->rectangle(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
      cr->fill();
    }
  }
  cr->restore();

  cr->save();
  cr->set_source_rgb(0.9, 0.9, 0.9);
  cr->set_line_width(0.3);
  for (int i = 0; i <= width; i++) {
    cr->move_to(i * CELL_SIZE, 0 * CELL_SIZE);
    cr->line_to(i * CELL_SIZE, height * CELL_SIZE);
    cr->stroke();
  }

  for (int i = 0; i <= height; i++) {
    cr->move_to(0 * CELL_SIZE, i * CELL_SIZE);
    cr->line_to(width * CELL_SIZE, i * CELL_SIZE);
    cr->stroke();
  }

  cr->restore();

  cr->save();
  cr->set_line_width(4);
  for(CLine l:lines) {
    cr->set_source_rgb(color[l.color].r, color[l.color].g, color[l.color].b);
    cr->move_to((l.x1+0.5) * CELL_SIZE, (l.y1+0.5) * CELL_SIZE);
    cr->line_to( (l.x2+0.5) * CELL_SIZE, (l.y2+0.5) * CELL_SIZE);
    cr->stroke();
  }
  cr->restore();
  return true;
}

void CCanvas::drawPixel(int x, int y, Color color) {
  std::lock_guard<std::mutex> lk(mtx);
  grid[y * width + x] = color;
}

void CCanvas::drawLine(int x1, int y1, int x2, int y2, Color color) {
  lines.push_back(CLine(x1,y1,x2,y2,color));
} 


void CCanvas::redraw() {
  auto win = get_window();
  if (win) {
    Gdk::Rectangle r(0, 0, get_allocation().get_width(),
                     get_allocation().get_height());
    win->invalidate_rect(r, false);
  }
}

void CCanvas::stop() {
  auto win = get_window();
  win->hide();
  // Gtk::Main::quit();
}

/* end of canvas.cc */
