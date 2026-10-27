/*
 * File name: canvas.h
 * Date:      Sun Feb 25 2018 15:57:01 GMT+0100 (CET)
 * Author:    Miroslav Kulich (kulich@cvut.cz)
 */

#ifndef __IMR_CANVAS__
#define __IMR_CANVAS__

#include <gtkmm/drawingarea.h>
#include <mutex>
#include "map_grid.h"
#include "point.h"
namespace imr {

enum Color {
  Red = 2,
  Cyan,
  Blue,
  DarkBlue,
  LightBlue,
  Purple,
  Yellow,
  Lime,
  Magenta,
  White,
  Silver,
  Gray,
  Black,
  Orange,
  Brown,
  Marron,
  Green,
  Olive
};

// Color Names
//         Red #FF0000 White #FFFFFF Cyan #00FFFF Silver #C0C0C0 Blue #0000FF
//         Gray or
//     Grey #808080 DarkBlue #0000A0 Black #000000 LightBlue #ADD8E6 Orange
//     #FFA500 Purple #800080 Brown #A52A2A Yellow #FFFF00 Maroon #800000 Lime
//     #00FF00 Green #008000 Magenta #FF00FF Olive #808000

class CLine {
 public:
  int x1, y1, x2, y2;
  int color;
  CLine(int x1, int y1, int x2, int y2, int color)
      : x1(x1), y1(y1), x2(x2), y2(y2), color(color){};
};

class CCanvas : public Gtk::DrawingArea {
 public:
  static CCanvas& getInstance();
  virtual ~CCanvas();
  bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
  void draw(CMapGrid& grid);
  void drawPixel(int x, int y, Color color);
  void drawLine(int x1, int y1, int x2, int y2, Color color); 
  void redraw();
  void stop();

 protected:
  bool on_scroll_event(GdkEventScroll* ev) override;
  bool on_button_press_event(GdkEventButton* event) override;
  bool on_button_release_event(GdkEventButton* event) override;
  bool on_motion_notify_event(GdkEventMotion* motion_event) override;

 private:
  double scale;
  CPoint grab;
  CPoint shift;
  int height;
  int width;
  double* grid;
  std::mutex mtx;
  std::vector<CLine> lines;
  static CCanvas* instance;  // Instance of the window itself (NULL if not
                             // initialized, else window object is stored)
  CCanvas();

  /**
       Auxiliary structure packing three pixel colors RGB to one object
   */
  struct CColor {
    double r, g, b;

    CColor() {
      r = 0;
      g = 0;
      b = 0;
    }

    CColor(double r, double g, double b) {
      this->r = r;
      this->g = g;
      this->b = b;
    }
  };

  CColor color[100];
};

}  // end namespace imr

#endif

/* end of canvas.h */
