/*
 * File name: window.cc
 * Date:      Sun Feb 25 2018 16:34:10 GMT+0100 (CET) 
 * Author:    Miroslav Kulich (kulich@cvut.cz)
 */


#include "window.h"
#include <iostream>

using namespace imr;

//CWindow constructor 
CWindow::CWindow() {
   add_events(Gdk::KEY_PRESS_MASK);
};

CWindow::~CWindow() {
};

bool CWindow::on_key_press_event(GdkEventKey* event)
{
  std::cout << "PRESS" << std::endl;
    if(event->type == GDK_KEY_PRESS && ( event->keyval == GDK_KEY_Escape || event->keyval == GDK_KEY_q)) {
    hide();
    return true;
  }
  return Gtk::Window::on_key_press_event(event);
}

/* end of window.cc */
