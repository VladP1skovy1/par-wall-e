/*
 * File name: window.h
 * Date:      Sun Feb 25 2018 16:34:01 GMT+0100 (CET) 
 * Author:    Miroslav Kulich (kulich@cvut.cz)
 */


#ifndef __IMR_WINDOW__
#define __IMR_WINDOW__

#include <gtkmm.h>
#include <gtkmm/window.h>

namespace imr {

class CWindow : public Gtk::Window {
    public:
        CWindow();
        virtual ~CWindow();

    private:
        bool on_key_press_event(GdkEventKey* event) override;        
};

}; //end namespace imr

#endif

/* end of window.h */