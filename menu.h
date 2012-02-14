///////////////////////////////////////////////////////////////////////////////
// RCS $Date: 2010/12/05 03:19:39 $
// RCS $Revision: 2.0 $
// RCS $Source: /home/brad/Documents/AVR/brewbot/RCS/menu.h,v $
// Copyright (C) 2007, Matthew Pratt
//
// Licensed under the GNU General Public License v3 or greater
//
// Date: 21 Jun 2007
///////////////////////////////////////////////////////////////////////////////

#ifndef MENU_H
#define MENU_H

struct menu {
    const char *text;             //displayed on LCD screen
    struct menu *next;            //Pointer to the next menu (if called)
    void (*activate)(void);       //Pointer to the function called from here
    void (*key_handler)(uint16_t x, uint16_t y); //Ptr to the key handler called
};

#define MAX_DEPTH 10

void menu_set_root(struct menu *root_menu);
void menu_key(uint16_t x, uint16_t y);
void menu_clear(void);
void menu_run_applet(void (*applet_key_handler)(unsigned char));
void menu_update(void);
#endif
