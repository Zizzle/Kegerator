///////////////////////////////////////////////////////////////////////////////
// RCS $Source: /home/brad/Documents/AVR/brewbot/RCS/menu.c,v $
// Copyright (C) 2007, Matthew Pratt
//
// Licensed under the GNU General Public License v3 or greater
//
// Date: 21 Jun 2007
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "touch.h"
#include "menu.h"
#include "queue.h"
#include "lcd.h"
#include "console.h"
#include "crane.h"
#define HEIGHT 6


char buf[30];
static struct menu  *g_menu[MAX_DEPTH];

static unsigned char g_crumbs[MAX_DEPTH];

static unsigned char g_item = 0; //this is the item on the screen 
                                 //from the top item down. ie top = 0 

static unsigned char g_index = 0;//This is the index to the depth of the 
                                 //menu. ie main = 0. 

static void (*g_menu_applet)(uint16_t x, uint16_t y) = NULL;

static void menu_update_hilight(void)
{
    // static short old = 0; // keep track of the old hilight so we can clear it
    //lcd_clear_pixels(0, HILIGHT_Y(old),        HILIGHT_W, HILIGHT_H);
    //lcd_set_pixels  (0, HILIGHT_Y(g_item + 1), HILIGHT_W, HILIGHT_H);
    // old = g_item + 1;
}

static void menu_run_callback(void)
{
    // if this function is called, and g_index (which is the index 
    // of the menu depth.ie main menu = 0) is greater than 0, then the 
    // menu in the lower level is called. 
    if (g_index > 0)
    {
        void (*callback)(void) = g_menu[g_index - 1][g_crumbs[g_index - 1]].activate;
     
        if (callback)
            callback(); //this is going to be the menu before the current
                        //menu's display. 
     }
}

void menu_update(void)
{
    unsigned char ii;
    
    lcd_menu_update(g_menu[g_index]);
    return;
                                 
}

void menu_set_root(struct menu *root_menu)
{
    //the menu at index 0 is the root. so make it so when called.
    g_menu[0] = root_menu;
    // menu_update();// dont enable this whilst set_root is outside scheduler.
}

//----------------------------------------------------------------
// When the user has selected an applet all the keys come here.
// We offer the key to the applet. If the back key is pressed then
// we disable keys coming here.
//----------------------------------------------------------------
static void menu_applet_key(uint16_t x, uint16_t y)
{
    g_menu_applet(x,y);
    if (touchIsInWindow(x,y, 160, 0, 230,100) == pdTRUE)
    {
        menu_clear();
        menu_update();
        g_menu_applet = NULL;
        menu_run_callback();
    }
}

void menu_key(uint16_t x, uint16_t y)
{
    // If this function is called, we definitely have a negative edge
    // on the touch screen so no need to check for that here


    if (g_menu_applet)
       {
           menu_applet_key(x,y);
        return;
     }

  
    uint16_t window = 0;
    static uint16_t last_window = 0; 
    
    if (touchIsInWindow(x,y, 0,0, 150,50) == pdTRUE)
        window = 0;
    
    else  if (touchIsInWindow(x,y, 0,50, 150,100) == pdTRUE)
        window = 1;
    
    
    else  if (touchIsInWindow(x,y, 0,100, 150,150) == pdTRUE)
        window = 2;

    
    else  if (touchIsInWindow(x,y, 0,150, 150,200) == pdTRUE)
        window = 3;
    
    else  if (touchIsInWindow(x,y, 0,200, 150,250) == pdTRUE)
        window = 4;
    
    
    else  if (touchIsInWindow(x,y, 0,250, 150,300) == pdTRUE)
        window = 5;
    
    else  if (touchIsInWindow(x,y, 160, 0, 230,100) == pdTRUE)
        window = 6;

    else  if (touchIsInWindow(x,y, 160, 100, 230,200) == pdTRUE)
        window = 7;

    else window = 255;

    
    //Back Button
    if (window == 6)
    {
        if (g_index > 0)
            g_index--;
        g_item = 0;
        menu_update();
        menu_run_callback();
        return;
    }
  
    if (window == 7){
  
  
    }
  
    if (window < 6)
    {
       
      
        if (g_menu[g_index][g_item+1].text != NULL)
            g_item = window;
        
        else {
            g_item = 0;
            return;
        }
        g_crumbs[g_index] = g_item;
        //first set the callback to the current menu's 
        //"activate" element. Ie this is where to 
        //call back to when we enter the next menu
        void (*callback)(void) = g_menu[g_index][g_item].activate;
        
        
        if (g_menu[g_index][g_item].next && g_index < MAX_DEPTH)
        {
            
            g_index++;   
            g_menu[g_index] = g_menu[g_index-1][g_item].next; //change menu
            g_item = 0; //clear g_item so that we can replace contents
            menu_update(); // update menu
        }
        else if (g_menu[g_index][g_item].key_handler)
        {
            g_menu_applet = g_menu[g_index][g_item].key_handler;
            menu_clear();
        }
        
        // run the callback which should start the applet or update the display
        if (callback)
        {
            callback();
        }
    }
    g_item = 0;
    
}

void menu_clear(void)
{
    lcd_clear(Black);
}

void menu_run_applet(void (*applet_key_handler)(uint16_t x, uint16_t y))
{
    g_menu_applet = applet_key_handler;
    menu_clear();
}
