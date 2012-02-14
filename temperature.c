//-------------------------------------------------------------------------
// Author: Brad Goold
// Date: 10 Feb 2012
// Email Address: W0085400@umail.usq.edu.au
// 
// Purpose:
// Pre:
// Post:
// RCS $Date$
// RCS $Revision$
// RCS $Source$
// RCS $Log$
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
// Included Libraries
//-------------------------------------------------------------------------
#include "FreeRTOS.h"
#include <stdio.h>
#include "ds1820.h"
//-------------------------------------------------------------------------
static void _heat_start(struct brew_task *bt)
{
    allOff();

    // kick off reading the temp
    vTaskEnterCritical();
    DS1820Init();
    DS1820Skip();
    DS1820Convert();
    vTaskExitCritical();

    vTaskDelay(1000);

    vTaskEnterCritical();
    DS1820ReadTemp();
    vTaskExitCritical();    

    // make sure we have consistent readings on the level probes
    level_wait_for_steady_readings();
}

//-------------------------------------------------------------------------

