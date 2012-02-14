///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, Brad Goold, All Rights Reserved.
//
// Authors: Brad Goold
//
// Date:  6 Feb 2012
//
// RCS $Date$
// RCS $Revision$
// RCS $Source$
// RCS $Log$///////////////////////////////////////////////////////////////////////////////

#ifndef TOUCH_H
#define TOUCH_H

void vTouchTask( void *pvParameters ) ;
void Touch_Initializtion(void);
uint16_t Touch_GetPhyX(void);
uint16_t Touch_GetPhyY(void);
uint16_t  Touch_MeasurementX(void);
uint16_t  Touch_MeasurementY(void);



typedef struct 
{
    unsigned int uiX;
    unsigned int uiY;
} TP_PosData;

TP_PosData TP_PD; 

#endif
