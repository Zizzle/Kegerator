///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, Brad Goold, All Rights Reserved.
//
// Authors: Brad Goold
//
// Date:  7 Feb 2012
//
// RCS $Date$
// RCS $Revision$
// RCS $Source$
// RCS $Log$///////////////////////////////////////////////////////////////////////////////

#ifndef LEDS_H
#define LEDS_H

#define D1 GPIO_Pin_6 
#define D2 GPIO_Pin_7
#define D3 GPIO_Pin_13
#define D4 GPIO_Pin_6

#define D1PORT GPIOC
#define D2PORT GPIOC
#define D3PORT GPIOD
#define D4PORT GPIOD

void vLEDSet( GPIO_TypeDef *GPIO_PORT , 
              uint16_t GPIO_Pin, 
              unsigned portBASE_TYPE uxValue);

void vLEDToggle( GPIO_TypeDef *GPIO_PORT , uint16_t GPIO_Pin);

void vLEDInit( void );

#endif
