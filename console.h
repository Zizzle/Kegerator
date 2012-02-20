///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, Brad Goold, All Rights Reserved.
//
// Authors: Brad Goold
//
// Date: 14 Feb 2012
//
// RCS $Date$
// RCS $Revision$
// RCS $Source$
// RCS $Log$///////////////////////////////////////////////////////////////////////////////

#ifndef CONSOLE_H
#define CONSOLE_H

#define mainMESSAGE_QUEUE_SIZE                                  ( 30 )

extern xQueueHandle xConsoleQueue;

void vTerminalMessagesTask( void *pvParameters );

#endif
