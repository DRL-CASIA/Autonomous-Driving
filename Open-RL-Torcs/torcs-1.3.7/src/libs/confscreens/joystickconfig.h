/***************************************************************************

    file                 : joystickconfig.h
    created              : Wed Mar 21 23:06:29 CET 2001
    copyright            : (C) 2001 by Eric Espié
    email                : Eric.Espie@torcs.org
    version              : $Id: joystickconfig.h,v 1.3.2.1 2015/04/18 11:04:55 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 
/** @file    
    		
    @author	<a href=mailto:torcs@free.fr>Eric Espie</a>
    @version	$Id: joystickconfig.h,v 1.3.2.1 2015/04/18 11:04:55 berniw Exp $
*/

#ifndef _JOYSTICKCONFIG_H_
#define _JOYSTICKCONFIG_H_

extern void *JoyCalMenuInit(void *prevMenu, tCmdInfo *cmd, int maxcmd, void *parmHandle, const char* driverSection);

#endif /* _JOYSTICKCONFIG_H_ */ 



