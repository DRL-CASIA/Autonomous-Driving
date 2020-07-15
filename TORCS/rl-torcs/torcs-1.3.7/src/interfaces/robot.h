/***************************************************************************

    file                 : robot.h
    created              : Sun Jan 30 22:59:40 CET 2000, 2002
    copyright            : (C) 2000-2014 by Eric Espie, Bernhard Wymann
    email                : torcs@free.fr
    version              : $Id: robot.h,v 1.10.2.3 2014/05/17 04:50:39 berniw Exp $

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
    Robot Module Interface Definition
    @author	<a href=mailto:eric.espie@torcs.org>Bernhard Wymann, Eric Espie</a>
    @version	$Id: robot.h,v 1.10.2.3 2014/05/17 04:50:39 berniw Exp $
    @ingroup	robotmodint
*/

/**
   @defgroup robotmodint Robot Module Interface
   @brief Interface for robot modules, robot modules are discovered and loaded during runtime.
   
   Multiple robot modules can be loaded at the same time, one robot module can host up to 10
   instances of a robot.
   This is the call flow of the robots callbacks during a race event.
   <br>The square boxes are for the race manager and the ellipses are for the robot.
   @image	html robot_call.gif
   
   A detailed robot tutorial is available, visit [www.torcs.org](http://www.torcs.org) for details.
   
   @ingroup	modint
*/
 
#ifndef _ROBOTV1_H_
#define _ROBOTV1_H_

#include <raceman.h>

#define ROB_IDENT	0

/** Callback function prototype for robot module, give the robot the track view, called for every track change or new race
 *  @ingroup robotmodint
 *  @param[in] index Index of the robot instance
 *  @param[in] track Track
 *  @param[in] carhandle Original car parameter set, intended for read only by the robot
 *  @param[out] myCarSettings Robot instance specific parameter set
 *  @param[in] s Situation
 */
typedef void (*tfRbNewTrack)(int index, tTrack *track, void *carHandle, void **myCarSettings, tSituation *s);

/** Callback function prototype for robot module, initialization for new race
 *  @ingroup robotmodint
 *  @param[in] index Index of the robot instance
 *  @param[in] car Car
 *  @param[in] s Situation
 */
typedef void (*tfRbNewRace) (int index, tCarElt *car, tSituation *s);

/** Callback function prototype for robot module, teardown after race, this is currently NOT called by TORCS
 *  @ingroup robotmodint
 *  @param[in] index Index of the robot instance
 *  @param[in] car Car
 *  @param[in] s Situation
 */
typedef void (*tfRbEndRace) (int index, tCarElt *car, tSituation *s);

/** Callback function prototype for robot module, driving the car
 *  @ingroup robotmodint
 *  @param[in] index Index of the robot instance
 *  @param[in,out] car Car, the tCarCtrl is modified to return the driving commands
 *  @param[in] s Situation
 */
typedef void (*tfRbDrive)   (int index, tCarElt *car, tSituation *s);

/** Callback function prototype for robot module, shutdown robot instance for given index
 *  @ingroup robotmodint
 *  @param[in] index Index of the robot instance
 */
typedef void (*tfRbShutdown)(int index);

/** Callback function prototype for robot module, handing over pit stop commands
 *  @ingroup robotmodint
 *  @param[in] index Index of the robot instance
 *  @param[in,out] car Car, the tCarPitCmd is modified to return the pitstop commands (refuel, repair, setup changes)
 *  @param[in] s Situation
 *  @return
 *  - #ROB_PIT_IM, immediate return from pit command
 *  - #ROB_PIT_MENU, call the interactive menu for pit command
 */
typedef int  (*tfRbPitCmd)  (int index, tCarElt* car, tSituation *s);

#define ROB_PIT_IM	0	/**< Immediate return from pit command */
#define ROB_PIT_MENU	1	/**< Call the interactive menu for pit command */


/** Interface Structure for Robots
    @ingroup robotmodint
*/
typedef struct RobotItf {
	tfRbNewTrack rbNewTrack;
	tfRbNewRace  rbNewRace;
	tfRbEndRace  rbEndRace;
	tfRbDrive	 rbDrive;
	tfRbPitCmd	 rbPitCmd;
	tfRbShutdown rbShutdown;
	int index;
} tRobotItf;



/*
 * Parameters definitions for driver
 */
#define ROB_SECT_ROBOTS		"Robots"

#define ROB_LIST_INDEX		"index"

#define ROB_ATTR_NAME		"name"
#define ROB_ATTR_TEAM		"team"
#define ROB_ATTR_DESC		"desc"
#define ROB_ATTR_AUTHOR		"author"
#define ROB_ATTR_CAR		"car name"
#define ROB_ATTR_CATEGORY	"category"
#define ROB_ATTR_RACENUM	"race number"
#define ROB_ATTR_RED		"red"
#define ROB_ATTR_GREEN		"green"
#define ROB_ATTR_BLUE		"blue"

#define ROB_ATTR_TYPE		"type"

#define ROB_VAL_HUMAN		"human"
#define ROB_VAL_ROBOT		"robot"

#define ROB_ATTR_LEVEL		"skill level"

#define ROB_VAL_ROOKIE		"rookie"
#define ROB_VAL_AMATEUR		"amateur"
#define ROB_VAL_SEMI_PRO	"semi-pro"
#define ROB_VAL_PRO		"pro"

#endif /* _ROBOTV1_H_ */ 



