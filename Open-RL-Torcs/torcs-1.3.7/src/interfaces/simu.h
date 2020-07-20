/***************************************************************************

    file                 : simu.h
    created              : Sun Jan 30 12:02:05 CET 2000
    copyright            : (C) 2000-2014 by Eric Espie, Bernhard Wymann
    email                : torcs@free.fr
    version              : $Id: simu.h,v 1.7.2.4 2014/05/17 04:50:39 berniw Exp $

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
    Simulation Module Interface Definition
    @author	<a href=mailto:eric.espie@torcs.org>Bernhard Wymann, Eric Espie</a>
    @version	$Id: simu.h,v 1.7.2.4 2014/05/17 04:50:39 berniw Exp $
    @ingroup	simumodint
*/

/**
   @defgroup simumodint Simulation Module Interface
   @brief Interface for simulation modules, the simulation module is discovered and loaded during runtime.
   
   @ingroup	modint
*/
 
#ifndef _SIMUV1_H_
#define _SIMUV1_H_

#include <track.h>
#include <raceman.h>

#define SIM_IDENT	0

struct Situation;
struct RmInfo;

/** Callback function prototype for simulation module initialization (bring up module and hand over track and parameters)
 *  @ingroup simumodint
 *  @param[in] nbCars Number of cars to simulate
 *  @param[in] track Track to race on
 *  @param[in] fuelFactor Factor for fuel consumtion, e.g 1 for normal, 0 for none, etc.
 *  @param[in] damageFactor Factor for damage, e.g. 1 for normal, 0 for none, etc.
 */
typedef void (*tfSimInit)(int nbCars, tTrack* track, tdble fuelFactor, tdble damageFactor);

/** Callback function prototype for simulation configuration of a given car
 *  @ingroup simumodint
 *  @param[in] carElt Car to configure
 *  @param[in] reInfo Race manager info
 */
typedef void (*tfSimConfig)(tCarElt* carElt, struct RmInfo* reInfo);

/** Callback function prototype for simulation reconfiguration of a given car (refueling, setup adjustments)
 *  @ingroup simumodint
 *  @param[in] carElt Car to reconfigure
 */
typedef void (*tfSimReConfig)(tCarElt* carElt);

/** Callback function prototype for progressing the given Situation by a given simulation time step
 *  @ingroup simumodint
 *  @param[in,out] s Situation to progress
 *  @param[in] deltaTime Timestep, usually @ref RCM_MAX_DT_SIMU
 *  @param[in] telemetry Index of car to receive telemetry
 */
typedef void (*tfSimUpdate)(struct Situation* s, double deltaTime, int telemetry);

/** Callback function prototype for simulation module shutdown
 *  @ingroup simumodint
 */
typedef void (*tfSimShutdown)(void);

/** Interface Structure for Simulation
    @ingroup simumodint
*/
typedef struct
{
	tfSimInit		init;
	tfSimConfig		config;
	tfSimReConfig	reconfig;
	tfSimUpdate		update;
	tfSimShutdown	shutdown;
} tSimItf;



#endif /* _SIMUV1_H_ */ 



