/***************************************************************************

    file                 : dm1.cpp
    created              : 2019年 03月 30日 星期六 01:14:16 CST
    copyright            : (C) 2002 lidong

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


/* 
 * Module entry point  
 */ 
extern "C" int 
dm1(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("dm1");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
} 

extern float* pdm1_x;
extern float* pdm1_y;
extern float* pdm1_dist_raced;
extern float* pdm1_speed;
extern float* pdm1_to_track_middle_m;
/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 
    float angle;
    float set_speed = 50 + 3;
    angle = RtTrackSideTgAngleL(&car->_trkPos) - car->_yaw;
    NORM_PI_PI(angle);
    angle -= ((car->_trkPos.toMiddle + 4) / car->_trkPos.seg->width);
    car->ctrl.steer = angle / car->_steerLock;
    car->ctrl.gear = 1; // first gear
    car->ctrl.brakeCmd = 0;

    float dimx = car->_dimension_x; // vehicle half long
	float dimy = car->_dimension_y; // vehicle half width

    float car_x = car->_pos_X;
    float car_y = car->_pos_Y;
    float dist_raced = car->_distRaced + 85;
    float speed_x = car->_speed_x;
    float to_track_middle_m = car->_trkPos.toMiddle;
    float dist_to_track_middle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width);
    *pdm1_x = car_x;
    *pdm1_y = car_y;
    *pdm1_dist_raced = dist_raced;
    *pdm1_speed = speed_x;
    *pdm1_to_track_middle_m = to_track_middle_m;
    // printf("to_track_middle_m: %.4f, to_track_middle = %.4f\n", to_track_middle_m, dist_to_track_middle);
	// printf("human car_x = %f, car_y = %f, dimx = %f, dimy =%f\n", car_x, car_y, dimx, dimy);
    // printf("TORCS: car1_dist_raced = %.3f\n", *pcar1_dist_raced);

    if (car->_speed_x < set_speed/3.6 - 0.5)
    {
        car->ctrl.accelCmd = 1;
        car->ctrl.brakeCmd = 0;
    }
    else if (car->_speed_x > set_speed/3.6 + 0.5)
    {
        car->ctrl.accelCmd = 0;
        car->ctrl.brakeCmd = 0.3;
    }
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}

