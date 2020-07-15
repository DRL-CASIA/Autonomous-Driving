/***************************************************************************

    file                 : lqr_bot.cpp
    created              : Mon Dec 11 17:17:40 CST 2017
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
#include <stdint.h>

// For killing torcs in this file
#include <sys/shm.h>

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
lqr_bot(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("lqr_bot");		/* name of the module (short) */
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

extern uint8_t* pwritten;
extern uint8_t* prestart;
extern uint8_t* prelaunch;
extern uint8_t* ptrk_type;
extern float* psteer;
extern float* pacc;
extern float* pbrake;
extern float* pspeed;
extern float* pto_track_middle; 
extern float* pangle;
extern float* pdist_raced;
extern float* ptoMarking_L;
extern float* ptoMarking_M;
extern float* ptoMarking_R;
extern float* ptoMarking_ML;
extern float* ptoMarking_MR;
extern void* shm;
/* Drive during race. */
float angle_prev = 0.0;
float to_track_middle_prev = 0.0;
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 

    float dt = 0.05;
    float angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle);
    // angle *= 180.0/3.1415926;
    float angle_rate = (angle - angle_prev) / dt;
    float to_track_middle = car->_trkPos.toMiddle;
    float to_track_middle_norm = 2*to_track_middle / (car->_trkPos.seg->width);
    float to_track_middle_rate = (to_track_middle - to_track_middle_prev) / dt;
    angle_prev = angle;
    to_track_middle_prev = to_track_middle;
    *pangle = angle;
    *pdist_raced = angle_rate;
    *pto_track_middle = to_track_middle;
    *pspeed = to_track_middle_rate;
    *ptoMarking_L = to_track_middle_norm;
    *ptoMarking_M = car->_speed_x;

    *pwritten = 1;
	while(*pwritten == 1){
		usleep(1);
	}
    
    car->ctrl.steer = *psteer;
    car->ctrl.gear = 1;
    car->ctrl.accelCmd = 0.2; //*pacc;
    car->ctrl.brakeCmd = 0.0; //*pbrake;
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

