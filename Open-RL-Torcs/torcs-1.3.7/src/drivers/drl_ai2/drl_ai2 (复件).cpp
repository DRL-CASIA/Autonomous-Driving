 /***************************************************************************

    file                 : drl_ai2.cpp
    created              : 2017年 06月 13日 星期二 14:53:12 CST
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
drl_ai2(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("drl_ai2");		/* name of the module (short) */
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

// counter
static int stuck = 0;
/* check if the car is stuck */
bool isStuck(tCarElt* car)
{
    float angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle);
    // angle smaller than 30 degrees?
    if (fabs(angle) < 30.0/180.0*PI) {
        stuck = 0;
        return false;
    }
    if (stuck < 100) {
        stuck++;
        return false;
    } else {
        return true;
    }
}

static int step_counter = 0; // used for schedule speed 
static int inc = 300;
// speed schedule
float get_desired_speed() {
    if (step_counter < inc) return 44/3.6;
    if (step_counter >= inc && step_counter < 2*inc) return 40/3.6;
    if (step_counter >= 2*inc && step_counter < 3*inc) return 36/3.6;
    if (step_counter >= 3*inc && step_counter < 4*inc) return 40/3.6;
    if (step_counter == 4*inc) {
        step_counter = 0;
        return 44/3.6;
    }
}

float getDistToSegStart(tCarElt *ocar)
{
    if (ocar->_trkPos.seg->type == TR_STR) {
        return ocar->_trkPos.toStart;
    } else {
        return ocar->_trkPos.toStart*ocar->_trkPos.seg->radius;
    }
}


double distance;
double desired_speed = 45/3.6;
/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 
    float angle;
    const float SC = 1.0;

    for (int i = 0; i < s->_ncars; i++) {
        if (s->cars[i] != car) {
            distance = s->cars[i]->_trkPos.seg->lgfromstart + getDistToSegStart(s->cars[i]) - car->_distFromStartLine;
            if (distance > curTrack->length/2.0) {
                distance -= curTrack->length;
            } else if (distance < -curTrack->length/2.0) {
                distance += curTrack->length;
            }
        }
    }


    if (isStuck(car)){
        angle = -RtTrackSideTgAngleL(&(car->_trkPos)) + car->_yaw;
        NORM_PI_PI(angle); // put the angle back in the range from -PI to PI
        car->ctrl.steer = angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } else {
        angle = RtTrackSideTgAngleL(&car->_trkPos) - car->_yaw;
        NORM_PI_PI(angle);
        angle -= SC*(car->_trkPos.toMiddle / car->_trkPos.seg->width);

        car->ctrl.steer = angle / car->_steerLock;
        car->ctrl.gear = 1; // first gear
        car->ctrl.accelCmd = 0.2;
        car->ctrl.brakeCmd = 0.0;
        // printf("seg width: %.4f | toMiddle: %.4f\n", car->_trkPos.seg->width, car->_trkPos.toMiddle);
        if (car->_speed_x>desired_speed) {
           car->ctrl.brakeCmd=0.2;
           car->ctrl.accelCmd=0.0;
        }
        else if  (car->_speed_x<desired_speed) {
           car->ctrl.accelCmd=0.2;
           car->ctrl.brakeCmd=0.0;
        }


    }
    // printf("step_counter: %d, acc: %f, speed: %.5f\n", step_counter, car->ctrl.accelCmd, car->_speed_x);
    step_counter += 1;
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

