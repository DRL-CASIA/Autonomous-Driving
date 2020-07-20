/***************************************************************************

    file                 : drl_ai1.cpp
    created              : 2017年 06月 13日 星期二 14:01:29 CST
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
#include <string> 
#include <iostream>
#include <sstream>
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
drl_ai1(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("drl_ai1");		/* name of the module (short) */
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

double desired_speed = 40/3.6;

/////////////////////////// measure traffic range
const float lane_width = 4.0;
const float half_lane_width=lane_width/2.0;
const float max_dist_range = 60;
float min_dist = 99999;
float distance;
double desired_dist = 12.0;
double range_min = 5.0;
double range_max = 30.0;
bool inc_flag = true;

extern uint8_t* prestart;
extern uint8_t* prelaunch;
extern float* psteer;
extern float* pacc;
extern float* pbrake;
extern float* pspeed;
extern float* pto_track_middle; 
extern float* pangle;
extern float* pdist_raced;
extern void* shm;

// Compute the length to the start of the segment.
float getDistToSegStart(tCarElt *ocar)
{
    if (ocar->_trkPos.seg->type == TR_STR) {
        return ocar->_trkPos.toStart;
    } else {
        return ocar->_trkPos.toStart*ocar->_trkPos.seg->radius;
    }
}

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

    if (min_dist > max_dist_range) min_dist = max_dist_range;

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

        if (distance > 0) { // sometimes distance is weired and is negative
            if (inc_flag) {
                // accelerate to increase distance
                if (desired_dist < distance) {
                    car->ctrl.brakeCmd=0.0;
                    car->ctrl.accelCmd=0.3;
                } else {
                    car->ctrl.brakeCmd=0.2;
                    car->ctrl.accelCmd=0.0;
                }

                desired_dist += 0.01;
                if (desired_dist > range_max) inc_flag = false;
            } else {
                // decelerate to decrease distance
                if (desired_dist < distance) {
                    car->ctrl.brakeCmd=0.0;
                    car->ctrl.accelCmd=0.3;
                } else {
                    car->ctrl.brakeCmd=0.2;
                    car->ctrl.accelCmd=0.0;
                }

                desired_dist -= 0.01;
                if (desired_dist < range_min) inc_flag = true;
            }

            *pspeed = car->_speed_x;
            // printf("From TORCS: speed_x = %.5f\tspeed_y = %.5f\n", car->_speed_x, car->_speed_y);
            *pto_track_middle = 0;
            *pangle = angle;
            *pdist_raced = distance;
        } // end of ( if distance > 0)

        printf("inc_flag: %d, desired_dist: %f, distance: %f\n", inc_flag, desired_dist, distance);

    }


    // check if restart
    if (*prestart == 49){
        car->ctrl.askRestart = true;
        *prestart = 0;
    }

    // check if relaunch
    if (*prelaunch == 49) {
        if (shmdt(shm) == -1) {
            printf("shmdt failed.\n");
            exit(EXIT_FAILURE);
        }
        printf("\n********** Memory sharing stopped. Good Bye! **********\n");  
        exit(EXIT_SUCCESS);
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

