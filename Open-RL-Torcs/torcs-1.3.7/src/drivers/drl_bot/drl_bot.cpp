/***************************************************************************

    file                 : drl_bot.cpp
    created              : 2016年 12月 19日 星期一 09:11:36 CST
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
drl_bot(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("drl_bot");		/* name of the module (short) */
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
extern float* psl_x;
extern float* psl_y;
extern float* psr_x;
extern float* psr_y;
extern float* pcar_x;
extern float* pcar_y;
extern void* shm;

#define TR_SL	0		/**< Start-Left corner */
#define TR_SR	1		/**< Start-Right corner */
#define TR_EL	2		/**< End-Left corner */
#define TR_ER	3		/**< End_Right corner */

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 

    float dist_to_track_middle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width); // factor
    float dist_to_track_middle_m = car->_trkPos.toMiddle;
    float angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle);
    float angle_ = angle - (car->_trkPos.toMiddle) / car->_trkPos.seg->width;
    *ptoMarking_R = car->_laps;

    float car_x = car->_pos_X;
    float car_y = car->_pos_Y;
    // write track info to shared memory
    *pspeed = car->_speed_x;
    *pto_track_middle = dist_to_track_middle;
    *pangle = angle;
    *pdist_raced = car->_distRaced + 85;
    *ptoMarking_L = dist_to_track_middle_m; // abuse
    *pcar_x = car_x;
    *pcar_y = car_y;

    *pwritten = 1;

	while(*pwritten == 1){
		usleep(1);
	}

    // if (car->_speed_x < 45./3.6 - 0.5)
    // {
    //     car->ctrl.accelCmd = 1;
    //     car->ctrl.brakeCmd = 0;
    // }
    // else if (car->_speed_x > 45./3.6 + 0.5)
    // {
    //     car->ctrl.accelCmd = 0;
    //     car->ctrl.brakeCmd = 0.3;
    // }

    // car->ctrl.steer = angle_ / car->_steerLock;
    car->ctrl.steer = *psteer;
    car->ctrl.gear = 1;
    car->ctrl.accelCmd = 0.2; //*pacc;
    car->ctrl.brakeCmd = 0.0; //*pbrake;
    // if (car->_speed_x < 2.0 and car->_distRaced + 100 > 200)
    // {
    //     car->ctrl.accelCmd = 0.4;
    // }

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
        // printf("\n********** Memory sharing stopped. Good Bye! **********\n");  
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

