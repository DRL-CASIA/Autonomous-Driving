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
#include <stdint.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

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
// float get_desired_speed() {
//     if (step_counter < inc) return 44/3.6;
//     if (step_counter >= inc && step_counter < 2*inc) return 40/3.6;
//     if (step_counter >= 2*inc && step_counter < 3*inc) return 36/3.6;
//     if (step_counter >= 3*inc && step_counter < 4*inc) return 40/3.6;
//     if (step_counter == 4*inc) {
//         step_counter = 0;
//         return 44/3.6;
//     }
// }

float getDistToSegEnd(tCarElt *car)
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
    }
}

uint8_t trk_type(tTrackSeg *seg) 
{
    if (seg->type == TR_STR){
        return 1;
    } else if (seg->type == TR_LFT) {
        return 0;
    } else if (seg->type == TR_RGT) {
        return 2;
    }
}

// locate the segment in the desired_range
uint8_t check_forward_trk_type(tTrackSeg *seg, float expected_range, float range, float thresh) 
{
    while (1) 
    {
        if (range >= expected_range) {
            break;
        } else {
            seg = seg->next;
            if (seg->type == TR_STR) {
                range = range + seg->length;
            } else {
                range = range + seg->arc * seg->radius;
            }
        }
    }
    // check the forward seg's radius,
    // if small: normal turn
    // if big: regard it as a straight track
    float radius = seg->radius;
    if (radius <= thresh) {
        return trk_type(seg);
    } else {
        return 1; // straigth
    }
}


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
extern float* ptoMarking_LL;
extern float* ptoMarking_ML;
extern float* ptoMarking_MR;
extern float* ptoMarking_RR;
extern void* shm;

float desired_speed = 55/3.6;
float desired_range = 20;
/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 
    float angle;
    const float SC = 1.0;
    float bias = -1.0; // + to left, - to right
    float radius_thresh = 170.0;

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
        angle -= SC*((car->_trkPos.toMiddle - bias) / car->_trkPos.seg->width);

        // calculate distance to marking here
        const float lane_width = 4.0;
        const float half_lane_width = lane_width / 2.0;
        const float max_toMarking_LR=7.0;
        const float max_toMarking_M=3.5;
        const float max_toMarking_LLRR=9.5;
        const float max_toMarking_MM=5.5;
        
        float toMarking_L = -max_toMarking_LR;
        float toMarking_M = max_toMarking_M;
        float toMarking_R = max_toMarking_LR;

        float toMarking_LL = -max_toMarking_LLRR;
        float toMarking_ML = -max_toMarking_MM;
        float toMarking_MR = max_toMarking_MM;
        float toMarking_RR = max_toMarking_LLRR;

        if (car->_trkPos.toMiddle <= (half_lane_width-0.5) && car->_trkPos.toMiddle>=-(half_lane_width-0.5)) {// in lane
            toMarking_ML = car->_trkPos.toMiddle-half_lane_width;
            toMarking_MR=car->_trkPos.toMiddle+half_lane_width;
            // printf("IN LANE, to_middle=%.4f | toMarking_ML=%.4f | toMarking_MR = %.4f\n", toMiddle, toMarking_ML, toMarking_MR);
        }

        if (car->_trkPos.toMiddle<=-(half_lane_width-1.2) && car->_trkPos.toMiddle>=-(half_lane_width+2)) { // on the right marking
            toMarking_L=car->_trkPos.toMiddle-half_lane_width;
            toMarking_M=car->_trkPos.toMiddle+half_lane_width;
            // printf("RIGHT MARKING, to_middle=%.4f | toMarking_L=%.4f | toMarking_M = %.4f\n", toMiddle, toMarking_L, toMarking_M);
        } else if (car->_trkPos.toMiddle<=(half_lane_width+2) && car->_trkPos.toMiddle>=(half_lane_width-1.2)) { // on the left marking
            toMarking_M=car->_trkPos.toMiddle-half_lane_width;
            toMarking_R=car->_trkPos.toMiddle+half_lane_width;
            // printf("LEFT MARKING, to_middle=%.4f | toMarking_M=%.4f | toMarking_R = %.4f\n", toMiddle, toMarking_M, toMarking_R);
        }

        // update shared memory data
        float angle_ = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
        NORM_PI_PI(angle_);
        *pangle = angle_;
        *ptoMarking_L = toMarking_L;
        *ptoMarking_M = toMarking_M;
        *ptoMarking_R = toMarking_R;
        *ptoMarking_LL = toMarking_LL;
        *ptoMarking_ML = toMarking_ML;
        *ptoMarking_MR = toMarking_MR;
        *ptoMarking_RR = toMarking_RR;
        

        tTrackSeg *segptr = car->_trkPos.seg;
        tTrackSeg *curr_segptr = car->_trkPos.seg;
        float range = getDistToSegEnd(car);
        float range_ = range;
        int segcnt = 0;
        bool check_trk_type_flag = true;
        if (curr_segptr->type == TR_STR){
            check_trk_type_flag = true;
        } else {
            check_trk_type_flag = false;
        }

        if (check_trk_type_flag) {
            *ptrk_type = check_forward_trk_type(segptr, desired_range, range, radius_thresh); // type, radius
        } else {
            float radius = curr_segptr->radius;
            if (radius <= radius_thresh) {
                // normal turn
                *ptrk_type = trk_type(curr_segptr);
            } else {
                // ignored turn and check the forward seg
                *ptrk_type = check_forward_trk_type(segptr, desired_range, range, radius_thresh);
            }
        }

        while (1) 
        {
            if (range >= desired_range) {
                break;
            } else {
                segptr = segptr->next;
                if (segptr->type == TR_STR) {
                    range = range + segptr->length;
                } else {
                    range = range + segptr->arc * segptr->radius;
                }
            }
        }
        float forward_radius = segptr->radius;
        // printf("type: %d | to current seg: %.1f | forward radius: %f | radius: %f\n", *ptrk_type, range_, forward_radius, curr_segptr->radius);

        car->ctrl.steer = angle / car->_steerLock;
        car->ctrl.gear = 1; // first gear
        car->ctrl.accelCmd = 0.2;
        car->ctrl.brakeCmd = 0.0;
        if (car->_speed_x>desired_speed) {
           car->ctrl.brakeCmd=0.2;
           car->ctrl.accelCmd=0.0;
        }
        else if  (car->_speed_x<desired_speed) {
           car->ctrl.accelCmd=0.2;
           car->ctrl.brakeCmd=0.0;
        }
        *pdist_raced = car->ctrl.steer; // surrogate steer

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

