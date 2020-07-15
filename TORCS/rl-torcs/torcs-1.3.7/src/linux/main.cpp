/***************************************************************************

    file                 : main.cpp
    created              : Sat Mar 18 23:54:30 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
    version              : $Id: main.cpp,v 1.14.2.3 2012/06/01 01:59:42 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <stdlib.h>

#include <GL/glut.h>

#include <tgfclient.h>
#include <client.h>

#include "linuxspec.h"
#include <raceinit.h>

/////////////////// added by lidong (start)
#include <sys/shm.h> 
#define image_width 640
#define image_height 480
#include <iostream> // added by lidong 
#include <unistd.h> // added by lidong
/////////////////// added by lidong (end)

extern bool bKeepModules;

static void
init_args(int argc, char **argv, const char **raceconfig)
{
	int i;
	char *buf;

	i = 1;
    printf("\n");

	while(i < argc) {
		if(strncmp(argv[i], "-l", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetLocalDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-L", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetLibDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-D", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetDataDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-s", 2) == 0) {
			i++;
			SetSingleTextureMode();
		} else if(strncmp(argv[i], "-k", 2) == 0) {
			i++;
			// Keep modules in memory (for valgrind)
			printf("Unloading modules disabled, just intended for valgrind runs.\n");
			bKeepModules = true;
        #ifndef FREEGLUT
        		} else if(strncmp(argv[i], "-m", 2) == 0) {
        			i++;
        			GfuiMouseSetHWPresent(); /* allow the hardware cursor */
        #endif
		} else if(strncmp(argv[i], "-r", 2) == 0) {
			i++;
			*raceconfig = "";

			if(i < argc) {
				*raceconfig = argv[i];
				i++;
			}

			if((strlen(*raceconfig) == 0) || (strstr(*raceconfig, ".xml") == 0)) {
				printf("Please specify a race configuration xml when using -r\n");
				exit(1);
			}
		} else {
			i++;		/* ignore bad args */
		}
	}

    #ifdef FREEGLUT
    	GfuiMouseSetHWPresent(); /* allow the hardware cursor (freeglut pb ?) */
    #endif
}

/*
 * Function
 *	main
 *
 * Description
 *	LINUX entry point of TORCS
 *
 * Parameters
 *
 *
 * Return
 *
 *
 * Remarks
 *
 */

///////////////// added by lidong (start)
struct shared_use_st
{
    // N.B. always put float behind uint8 to save memory.
    uint8_t data[image_width*image_height*3];
	uint8_t written;
	uint8_t restart;
    uint8_t relaunch;
    uint8_t trk_type;
    float steer; // only read
    float acc;
    float brake;
	float speed; // only write
	float to_track_middle;  // only write
    float angle;  // only write
    float dist_raced;  // only write
    float toMarking_L;
    float toMarking_M;
    float toMarking_R;
    float toMarking_LL;
    float toMarking_ML;
    float toMarking_MR;
    float toMarking_RR;
    float sl_x; // track seg start left x
    float sl_y;
    float sr_x; // track seg start right x
    float sr_y;
    float car_x;
    float car_y;
    // for ssci2018 data collection
    float hm_dist_raced;
    float car1_dist_raced;
    float car2_dist_raced;
    float car3_dist_raced;
    // for decision-making 2 lanes 6 cars
    float dm1_x;
    float dm1_y;
    float dm2_x;
    float dm2_y;
    float dm3_x;
    float dm3_y;
    float dm4_x;
    float dm4_y;
    float dm5_x;
    float dm5_y;
    float dm6_x;
    float dm6_y;
    float dm7_x;
    float dm7_y;
    float dm8_x;
    float dm8_y;
    float dm9_x;
    float dm9_y;
    float dm1_dist_raced;
    float dm2_dist_raced;
    float dm3_dist_raced;
    float dm4_dist_raced;
    float dm5_dist_raced;
    float dm6_dist_raced;
    float dm7_dist_raced;
    float dm8_dist_raced;
    float dm9_dist_raced;
    float dm1_speed;
    float dm2_speed;
    float dm3_speed;
    float dm4_speed;
    float dm5_speed;
    float dm6_speed;
    float dm7_speed;
    float dm8_speed;
    float dm9_speed;
    float dm1_to_track_middle_m;
    float dm2_to_track_middle_m;
    float dm3_to_track_middle_m;
    float dm4_to_track_middle_m;
    float dm5_to_track_middle_m;
    float dm6_to_track_middle_m;
    float dm7_to_track_middle_m;
    float dm8_to_track_middle_m;
    float dm9_to_track_middle_m;
};
 
uint8_t* pdata = NULL;
// uint8_t* pwritten = NULL;
uint8_t* pwritten_ghost = NULL;
uint8_t* prestart_ghost = NULL;
uint8_t* prelaunch_ghost = NULL;
uint8_t* ptrk_type_ghost = NULL;
float* psteer_ghost = NULL;
float* pacc_ghost = NULL;
float* pbrake_ghost = NULL;
float* pspeed_ghost = NULL;
float* pangle_ghost = NULL;
float* pto_track_middle_ghost = NULL;
float* pdist_raced_ghost = NULL;
float* ptoMarking_L_ghost = NULL;
float* ptoMarking_M_ghost = NULL;
float* ptoMarking_R_ghost = NULL;
float* ptoMarking_LL_ghost = NULL;
float* ptoMarking_ML_ghost = NULL;
float* ptoMarking_MR_ghost = NULL;
float* ptoMarking_RR_ghost = NULL;
float* psl_x_ghost = NULL;
float* psl_y_ghost = NULL;
float* psr_x_ghost = NULL;
float* psr_y_ghost = NULL;
float* pcar_x_ghost = NULL;
float* pcar_y_ghost = NULL;
float* phm_dist_raced_ghost = NULL;
float* pcar1_dist_raced_ghost = NULL;
float* pcar2_dist_raced_ghost = NULL;
float* pcar3_dist_raced_ghost = NULL;
float* pdm1_x_ghost = NULL;
float* pdm1_y_ghost = NULL;
float* pdm2_x_ghost = NULL;
float* pdm2_y_ghost = NULL;
float* pdm3_x_ghost = NULL;
float* pdm3_y_ghost = NULL;
float* pdm4_x_ghost = NULL;
float* pdm4_y_ghost = NULL;
float* pdm5_x_ghost = NULL;
float* pdm5_y_ghost = NULL;
float* pdm6_x_ghost = NULL;
float* pdm6_y_ghost = NULL;
float* pdm7_x_ghost = NULL;
float* pdm7_y_ghost = NULL;
float* pdm8_x_ghost = NULL;
float* pdm8_y_ghost = NULL;
float* pdm9_x_ghost = NULL;
float* pdm9_y_ghost = NULL;
float* pdm1_dist_raced_ghost  = NULL;
float* pdm2_dist_raced_ghost = NULL;
float* pdm3_dist_raced_ghost = NULL;
float* pdm4_dist_raced_ghost = NULL;
float* pdm5_dist_raced_ghost = NULL;
float* pdm6_dist_raced_ghost = NULL;
float* pdm7_dist_raced_ghost = NULL;
float* pdm8_dist_raced_ghost = NULL;
float* pdm9_dist_raced_ghost = NULL;
float* pdm1_speed_ghost = NULL;
float* pdm2_speed_ghost = NULL;
float* pdm3_speed_ghost = NULL;
float* pdm4_speed_ghost = NULL;
float* pdm5_speed_ghost = NULL;
float* pdm6_speed_ghost = NULL;
float* pdm7_speed_ghost = NULL;
float* pdm8_speed_ghost = NULL;
float* pdm9_speed_ghost = NULL;
float* pdm1_to_track_middle_m_ghost = NULL;
float* pdm2_to_track_middle_m_ghost = NULL;
float* pdm3_to_track_middle_m_ghost = NULL;
float* pdm4_to_track_middle_m_ghost = NULL;
float* pdm5_to_track_middle_m_ghost = NULL;
float* pdm6_to_track_middle_m_ghost = NULL;
float* pdm7_to_track_middle_m_ghost = NULL;
float* pdm8_to_track_middle_m_ghost = NULL;
float* pdm9_to_track_middle_m_ghost = NULL;
void* shm = NULL;
///////////////// added by lidong (end) 

int
main(int argc, char *argv[])
{
	///////////////// added by lidong (start)
	struct shared_use_st *shared = NULL;
	int shmid;
	shmid = shmget((key_t)934, sizeof(struct shared_use_st), 0666|IPC_CREAT);
	if(shmid == -1)  
    {  
        fprintf(stderr, "shmget failed\n");  
        exit(EXIT_FAILURE);  
    } 

    shm = shmat(shmid, 0, 0);
    if(shm == (void*)-1)  
    {  
        fprintf(stderr, "shmat failed\n");  
        exit(EXIT_FAILURE);  
    } 
    // printf("\n **************** Memory sharing starting *************\n");
    shared = (struct shared_use_st*) shm;
    shared->written = 0;
    shared->restart = 0; 
    shared->relaunch = 0;
    shared->trk_type = 0;
    shared->steer = 0.0;
    shared->acc = 0.0;
    shared->brake = 0.0;
    shared->speed = 0.0;
    shared->to_track_middle = 0.0;
    shared->angle = 0.0;
    shared->dist_raced = 0.0;
    shared->toMarking_L = 0.0;
    shared->toMarking_M = 0.0;
    shared->toMarking_R = 0.0;
    shared->toMarking_LL = 0.0;
    shared->toMarking_ML = 0.0;
    shared->toMarking_MR = 0.0;
    shared->toMarking_RR = 0.0;
    shared->hm_dist_raced = 0.;
    shared->car1_dist_raced = 0.;
    shared->car2_dist_raced = 0.;
    shared->car3_dist_raced = 0.;
    shared->dm1_x = 0.;
    shared->dm1_y = 0.;
    shared->dm2_x = 0.;
    shared->dm2_y = 0.;
    shared->dm3_x = 0.;
    shared->dm3_y = 0.;
    shared->dm4_x = 0.;
    shared->dm4_y = 0.;
    shared->dm5_x = 0.;
    shared->dm5_y = 0.;
    shared->dm6_x = 0.;
    shared->dm6_y = 0.;
    shared->dm7_x = 0.;
    shared->dm7_y = 0.;
    shared->dm8_x = 0.;
    shared->dm8_y = 0.;
    shared->dm9_x = 0.;
    shared->dm9_y = 0.;
    shared->dm1_dist_raced = 0.;
    shared->dm2_dist_raced = 0.;
    shared->dm3_dist_raced = 0.;
    shared->dm4_dist_raced = 0.;
    shared->dm5_dist_raced = 0.;
    shared->dm6_dist_raced = 0.;
    shared->dm7_dist_raced = 0.;
    shared->dm8_dist_raced = 0.;
    shared->dm9_dist_raced = 0.;
    shared->dm1_speed = 0.;
    shared->dm2_speed = 0.;
    shared->dm3_speed = 0.;
    shared->dm4_speed = 0.;
    shared->dm5_speed = 0.;
    shared->dm6_speed = 0.;
    shared->dm7_speed = 0.;
    shared->dm8_speed = 0.;
    shared->dm9_speed = 0.;
    shared->dm1_to_track_middle_m = 0.;
    shared->dm2_to_track_middle_m = 0.;
    shared->dm3_to_track_middle_m = 0.;
    shared->dm4_to_track_middle_m = 0.;
    shared->dm5_to_track_middle_m = 0.;
    shared->dm6_to_track_middle_m = 0.;
    shared->dm7_to_track_middle_m = 0.;
    shared->dm8_to_track_middle_m = 0.;
    shared->dm9_to_track_middle_m = 0.;
    pwritten_ghost = &shared->written;
    prestart_ghost = &shared->restart;
    prelaunch_ghost = &shared->relaunch;
    ptrk_type_ghost = &shared->trk_type;
    psteer_ghost = &shared->steer;
    pacc_ghost = &shared->acc;
    pbrake_ghost = &shared->brake;
    pspeed_ghost = &shared->speed;
    pto_track_middle_ghost = &shared->to_track_middle;
    pangle_ghost = &shared->angle;
    pdist_raced_ghost = &shared->dist_raced;
    ptoMarking_L_ghost = &shared->toMarking_L;
    ptoMarking_M_ghost = &shared->toMarking_M;
    ptoMarking_R_ghost = &shared->toMarking_R;
    ptoMarking_LL_ghost = &shared->toMarking_LL;
    ptoMarking_ML_ghost = &shared->toMarking_ML;
    ptoMarking_MR_ghost = &shared->toMarking_MR;
    ptoMarking_RR_ghost = &shared->toMarking_RR;
    psl_x_ghost = &shared->sl_x;
    psl_y_ghost = &shared->sl_y;
    psr_x_ghost = &shared->sr_x;
    psr_y_ghost = &shared->sr_y;
    pcar_x_ghost = &shared->car_x;
    pcar_y_ghost = &shared->car_y;
    phm_dist_raced_ghost = &shared->hm_dist_raced;
    pcar1_dist_raced_ghost = &shared->car1_dist_raced;
    pcar2_dist_raced_ghost = &shared->car2_dist_raced;
    pcar3_dist_raced_ghost = &shared->car3_dist_raced;
    pdm1_x_ghost = &shared->dm1_x;
    pdm1_y_ghost = &shared->dm1_y;
    pdm2_x_ghost = &shared->dm2_x;
    pdm2_y_ghost = &shared->dm2_y;
    pdm3_x_ghost = &shared->dm3_x;
    pdm3_y_ghost = &shared->dm3_y;
    pdm4_x_ghost = &shared->dm4_x;
    pdm4_y_ghost = &shared->dm4_y;
    pdm5_x_ghost = &shared->dm5_x;
    pdm5_y_ghost = &shared->dm5_y;
    pdm6_x_ghost = &shared->dm6_x;
    pdm6_y_ghost = &shared->dm6_y;
    pdm7_x_ghost = &shared->dm7_x;
    pdm7_y_ghost = &shared->dm7_y;
    pdm8_x_ghost = &shared->dm8_x;
    pdm8_y_ghost = &shared->dm8_y;
    pdm9_x_ghost = &shared->dm9_x;
    pdm9_y_ghost = &shared->dm9_y;
    pdm1_dist_raced_ghost = &shared->dm1_dist_raced;
    pdm2_dist_raced_ghost = &shared->dm2_dist_raced;
    pdm3_dist_raced_ghost = &shared->dm3_dist_raced;
    pdm4_dist_raced_ghost = &shared->dm4_dist_raced;
    pdm5_dist_raced_ghost = &shared->dm5_dist_raced;
    pdm6_dist_raced_ghost = &shared->dm6_dist_raced;
    pdm7_dist_raced_ghost = &shared->dm7_dist_raced;
    pdm8_dist_raced_ghost = &shared->dm8_dist_raced;
    pdm9_dist_raced_ghost = &shared->dm9_dist_raced;
    pdm1_speed_ghost = &shared->dm1_speed;
    pdm2_speed_ghost = &shared->dm2_speed;
    pdm3_speed_ghost = &shared->dm3_speed;
    pdm4_speed_ghost = &shared->dm4_speed;
    pdm5_speed_ghost = &shared->dm5_speed;
    pdm6_speed_ghost = &shared->dm6_speed;
    pdm7_speed_ghost = &shared->dm7_speed;
    pdm8_speed_ghost = &shared->dm8_speed;
    pdm9_speed_ghost = &shared->dm9_speed;
    pdm1_to_track_middle_m_ghost = &shared->dm1_to_track_middle_m;
    pdm2_to_track_middle_m_ghost = &shared->dm2_to_track_middle_m;
    pdm3_to_track_middle_m_ghost = &shared->dm3_to_track_middle_m;
    pdm4_to_track_middle_m_ghost = &shared->dm4_to_track_middle_m;
    pdm5_to_track_middle_m_ghost = &shared->dm5_to_track_middle_m;
    pdm6_to_track_middle_m_ghost = &shared->dm6_to_track_middle_m;
    pdm7_to_track_middle_m_ghost = &shared->dm7_to_track_middle_m;
    pdm8_to_track_middle_m_ghost = &shared->dm8_to_track_middle_m;
    pdm9_to_track_middle_m_ghost = &shared->dm9_to_track_middle_m;
    pdata = shared->data;
	///////////////// added by lidong (end) 

	const char *raceconfig = "";

	init_args(argc, argv, &raceconfig);
	LinuxSpecInit();			/* init specific linux functions */

	if(strlen(raceconfig) == 0) {
		GfScrInit(argc, argv);	/* init screen */
		TorcsEntry();			/* launch TORCS */
		glutMainLoop();			/* event loop of glut */
	} else {
		// Run race from console, no Window, no OpenGL/OpenAL etc.
		// Thought for blind scripted AI training
		ReRunRaceOnConsole(raceconfig);
	}

	return 0;					/* just for the compiler, never reached */
}

