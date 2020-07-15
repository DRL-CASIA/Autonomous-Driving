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

// For UDP
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>


/********* Defination for UDP usage *********/
static int IMG_W = 64;
static int IMG_H = 64;
static int UDP_MSGLEN = IMG_H * IMG_W * 3 + 100;
static int UDP_TIMEOUT = 50000;
static int UDP_SERVER_PORT = 5001;
#define UDP_ID "DRL"
socklen_t client_len;
struct sockaddr_in server;
struct sockaddr_in client;
int sock, length, n;



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

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
    printf("---DEBUG: running newrace().\n");
    char buf[UDP_MSGLEN];
    bool identified=false;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) error("Opening socket failed.");
    length = sizeof(server);
    bzero(&server, length);
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(UDP_SERVER_PORT);
    if (bind(sock, (struct sockaddr *)&server, length) < 0)
        error("Binding failed.");

    // Wait connections from clients.
    listen(sock, 5);
    printf("Waiting for client connection.\n");
    // client_len = sizeof(struct sockaddr_in);
    // newsock = accept(sock,(struct sockaddr *)&client,&client_len);
    // if (newsock < 0) error("Accept failed.");
    // // Make sure the right client identified.
    // while (!identified){
    //     client_len = sizeof(struct sockaddr_in);
    //     bzero(buf, UDP_MSGLEN);
    //     n = read(newsock, buf, UDP_MSGLEN);
    //     if (n < 0) error("Recvfrom failed.");
    //     if (strncmp(buf, UDP_ID, 3) == 0){
    //         printf("Identified the DRL client.\n");
    //     }

    //     // Respond the msg to client.
    //     sprintf(buf, "IDENTIFIED");
    //     n = write(newsock, buf, sizeof(buf));
    //     printf("%s\n", );
    //     if (n < 0) error("Sendto failed.");
    //     identified = true;
    // }

    while (!identified){
        client_len = sizeof(struct sockaddr_in);
        bzero(buf, UDP_MSGLEN);
        n = recvfrom(sock, buf, UDP_MSGLEN, 0, 
            (struct sockaddr *)&client, &client_len);
        if (n < 0) error("Recvfrom failed.");
        if (strncmp(buf, UDP_ID, 3) == 0){
            printf("Identified the DRL client.\n");
        }

        // Respond the msg to client.
        sprintf(buf, "IDENTIFIED");
        n = sendto(sock, buf, sizeof(buf), 0, 
            (struct sockaddr *)&client, client_len);
        if (n < 0) error("Sendto failed.");
        identified = true;
    }
    printf("***DEBUG: running newrace().\n");
} 

// Get img string
std::string get_img_string(unsigned char *img, int size){
    // printf("---DEBUG: running get_img_string()...\n");
    std::ostringstream ostr;
    ostr << "(img";
    for (int i = 0; i < size; i++) ostr << " " << (int)img[i];
    ostr << ")";
    // std::string s = ostr.str();
    // std::cout<< s << "\n";
    // printf("***DEBUG: running get_img_string()...\n");
    return ostr.str();
}

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    printf("---DEBUG: running drive()...\n");
    char buf[15];
    // For unblock mode
    struct timeval timeval;
    fd_set readSet;

    /********** State string **********/
    std::string state;
    if (get_vision()){
        printf("---CKPT: 1\n");
        state = get_img_string(car->vision->img, car->vision->size);
    }
    // printf("%s\n", state.c_str());
    // sprintf(buf, "test msg");
    printf("car->vision->size = %d\n", car->vision->size);
    printf("state len = %d\n", state.length()+1);
    n = sendto(sock,state.c_str(), state.length()+1, 0,
     (struct sockaddr *)&client, client_len);
    // n = sendto(sock, buf, sizeof(buf), 0, 
    //         (struct sockaddr *)&client, client_len);
    if (n < 0) error("Sendto failed.");


    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 
    car->ctrl.brakeCmd = 1.0; /* all brakes on ... */ 
    /*  
     * add the driving code here to modify the 
     * car->_steerCmd 
     * car->_accelCmd 
     * car->_brakeCmd 
     * car->_gearCmd 
     * car->_clutchCmd 
     */ 
    printf("***DEBUG: running drive()...\n");
    // exit(-1);
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

