/***************************************************************************

    file                 : racemantools.h
    created              : Sat Mar 18 23:33:01 CET 2000
    copyright            : (C) 2000-2013 by Eric Espie, Bernhard Wymann
    email                : torcs@free.fr
    version              : $Id: racescreens.h,v 1.2.2.6 2014/05/20 11:02:18 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 
#ifndef __RACEMANTOOLS_H__
#define __RACEMANTOOLS_H__

#include <car.h>
#include <raceman.h>
#include <track.h>
#include <simu.h>


/** @brief Structure for track selection
 *  @ingroup racemantools
 */
typedef struct
{
	void *param;			/**< Pointer to race manager parameter set (to set the track) */
	void *prevScreen;		/**< Race manager screen to go back */
	void *nextScreen;		/**< Race manager screen to go after select */
	tTrackItf trackItf;		/**< Track module interface */
} tRmTrackSelect;


/** @brief Structure for driver selection
 *  @ingroup racemantools
 */
typedef struct
{
	void *param;		/**< Pointer to race manager parameter set (to set the driver) */
	void *prevScreen;	/**< Race manager screen to go back */
	void *nextScreen;	/**< Race manager screen to go after select */
} tRmDrvSelect;


/** @brief Structure for configuring parameters
 *  @ingroup racemantools
 */
typedef struct
{
	void *param;			/**< Pointer to race manager parameter set (to set the parameters) */
	void *prevScreen;		/**< Race manager screen to go back */
	void *nextScreen;		/**< Race manager screen to go after select */
	const char *title;		/**< Title for race parameters (for screen title) */
	unsigned int confMask;	/**< Tell what to configure */
#define RM_CONF_RACE_LEN	0x00000001
#define RM_CONF_DISP_MODE	0x00000002
} tRmRaceParam;


/** @brief Callback function definition for handling file selection
 *  @ingroup racemantools
 */
typedef void (*tfSelectFile) (char *);


/** @brief Structure for file selection
 *  @ingroup racemantools
 */
typedef struct
{
	const char *title;		/**< Title for file selection (for screen title) */
	char *path;				/**< Directory containing the files to list */
	void *prevScreen;		/**< Link to previous screen */
	tfSelectFile select;	/**< Callback function to handle the selection (store the selection in the caller) */
} tRmFileSelect;


extern void RmTrackSelect(void * /* vs */);
extern char *RmGetTrackName(char * /* category */, char * /* trackName */);
extern char *RmGetCategoryName(char /* *category*/);

extern void RmDriversSelect(void * /* vs */);

extern void RmPitMenuStart(tCarElt * /* car */, tRmInfo *rmInfo, void * /* userdata */, tfuiCallback /* callback */);
extern void *RmCarSetupScreenInit(void *prevMenu, tCarElt *car, tRmInfo* reInfo);

extern void RmLoadingScreenStart(const char * /* text */, const char * /* bgimg */);
extern void RmLoadingScreenSetText(const char * /* text */);
extern void RmShutdownLoadingScreen(void);

extern void RmShowResults(void * /* prevHdle */, tRmInfo * /* info */);

extern void *RmTwoStateScreen(const char *title,
			      const char *label1, const char *tip1, void *screen1,
			      const char *label2, const char *tip2, void *screen2);

extern void *RmTriStateScreen(const char *title,
			      const char *label1, const char *tip1, void *screen1,
			      const char *label2, const char *tip2, void *screen2,
			      const char *label3, const char *tip3, void *screen3);

extern void *RmFourStateScreen(const char *title,
			       const char *label1, const char *tip1, void *screen1,
			       const char *label2, const char *tip2, void *screen2,
			       const char *label3, const char *tip3, void *screen3,
			       const char *label4, const char *tip4, void *screen4);

extern void *RmNStateScreen(
					 const char *title,
					 const char** label,
					 const char** tip,
					 void** screen,
					 const int n
);

extern void RmDisplayStartRace(tRmInfo *info, void *startScr, void *abortScr);


extern void RmRaceParamMenu(void *vrp);

extern void RmShowStandings(void *prevHdle, tRmInfo *info);

extern void RmFileSelect(void *vs);

#endif /* __RACEMANTOOLS_H__ */

