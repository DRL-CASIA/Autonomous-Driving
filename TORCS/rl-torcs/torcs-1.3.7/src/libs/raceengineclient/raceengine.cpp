/***************************************************************************

    file        : raceengine.cpp
    created     : Sat Nov 23 09:05:23 CET 2002
    copyright   : (C) 2002-2014 by Eric Espie, Bernhard Wymann 
    email       : eric.espie@torcs.org 
    version     : $Id: raceengine.cpp,v 1.19.2.23 2014/08/05 23:05:06 berniw Exp $

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
    		
    @author	<a href=mailto:eric.espie@torcs.org>Eric Espie</a>
    @version	$Id: raceengine.cpp,v 1.19.2.23 2014/08/05 23:05:06 berniw Exp $
*/

#include <stdlib.h>
#include <stdio.h>
#include <tgfclient.h>
#include <robot.h>
#include <raceman.h>
#include <racescreens.h>
#include <robottools.h>
#include <portability.h>

#include "racemain.h"
#include "racegl.h"
#include "raceinit.h"
#include "raceresults.h"

#include "raceengine.h"

/////////////////// added by lidong (start)
#define image_width 640
#define image_height 480
/////////////////// added by lidong (end)

static double	msgDisp;
static double	bigMsgDisp;

tRmInfo	*ReInfo = 0;

static void ReRaceRules(tCarElt *car);


/* Compute Pit stop time */
static void
ReUpdtPitTime(tCarElt *car)
{
	tSituation *s = ReInfo->s;
	tReCarInfo *info = &(ReInfo->_reCarInfo[car->index]);
	int i;

	switch (car->_pitStopType) {
		case RM_PIT_REPAIR:
			info->totalPitTime = ReInfo->raceRules.pitstopBaseTime + fabs((double)(car->_pitFuel)) / ReInfo->raceRules.refuelFuelFlow + (tdble)(fabs((double)(car->_pitRepair))) * ReInfo->raceRules.damageRepairFactor + car->_penaltyTime;
			if (ReInfo->s->raceInfo.type == RM_TYPE_PRACTICE || ReInfo->s->raceInfo.type == RM_TYPE_QUALIF) { 
				// Ensure that the right min/max values are in the setup structure (could have been modified by the robot))
				RtInitCarPitSetup(car->_carHandle, &(car->pitcmd.setup), true);
			} else {
				// In case of the race no modifications are allowed, so completely reload the structure
				RtInitCarPitSetup(car->_carHandle, &(car->pitcmd.setup), false);
			}
			car->_scheduledEventTime = s->currentTime + info->totalPitTime;
			car->_penaltyTime = 0.0f;
			ReInfo->_reSimItf.reconfig(car);
			for (i=0; i<4; i++) {
				car->_tyreCondition(i) = 1.01;
				car->_tyreT_in(i) = 50.0;
				car->_tyreT_mid(i) = 50.0;
				car->_tyreT_out(i) = 50.0;
			}
			break;
		case RM_PIT_STOPANDGO:
			info->totalPitTime = car->_penaltyTime;
			car->_scheduledEventTime = s->currentTime + info->totalPitTime;
			car->_penaltyTime = 0.0f;
			break;
	}
}

/* Return from interactive pit information */
static void
ReUpdtPitCmd(void *pvcar)
{
	tCarElt *car = (tCarElt*)pvcar;

	ReUpdtPitTime(car);
	GfuiScreenActivate(ReInfo->_reGameScreen);
}

static void
ReRaceMsgUpdate(void)
{
	if (ReInfo->_reCurTime > msgDisp) {
		ReSetRaceMsg("");
	}
	if (ReInfo->_reCurTime > bigMsgDisp) {
		ReSetRaceBigMsg("");
	}
}

static void
ReRaceMsgSet(const char *msg, double life)
{
	if ((ReInfo->_displayMode != RM_DISP_MODE_NONE) && (ReInfo->_displayMode != RM_DISP_MODE_CONSOLE)) {
		ReSetRaceMsg(msg);
		msgDisp = ReInfo->_reCurTime + life;
	}
}


static void
ReRaceBigMsgSet(const char *msg, double life)
{
	if ((ReInfo->_displayMode != RM_DISP_MODE_NONE) && (ReInfo->_displayMode != RM_DISP_MODE_CONSOLE)) {
		ReSetRaceBigMsg(msg);
		bigMsgDisp = ReInfo->_reCurTime + life;
	}
}


static void
ReManage(tCarElt *car)
{
	int i, pitok;
	tTrackSeg *sseg;
	tdble wseg;
	static float color[] = {0.0, 0.0, 1.0, 1.0};
	tSituation *s = ReInfo->s;
	const int BUFSIZE = 1024;
	char buf[BUFSIZE];
	
	tReCarInfo *info = &(ReInfo->_reCarInfo[car->index]);
	
	if (car->_speed_x > car->_topSpeed) {
		car->_topSpeed = car->_speed_x;
	}

	// For practice and qualif.
	if (car->_speed_x > info->topSpd) {
		info->topSpd = car->_speed_x;
	}
	
	if (car->_speed_x < info->botSpd) {
		info->botSpd = car->_speed_x;
		car->_currentMinSpeedForLap = car->_speed_x;
	}
	
	// Pitstop.
	if (car->_pit) {
		if (car->ctrl.raceCmd & RM_CMD_PIT_ASKED) {
			// Pit already occupied?
			if (car->_pit->pitCarIndex == TR_PIT_STATE_FREE) {
				snprintf(car->ctrl.msg[2], 32, "Can Pit");
			} else {
				snprintf(car->ctrl.msg[2], 32, "Pit Occupied");
			}
			memcpy(car->ctrl.msgColor, color, sizeof(car->ctrl.msgColor));
		}
		
		if (car->_state & RM_CAR_STATE_PIT) {
			car->ctrl.raceCmd &= ~RM_CMD_PIT_ASKED; // clear the flag.
			if (car->_scheduledEventTime < s->currentTime) {
				car->_state &= ~RM_CAR_STATE_PIT;
				car->_pit->pitCarIndex = TR_PIT_STATE_FREE;
				snprintf(buf, BUFSIZE, "%s pit stop %.1fs", car->_name, info->totalPitTime);
				ReRaceMsgSet(buf, 5);
			} else {
				snprintf(car->ctrl.msg[2], 32, "in pits %.1fs", s->currentTime - info->startPitTime);
			}
		} else if ((car->ctrl.raceCmd & RM_CMD_PIT_ASKED) &&
					car->_pit->pitCarIndex == TR_PIT_STATE_FREE &&	
				   (s->_maxDammage == 0 || car->_dammage <= s->_maxDammage))
		{
			tdble lgFromStart = car->_trkPos.seg->lgfromstart;
			
			switch (car->_trkPos.seg->type) {
				case TR_STR:
					lgFromStart += car->_trkPos.toStart;
					break;
				default:
					lgFromStart += car->_trkPos.toStart * car->_trkPos.seg->radius;
					break;
			}
		
			if ((lgFromStart > car->_pit->lmin) && (lgFromStart < car->_pit->lmax)) {
				pitok = 0;
				int side;
				tdble toBorder;
				if (ReInfo->track->pits.side == TR_RGT) {
					side = TR_SIDE_RGT;
					toBorder = car->_trkPos.toRight;
				} else {
					side = TR_SIDE_LFT;
					toBorder = car->_trkPos.toLeft;
				}
				
				sseg = car->_trkPos.seg->side[side];
				wseg = RtTrackGetWidth(sseg, car->_trkPos.toStart);
				if (sseg->side[side]) {
					sseg = sseg->side[side];
					wseg += RtTrackGetWidth(sseg, car->_trkPos.toStart);
				}
				if (((toBorder + wseg) < (ReInfo->track->pits.width - car->_dimension_y / 2.0)) &&
					(fabs(car->_speed_x) < 1.0) &&
					(fabs(car->_speed_y) < 1.0))
				{
					pitok = 1;
				}
				
				if (pitok) {
					car->_state |= RM_CAR_STATE_PIT;
					car->_nbPitStops++;
					for (i = 0; i < car->_pit->freeCarIndex; i++) {
						if (car->_pit->car[i] == car) {
							car->_pit->pitCarIndex = i;
							break;
						}
					}
					info->startPitTime = s->currentTime;
					snprintf(buf, BUFSIZE, "%s in pits", car->_name);
					ReRaceMsgSet(buf, 5);
					if (car->robot->rbPitCmd(car->robot->index, car, s) == ROB_PIT_MENU) {
						// the pit cmd is modified by menu.
						ReStop();
						RmPitMenuStart(car, ReInfo, (void*)car, ReUpdtPitCmd);
					} else {
						ReUpdtPitTime(car);
					}
				}
			}
		}
	}
	
	/* Start Line Crossing */
	if (info->prevTrkPos.seg != car->_trkPos.seg) {
		if ((info->prevTrkPos.seg->raceInfo & TR_LAST) && (car->_trkPos.seg->raceInfo & TR_START)) {
			if (info->lapFlag == 0) {
				if ((car->_state & RM_CAR_STATE_FINISH) == 0) {
					car->_laps++;
					car->_remainingLaps--;
					if (car->_laps > 1) {
						car->_lastLapTime = s->currentTime - info->sTime;
						car->_curTime += car->_lastLapTime;
						if (car->_bestLapTime != 0) {
							car->_deltaBestLapTime = car->_lastLapTime - car->_bestLapTime;
						}
						if ((car->_lastLapTime < car->_bestLapTime) || (car->_bestLapTime == 0)) {
							if (car->_commitBestLapTime) {
								car->_bestLapTime = car->_lastLapTime;
							}
						}
						
						car->_commitBestLapTime = true;
						
						if (car->_pos != 1) {
							car->_timeBehindLeader = car->_curTime - s->cars[0]->_curTime;
							car->_lapsBehindLeader = s->cars[0]->_laps - car->_laps;
							car->_timeBehindPrev = car->_curTime - s->cars[car->_pos - 2]->_curTime;
							s->cars[car->_pos - 2]->_timeBeforeNext = car->_timeBehindPrev;
						} else {
							car->_timeBehindLeader = 0;
							car->_lapsBehindLeader = 0;
							car->_timeBehindPrev = 0;

							if (ReInfo->_displayMode == RM_DISP_MODE_CONSOLE) {
								printf("Sim Time: %8.2f [s], Leader Laps: %4d, Leader Distance: %8.3f [km]\n", s->currentTime, car->_laps - 1, car->_distRaced/1000.0f);
							}
						}
						info->sTime = s->currentTime;
						switch (ReInfo->s->_raceType) {
							case RM_TYPE_PRACTICE:
								if (ReInfo->_displayMode == RM_DISP_MODE_NONE) {
									ReInfo->_refreshDisplay = 1;
									const int TIMEFMTSIZE=256;
									char t1[TIMEFMTSIZE], t2[TIMEFMTSIZE];
									GfTime2Str(t1, TIMEFMTSIZE, car->_lastLapTime, 0);
									GfTime2Str(t2, TIMEFMTSIZE, car->_bestLapTime, 0);
									snprintf(buf, BUFSIZE, "lap: %02d   time: %s  best: %s  top spd: %.2f    min spd: %.2f    damage: %d",
										car->_laps - 1, t1, t2,
										info->topSpd * 3.6, info->botSpd * 3.6, car->_dammage);
									ReResScreenAddText(buf);
								}
								/* save the lap result */
								ReSavePracticeLap(car);
								break;
								
							case RM_TYPE_QUALIF:
								if (ReInfo->_displayMode == RM_DISP_MODE_NONE) {
									ReUpdateQualifCurRes(car);
								}
								break;
						}
					} else {
						if ((ReInfo->_displayMode == RM_DISP_MODE_NONE) && (ReInfo->s->_raceType == RM_TYPE_QUALIF)) {
							ReUpdateQualifCurRes(car);
						}
					}
			
					info->topSpd = car->_speed_x;
					info->botSpd = car->_speed_x;
					car->_currentMinSpeedForLap = car->_speed_x;
					if ((car->_remainingLaps < 0) || (s->_raceState == RM_RACE_FINISHING)) {
						car->_state |= RM_CAR_STATE_FINISH;
						s->_raceState = RM_RACE_FINISHING;
						if (ReInfo->s->_raceType == RM_TYPE_RACE) {
							if (car->_pos == 1) {
								snprintf(buf, BUFSIZE, "Winner %s", car->_name);
								ReRaceBigMsgSet(buf, 10);
							} else {
								const char *numSuffix = "th";
								if (abs(12 - car->_pos) > 1) { /* leave suffix as 'th' for 11 to 13 */
									switch (car->_pos % 10) {
									case 1:
									numSuffix = "st";
									break;
									case 2:
									numSuffix = "nd";
									break;
									case 3:
									numSuffix = "rd";
									break;
									default:
									break;
									}
								}
								snprintf(buf, BUFSIZE, "%s Finished %d%s", car->_name, car->_pos, numSuffix);
								ReRaceMsgSet(buf, 5);
							}
						}
					}
				} else {
					/* prevent infinite looping of cars around track, allow one lap after finish for the first car */
					for (i = 0; i < s->_ncars; i++) {
						s->cars[i]->_state |= RM_CAR_STATE_FINISH;
					}
					return;
				}
			} else {
				info->lapFlag--;
			}
		}

		if ((info->prevTrkPos.seg->raceInfo & TR_START) && (car->_trkPos.seg->raceInfo & TR_LAST)) {
			/* going backward through the start line */
			info->lapFlag++;
		}
	}
	ReRaceRules(car);
	
	info->prevTrkPos = car->_trkPos;
	car->_curLapTime = s->currentTime - info->sTime;
	car->_distFromStartLine = car->_trkPos.seg->lgfromstart +
	(car->_trkPos.seg->type == TR_STR ? car->_trkPos.toStart : car->_trkPos.toStart * car->_trkPos.seg->radius);
	car->_distRaced = (car->_laps - (info->lapFlag + 1)) * ReInfo->track->length + car->_distFromStartLine;
}


static void ReSortCars(void)
{
	int i, j;
	tCarElt	*car;
	int	allfinish;
	tSituation *s = ReInfo->s;

	if ((s->cars[0]->_state & RM_CAR_STATE_FINISH) == 0) {
		allfinish = 0;
	} else {
		allfinish = 1;
	}

	for (i = 1; i < s->_ncars; i++) {
		j = i;
		while (j > 0) {
			if ((s->cars[j]->_state & RM_CAR_STATE_FINISH) == 0) {
				allfinish = 0;
				if (s->cars[j]->_distRaced > s->cars[j-1]->_distRaced) {
					car = s->cars[j];
					s->cars[j] = s->cars[j-1];
					s->cars[j-1] = car;
					s->cars[j]->_pos = j+1;
					s->cars[j-1]->_pos = j;
					j--;
					continue;
				}
			}
			j = 0;
		}
	}

	if (allfinish) {
		ReInfo->s->_raceState = RM_RACE_ENDED;
	}
}


/* Compute the race rules and penalties */
static void
ReRaceRules(tCarElt *car)
{
	tCarPenalty *penalty;
	tTrack *track = ReInfo->track;
	tRmCarRules *rules = &(ReInfo->rules[car->index]);
	tTrackSeg *seg = RtTrackGetSeg(&(car->_trkPos));
	tReCarInfo *info = &(ReInfo->_reCarInfo[car->index]);
	tTrackSeg *prevSeg = RtTrackGetSeg(&(info->prevTrkPos));
	static float color[] = {0.0, 0.0, 1.0, 1.0};

	// DNF cars which need too much time for the current lap, this is mainly to avoid
	// that a "hanging" driver can stop the quali from finishing.
	// Allowed time is longest pitstop possible + time for tracklength with speed??? (currently fixed 10 [m/s]).
	// for simplicity. Human driver is an exception to this rule, to allow explorers
	// to enjoy the landscape.
	// TODO: Make it configurable.
	if ((car->_curLapTime > 84.5 + ReInfo->track->length/10.0) &&
		(car->_driverType != RM_DRV_HUMAN))
	{
		car->_state |= RM_CAR_STATE_ELIMINATED;
	    return;
	}

	const int BUFSIZE = 1024;
	char buf[BUFSIZE];

	// Ignore some rules after the car has finished the race
	if ((car->pub.state & RM_CAR_STATE_FINISH) == 0) {

		// If a car hits the track wall the lap time is invalidated, because of tracks where this behaviour allows much faster laps (e.g. alpine-2)
		// Invalidation and message is just shown on the first hit
		if (ReInfo->raceRules.enabled & RmRaceRules::WALL_HIT_TIME_INVALIDATE) {
			if (car->_commitBestLapTime && (car->priv.simcollision & SEM_COLLISION_XYSCENE)) {
				car->_commitBestLapTime = false;
				if (ReInfo->s->_raceType != RM_TYPE_RACE) {
					ReRaceMsgSet("Hit wall, laptime invalidated", 5);
				}
			}
		}
			
		// If the car cuts a corner the lap time is invalidated. Cutting a corner means: the center of gravity is more than 0.7 times the car width
		// away from the main track segment on the inside of a turn. The rule does not apply on the outside and on straights, pit entry and exit
		// count as well as track.
		tTrackSeg *mainseg = car->_trkPos.seg;
		bool pit = false;
		tTrackPitInfo pitInfo = track->pits;
		tdble toborder = 0.0f;
		tdble minradius = 1.0f;

		if (mainseg->type != TR_STR) {
			if (track->pits.type == TR_PIT_ON_TRACK_SIDE) {
				if (pitInfo.pitEntry->id < pitInfo.pitExit->id) {
					if ((mainseg->id >= pitInfo.pitEntry->id) && (mainseg->id <= pitInfo.pitExit->id)) {
						pit = true;
					}
				} else {
					if ((mainseg->id >= pitInfo.pitEntry->id) || (mainseg->id <= pitInfo.pitExit->id)) {
						pit = true;
					}
				}
			}

			if (mainseg->type == TR_LFT) {
				if (!(pit && (pitInfo.side == TR_LFT))) {
					toborder = car->_trkPos.toLeft;
					minradius = mainseg->radiusl;
				}
			} else if (mainseg->type == TR_RGT) {
				if (!(pit && (pitInfo.side == TR_RGT))) {
					toborder = car->_trkPos.toRight;
					minradius = mainseg->radiusr;
				}
			}
		}

		tdble cuttinglimit = car->_dimension_y*0.7f;
		if (toborder < -cuttinglimit) {
			if (ReInfo->raceRules.enabled & RmRaceRules::CORNER_CUTTING_TIME_INVALIDATE) {
				if (ReInfo->s->_raceType != RM_TYPE_RACE && car->_commitBestLapTime) {
					ReRaceMsgSet("Cut corner, laptime invalidated", 5);
				}
				car->_commitBestLapTime = false;
			}
			if (ReInfo->s->_raceType == RM_TYPE_RACE && ReInfo->raceRules.enabled & RmRaceRules::CORNER_CUTTING_TIME_PENALTY) {
				// In race, apply additionally corner cutting time penalty
				minradius -= cuttinglimit;
				if (minradius > 1.0f) {
					car->_penaltyTime += car->pub.speed*RCM_MAX_DT_SIMU*(-toborder-cuttinglimit)/minradius;
				}			
			}
		}
	}

	if (car->_skillLevel < 3) {
		/* only for the pros */
		return;
	}

	penalty = GF_TAILQ_FIRST(&(car->_penaltyList));
	if (penalty) {
		if (car->_laps > penalty->lapToClear) {
			/* too late to clear the penalty, out of race */
			car->_state |= RM_CAR_STATE_ELIMINATED;
			return;
		}
	
		switch (penalty->penalty) {
			case RM_PENALTY_DRIVETHROUGH:
				snprintf(car->ctrl.msg[3], 32, "Drive Through Penalty");
				break;
			case RM_PENALTY_STOPANDGO:
				snprintf(car->ctrl.msg[3], 32, "Stop And Go Penalty");
				break;
			default:
				*(car->ctrl.msg[3]) = 0;
				break;
		}
		
		memcpy(car->ctrl.msgColor, color, sizeof(car->ctrl.msgColor));
	}
    
	if (prevSeg->raceInfo & TR_PITSTART) {
		/* just entered the pit lane */
		if (seg->raceInfo & TR_PIT) {
			/* may be a penalty can be cleaned up */
			if (penalty) {
				switch (penalty->penalty) {
					case RM_PENALTY_DRIVETHROUGH:
						snprintf(buf, BUFSIZE, "%s DRIVE THROUGH PENALTY CLEANING", car->_name);
						ReRaceMsgSet(buf, 5);
						rules->ruleState |= RM_PNST_DRIVETHROUGH;
						break;
					case RM_PENALTY_STOPANDGO:
						snprintf(buf, BUFSIZE, "%s STOP&GO PENALTY CLEANING", car->_name);
						ReRaceMsgSet(buf, 5);
						rules->ruleState |= RM_PNST_STOPANDGO;
						break;
				}
			}
		}
    } else if (prevSeg->raceInfo & TR_PIT) {
		if (seg->raceInfo & TR_PIT) {
			/* the car stopped in pits */
			if (car->_state & RM_CAR_STATE_PIT) {
				if (rules->ruleState & RM_PNST_DRIVETHROUGH) {
					/* it's not more a drive through */
					rules->ruleState &= ~RM_PNST_DRIVETHROUGH;
				} else if (rules->ruleState & RM_PNST_STOPANDGO) {
					rules->ruleState |= RM_PNST_STOPANDGO_OK;
				}
			} else {
				if(rules->ruleState & RM_PNST_STOPANDGO_OK && car->_pitStopType != RM_PIT_STOPANDGO) {
					rules->ruleState &= ~ ( RM_PNST_STOPANDGO | RM_PNST_STOPANDGO_OK );
				}
			}
		} else if (seg->raceInfo & TR_PITEND) {
			/* went out of the pit lane, check if the current penalty is cleared */
			if (rules->ruleState & (RM_PNST_DRIVETHROUGH | RM_PNST_STOPANDGO_OK)) {
				/* clear the penalty */
				snprintf(buf, BUFSIZE, "%s penalty cleared", car->_name);
				ReRaceMsgSet(buf, 5);
				penalty = GF_TAILQ_FIRST(&(car->_penaltyList));
				GF_TAILQ_REMOVE(&(car->_penaltyList), penalty, link);
				FREEZ(penalty);
			}
			
			rules->ruleState = 0;
		} else {
			/* went out of the pit lane illegally... */
			/* it's a new stop and go... */
			if (!(rules->ruleState & RM_PNST_STNGO)) {
				snprintf(buf, BUFSIZE, "%s STOP&GO PENALTY", car->_name);
				ReRaceMsgSet(buf, 5);
				penalty = (tCarPenalty*)calloc(1, sizeof(tCarPenalty));
				penalty->penalty = RM_PENALTY_STOPANDGO;
				penalty->lapToClear = car->_laps + 5;
				GF_TAILQ_INSERT_TAIL(&(car->_penaltyList), penalty, link);
				rules->ruleState = RM_PNST_STNGO;
			}
		}
    } else if (seg->raceInfo & TR_PITEND) {
		rules->ruleState = 0;
    } else if (seg->raceInfo & TR_PIT) {
		/* entrered the pits not from the pit entry... */
		/* it's a new stop and go... */
		if (!(rules->ruleState & RM_PNST_STNGO)) {
			snprintf(buf, BUFSIZE, "%s STOP&GO PENALTY", car->_name);
			ReRaceMsgSet(buf, 5);
			penalty = (tCarPenalty*)calloc(1, sizeof(tCarPenalty));
			penalty->penalty = RM_PENALTY_STOPANDGO;
			penalty->lapToClear = car->_laps + 5;
			GF_TAILQ_INSERT_TAIL(&(car->_penaltyList), penalty, link);
			rules->ruleState = RM_PNST_STNGO;
		}
    }

	if (seg->raceInfo & TR_SPEEDLIMIT) {
		if (!(rules->ruleState & (RM_PNST_SPD | RM_PNST_STNGO)) && (car->_speed_x > track->pits.speedLimit)) {
			snprintf(buf, BUFSIZE, "%s DRIVE THROUGH PENALTY", car->_name);
			ReRaceMsgSet(buf, 5);
			rules->ruleState |= RM_PNST_SPD;
			penalty = (tCarPenalty*)calloc(1, sizeof(tCarPenalty));
			penalty->penalty = RM_PENALTY_DRIVETHROUGH;
			penalty->lapToClear = car->_laps + 5;
			GF_TAILQ_INSERT_TAIL(&(car->_penaltyList), penalty, link);
		}
	}
}

extern uint8_t* pwritten_ghost;
extern uint8_t* prestart_ghost;
extern uint8_t* prelaunch_ghost;
extern uint8_t* ptrk_type_ghost;
extern float* psteer_ghost;
extern float* pacc_ghost;
extern float* pbrake_ghost;
extern float* pspeed_ghost;
extern float* pto_track_middle_ghost;
extern float* pangle_ghost;
extern float* pdist_raced_ghost;
extern float* ptoMarking_L_ghost;
extern float* ptoMarking_M_ghost;
extern float* ptoMarking_R_ghost;
extern float* ptoMarking_LL_ghost;
extern float* ptoMarking_ML_ghost;
extern float* ptoMarking_MR_ghost;
extern float* ptoMarking_RR_ghost;
extern float* psl_x_ghost;
extern float* psl_y_ghost;
extern float* psr_x_ghost;
extern float* psr_y_ghost;
extern float* pcar_x_ghost;
extern float* pcar_y_ghost;
extern float* phm_dist_raced_ghost;
extern float* pcar1_dist_raced_ghost;
extern float* pcar2_dist_raced_ghost;
extern float* pcar3_dist_raced_ghost;
extern float* pdm1_x_ghost;
extern float* pdm1_y_ghost;
extern float* pdm2_x_ghost;
extern float* pdm2_y_ghost;
extern float* pdm3_x_ghost;
extern float* pdm3_y_ghost;
extern float* pdm4_x_ghost;
extern float* pdm4_y_ghost;
extern float* pdm5_x_ghost;
extern float* pdm5_y_ghost;
extern float* pdm6_x_ghost;
extern float* pdm6_y_ghost;
extern float* pdm7_x_ghost;
extern float* pdm7_y_ghost;
extern float* pdm8_x_ghost;
extern float* pdm8_y_ghost;
extern float* pdm9_x_ghost;
extern float* pdm9_y_ghost;
extern float* pdm1_dist_raced_ghost;
extern float* pdm2_dist_raced_ghost;
extern float* pdm3_dist_raced_ghost;
extern float* pdm4_dist_raced_ghost;
extern float* pdm5_dist_raced_ghost;
extern float* pdm6_dist_raced_ghost;
extern float* pdm7_dist_raced_ghost;
extern float* pdm8_dist_raced_ghost;
extern float* pdm9_dist_raced_ghost;
extern float* pdm1_speed_ghost;
extern float* pdm2_speed_ghost;
extern float* pdm3_speed_ghost;
extern float* pdm4_speed_ghost;
extern float* pdm5_speed_ghost;
extern float* pdm6_speed_ghost;
extern float* pdm7_speed_ghost;
extern float* pdm8_speed_ghost;
extern float* pdm9_speed_ghost;
extern float* pdm1_to_track_middle_m_ghost;
extern float* pdm2_to_track_middle_m_ghost;
extern float* pdm3_to_track_middle_m_ghost;
extern float* pdm4_to_track_middle_m_ghost;
extern float* pdm5_to_track_middle_m_ghost;
extern float* pdm6_to_track_middle_m_ghost;
extern float* pdm7_to_track_middle_m_ghost;
extern float* pdm8_to_track_middle_m_ghost;
extern float* pdm9_to_track_middle_m_ghost;
extern uint8_t* pdata;

uint8_t* pwritten = NULL;
uint8_t* prestart = NULL;
uint8_t* prelaunch = NULL;
uint8_t* ptrk_type = NULL;
float* psteer = NULL;
float* pacc = NULL;
float* pbrake = NULL;
float* pspeed = NULL;
float* pto_track_middle = NULL;
float* pangle = NULL;
float* pdist_raced = NULL;
float* ptoMarking_L = NULL;
float* ptoMarking_M = NULL;
float* ptoMarking_R = NULL;
float* ptoMarking_LL = NULL;
float* ptoMarking_ML = NULL;
float* ptoMarking_MR = NULL;
float* ptoMarking_RR = NULL;
float* psl_x = NULL;
float* psl_y = NULL;
float* psr_x = NULL;
float* psr_y = NULL;
float* pcar_x = NULL;
float* pcar_y = NULL;
float* phm_dist_raced;
float* pcar1_dist_raced;
float* pcar2_dist_raced;
float* pcar3_dist_raced;
float* pdm1_x = NULL;
float* pdm1_y = NULL;
float* pdm2_x = NULL;
float* pdm2_y = NULL;
float* pdm3_x = NULL;
float* pdm3_y = NULL;
float* pdm4_x = NULL;
float* pdm4_y = NULL;
float* pdm5_x = NULL;
float* pdm5_y = NULL;
float* pdm6_x = NULL;
float* pdm6_y = NULL;
float* pdm7_x = NULL;
float* pdm7_y = NULL;
float* pdm8_x = NULL;
float* pdm8_y = NULL;
float* pdm9_x = NULL;
float* pdm9_y = NULL;
float* pdm1_dist_raced = NULL;
float* pdm2_dist_raced = NULL;
float* pdm3_dist_raced = NULL;
float* pdm4_dist_raced = NULL;
float* pdm5_dist_raced = NULL;
float* pdm6_dist_raced = NULL;
float* pdm7_dist_raced = NULL;
float* pdm8_dist_raced = NULL;
float* pdm9_dist_raced = NULL;
float* pdm1_speed = NULL;
float* pdm2_speed = NULL;
float* pdm3_speed = NULL;
float* pdm4_speed = NULL;
float* pdm5_speed = NULL;
float* pdm6_speed = NULL;
float* pdm7_speed = NULL;
float* pdm8_speed = NULL;
float* pdm9_speed = NULL;
float* pdm1_to_track_middle_m = NULL;
float* pdm2_to_track_middle_m = NULL;
float* pdm3_to_track_middle_m = NULL;
float* pdm4_to_track_middle_m = NULL;
float* pdm5_to_track_middle_m = NULL;
float* pdm6_to_track_middle_m = NULL;
float* pdm7_to_track_middle_m = NULL;
float* pdm8_to_track_middle_m = NULL;
float* pdm9_to_track_middle_m = NULL;

uint8_t img[image_width*image_height*3];
int count = 0;

static void
ReOneStep(double deltaTimeIncrement) // deltaTimeIncrement = RCM_MAX_DT_SIMU = 0.002
{
	if (pspeed == NULL){
		pwritten = pwritten_ghost;
		prestart = prestart_ghost; 
		prelaunch = prelaunch_ghost;
		ptrk_type = ptrk_type_ghost;
		psteer = psteer_ghost;
		pacc = pacc_ghost;
		pbrake = pbrake_ghost;
		pspeed = pspeed_ghost;
		pto_track_middle = pto_track_middle_ghost;
		pangle = pangle_ghost;
		pdist_raced = pdist_raced_ghost;
		ptoMarking_L = ptoMarking_L_ghost;
		ptoMarking_M = ptoMarking_M_ghost;
		ptoMarking_R = ptoMarking_R_ghost;
		ptoMarking_LL = ptoMarking_LL_ghost;
		ptoMarking_ML = ptoMarking_ML_ghost;
		ptoMarking_MR = ptoMarking_MR_ghost;
		ptoMarking_RR = ptoMarking_RR_ghost;
		psl_x = psl_x_ghost;
		psl_y = psl_y_ghost;
		psr_x = psr_x_ghost;
		psr_y = psr_y_ghost;
		pcar_x = pcar_x_ghost;
		pcar_y = pcar_y_ghost;
		phm_dist_raced = phm_dist_raced_ghost;
		pcar1_dist_raced = pcar1_dist_raced_ghost;
		pcar2_dist_raced = pcar2_dist_raced_ghost;
		pcar3_dist_raced = pcar3_dist_raced_ghost;
		pdm1_x = pdm1_x_ghost;
		pdm1_y = pdm1_y_ghost;
		pdm2_x = pdm2_x_ghost;
		pdm2_y = pdm2_y_ghost;
		pdm3_x = pdm3_x_ghost;
		pdm3_y = pdm3_y_ghost;
		pdm4_x = pdm4_x_ghost;
		pdm4_y = pdm4_y_ghost;
		pdm5_x = pdm5_x_ghost;
		pdm5_y = pdm5_y_ghost;
		pdm6_x = pdm6_x_ghost;
		pdm6_y = pdm6_y_ghost;
		pdm7_x = pdm7_x_ghost;
		pdm7_y = pdm7_y_ghost;
		pdm8_x = pdm8_x_ghost;
		pdm8_y = pdm8_y_ghost;
		pdm9_x = pdm9_x_ghost;
		pdm9_y = pdm9_y_ghost;
		pdm1_dist_raced = pdm1_dist_raced_ghost;
		pdm2_dist_raced = pdm2_dist_raced_ghost;
		pdm3_dist_raced = pdm3_dist_raced_ghost;
		pdm4_dist_raced = pdm4_dist_raced_ghost;
		pdm5_dist_raced = pdm5_dist_raced_ghost;
		pdm6_dist_raced = pdm6_dist_raced_ghost;
		pdm7_dist_raced = pdm7_dist_raced_ghost;
		pdm8_dist_raced = pdm8_dist_raced_ghost;
		pdm9_dist_raced = pdm9_dist_raced_ghost;
		pdm1_speed = pdm1_speed_ghost;
		pdm2_speed = pdm2_speed_ghost;
		pdm3_speed = pdm3_speed_ghost;
		pdm4_speed = pdm4_speed_ghost;
		pdm5_speed = pdm5_speed_ghost;
		pdm6_speed = pdm6_speed_ghost;
		pdm7_speed = pdm7_speed_ghost;
		pdm8_speed = pdm8_speed_ghost;
		pdm9_speed = pdm9_speed_ghost;
		pdm1_to_track_middle_m = pdm1_to_track_middle_m_ghost;
		pdm2_to_track_middle_m = pdm2_to_track_middle_m_ghost;
		pdm3_to_track_middle_m = pdm3_to_track_middle_m_ghost;
		pdm4_to_track_middle_m = pdm4_to_track_middle_m_ghost;
		pdm5_to_track_middle_m = pdm5_to_track_middle_m_ghost;
		pdm6_to_track_middle_m = pdm6_to_track_middle_m_ghost;
		pdm7_to_track_middle_m = pdm7_to_track_middle_m_ghost;
		pdm8_to_track_middle_m = pdm8_to_track_middle_m_ghost;
		pdm9_to_track_middle_m = pdm9_to_track_middle_m_ghost;
	}
	if (1){
		count ++;
		if (count > 5){
			count = 0;
			glReadPixels(0, 0, image_width, image_height, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)img);
			for (int h = 0; h < image_height; ++h){
				for (int w = 0; w < image_width; ++w){
					pdata[(h*image_width+w)*3+0] = img[((image_height-h-1)*image_width+w)*3+0];
           			pdata[(h*image_width+w)*3+1] = img[((image_height-h-1)*image_width+w)*3+1];
           			pdata[(h*image_width+w)*3+2] = img[((image_height-h-1)*image_width+w)*3+2];
				}
			}

		}
	}

	int i;
	tRobotItf *robot;
	tSituation *s = ReInfo->s;

	bool restartRequested = false;
	for (int i = 0; i < s->_ncars; ++i){
		if (s->cars[i]->ctrl.askRestart){
			restartRequested = true;
			s->cars[i]->ctrl.askRestart = false;
		}
	}

	if (restartRequested){
		ReRaceCleanup();
		ReInfo->_reState = RE_STATE_PRE_RACE;
		GfuiScreenActivate(ReInfo->_reGameScreen);
	}

	ReInfo->_reCurTime += deltaTimeIncrement * ReInfo->_reTimeMult; /* "Real" time */
	s->currentTime += deltaTimeIncrement; /* Simulated time */

	if (s->currentTime < 0) {
		/* no simu yet */
		ReInfo->s->_raceState = RM_RACE_PRESTART;
	} else if (ReInfo->s->_raceState == RM_RACE_PRESTART) {
		ReInfo->s->_raceState = RM_RACE_RUNNING;
		s->currentTime = 0.0; /* resynchronize */
		ReInfo->_reLastTime = 0.0;
	}

	START_PROFILE("rbDrive*");
	if ((s->currentTime - ReInfo->_reLastTime) >= RCM_MAX_DT_ROBOTS) {
		s->deltaTime = s->currentTime - ReInfo->_reLastTime;
		for (i = 0; i < s->_ncars; i++) {
			if ((s->cars[i]->_state & RM_CAR_STATE_NO_SIMU) == 0) {
				robot = s->cars[i]->robot;
				robot->rbDrive(robot->index, s->cars[i], s);
				// printf("collosion = %d\n", s->cars[i]->_collision_cnt);
			}
		}
		ReInfo->_reLastTime = s->currentTime;
	}
	STOP_PROFILE("rbDrive*");

	START_PROFILE("_reSimItf.update*");
	ReInfo->_reSimItf.update(s, deltaTimeIncrement, -1);
	for (i = 0; i < s->_ncars; i++) {
		ReManage(s->cars[i]);
	}
	STOP_PROFILE("_reSimItf.update*");

	if ((ReInfo->_displayMode != RM_DISP_MODE_NONE) && (ReInfo->_displayMode != RM_DISP_MODE_CONSOLE)) {
		ReRaceMsgUpdate();
	}
	ReSortCars();
}

void
ReStart(void)
{
    ReInfo->_reRunning = 1;
    ReInfo->_reCurTime = GfTimeClock() - RCM_MAX_DT_SIMU;
}

void
ReStop(void)
{
	ReInfo->_reGraphicItf.muteformenu();
    ReInfo->_reRunning = 0;
}

static void
reCapture(void)
{
	unsigned char *img;
	int sw, sh, vw, vh;
	tRmMovieCapture	*capture = &(ReInfo->movieCapture);
	const int BUFSIZE = 1024;
	char buf[BUFSIZE];
	
	GfScrGetSize(&sw, &sh, &vw, &vh);
	img = (unsigned char*)malloc(vw * vh * 3);
	if (img == NULL) {
		return;
	}

	glPixelStorei(GL_PACK_ROW_LENGTH, 0);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadBuffer(GL_FRONT);
	glReadPixels((sw-vw)/2, (sh-vh)/2, vw, vh, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)img);

	snprintf(buf, BUFSIZE, "%s/torcs-%4.4d-%8.8d.png", capture->outputBase, capture->currentCapture, capture->currentFrame++);
	GfImgWritePng(img, buf, vw, vh);
	free(img);
}


int
ReUpdate(void)
{
	double t;
	tRmMovieCapture	*capture;
	int mode = RM_ASYNC;
	int i;
	const int MAXSTEPS = 2000;
	
	START_PROFILE("ReUpdate");
	ReInfo->_refreshDisplay = 0;
	switch (ReInfo->_displayMode) {
		case RM_DISP_MODE_NORMAL:
			t = GfTimeClock();
			
			i = 0;
			START_PROFILE("ReOneStep*");
			while ((ReInfo->_reRunning && ((t - ReInfo->_reCurTime) > RCM_MAX_DT_SIMU)) && MAXSTEPS > i++) {
				ReOneStep(RCM_MAX_DT_SIMU);
			}
			STOP_PROFILE("ReOneStep*");

			if (i > MAXSTEPS) {
				// Cannot keep up with time warp, reset time to avoid lag when running slower again
				ReInfo->_reCurTime = GfTimeClock();
			}
			
			GfuiDisplay();
			ReInfo->_reGraphicItf.refresh(ReInfo->s);
			glutPostRedisplay();	/* Callback -> reDisplay */
			break;

		case RM_DISP_MODE_NONE:
			// Just update view once per 2 seconds simulation time to avoid trouble with graphics cards
			// which are bad in buffer switching (e.g. ATI fglrx driver on Linux).
			t = ReInfo->_reCurTime;
			while ((t - ReInfo->_reCurTime + 2.0) > 0.0) {
				ReOneStep(RCM_MAX_DT_SIMU);
			}

			GfuiDisplay();
			glutPostRedisplay();	/* Callback -> reDisplay */
			break;

		case RM_DISP_MODE_CAPTURE:
			capture = &(ReInfo->movieCapture);
			while ((ReInfo->_reCurTime - capture->lastFrame) < capture->deltaFrame) {
				ReOneStep(capture->deltaSimu);
			}
			capture->lastFrame = ReInfo->_reCurTime;

			GfuiDisplay();
			ReInfo->_reGraphicItf.refresh(ReInfo->s);
			reCapture();
			glutPostRedisplay();	/* Callback -> reDisplay */
			break;

		case RM_DISP_MODE_CONSOLE:
			t = ReInfo->_reCurTime;
			while ((t - ReInfo->_reCurTime + 2.0) > 0.0) {
				ReOneStep(RCM_MAX_DT_SIMU);
			}
			mode = RM_SYNC;
			break;

	}
	STOP_PROFILE("ReUpdate");

	return mode;
}

void
ReTimeMod (void *vcmd)
{
	long cmd = (long)vcmd;

	switch ((int)cmd) {
		case 0:
			ReInfo->_reTimeMult *= 2.0;
			if (ReInfo->_reTimeMult > 64.0) {
				ReInfo->_reTimeMult = 64.0;
			}
			break;
		case 1:
			ReInfo->_reTimeMult *= 0.5;
			if (ReInfo->_reTimeMult < 1.0f/128.0f) {
				ReInfo->_reTimeMult = 1.0f/128.0f;
			}
			break;
		case 2:
			default:
			ReInfo->_reTimeMult = 1.0;
			break;
	}

	const int BUFSIZE = 1024;
	char buf[BUFSIZE];
	
	snprintf(buf, BUFSIZE, "Time x%.2f", 1.0 / ReInfo->_reTimeMult);
	ReRaceMsgSet(buf, 5);
}
