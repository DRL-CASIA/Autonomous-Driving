/***************************************************************************

    file                 : track4.cpp
    created              : Sat May 18 12:46:26 CEST 2002
    copyright            : (C) 2002-2015 by Eric Espie, Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: track4.cpp,v 1.15.2.18 2015/04/18 07:59:01 berniw Exp $

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
    Track loader for tracks of version 4.
    @author Bernhard Wymann, Eric Espie
    @version    $Id: track4.cpp,v 1.15.2.18 2015/04/18 07:59:01 berniw Exp $
*/

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include <tgf.h>
#include <robottools.h>
#include <track.h>
#include <portability.h>
#include "trackinc.h"

static tdble	xmin, xmax, ymin, ymax, zmin, zmax;

#define TSTX(x)	do {				\
    if (xmin > (x)) xmin = (x);			\
    if (xmax < (x)) xmax = (x);			\
    } while (0)

#define TSTY(y)	do {				\
    if (ymin > (y)) ymin = (y);			\
    if (ymax < (y)) ymax = (y);			\
    } while (0)

#define TSTZ(z)	do {				\
    if (zmin > (z)) zmin = (z);			\
    if (zmax < (z)) zmax = (z);			\
    } while (0)

/*
 * Sides global variables
 */
static const char *SectSide[2]    = {TRK_SECT_RSIDE, TRK_SECT_LSIDE};
static const char *SectBorder[2]  = {TRK_SECT_RBORDER, TRK_SECT_LBORDER};
static const char *SectBarrier[2] = {TRK_SECT_RBARRIER, TRK_SECT_LBARRIER};

static const char *ValStyle[] = {TRK_VAL_PLAN, TRK_VAL_CURB, TRK_VAL_WALL, TRK_VAL_FENCE, TRK_VAL_FENCE}; // TODO: TRK_VAL_FENCE 2 times, remove?

static tdble sideEndWidth[2];
static tdble sideStartWidth[2];
static int sideBankType[2];
static const char *sideMaterial[2];
static tTrackSurface *sideSurface[2];

static int envIndex;
static tdble DoVfactor=1.0;

static tdble borderWidth[2];
static tdble borderHeight[2];
static int borderStyle[2];
static const char *borderMaterial[2];
static tTrackSurface *borderSurface[2];

static tdble barrierWidth[2];
static tdble barrierHeight[2];
static int barrierStyle[2];
static const char *barrierMaterial[2];
static tTrackSurface *barrierSurface[2];

static tdble	GlobalStepLen = 0;

// Function prototypes
static void initSideForTurn(
	const int turntype,
	tTrackSeg* const curBorder,
	const tTrackSeg* const curSeg,
	const int side,
	const int bankingtype,
	const tdble startwidth,
	const tdble endwidth,
	const tdble maxwidth
);

static void initSideForStraight(
	tTrackSeg* const curBorder,
	const tTrackSeg* const curSeg,
	const int side,
	const int bankingtype,
	const tdble startwidth,
	const tdble endwidth
);

static tTrackSeg* commonSideInit(
	tTrackSeg* const curSeg,
	const int side,
	const int lefttype,
	const int righttype,
	const int bankingtype,
	const tdble startwidth,
	const tdble endwidth,
	tTrackSurface* surface,
	const tdble borderheight,
	const int borderstyle
);


/** Gets surface properties based on the material.

	If the material is not known, then a new material is created, initialized and attached to tTrack.

	@param[in] trackHandle Handle to track parameter set
	@param[in,out] theTrack Pointer to track structure
	@param[in] material Material name of the surface

	@return tTrackSurface* for the material
 */
static tTrackSurface* getTrackSurface(void *trackHandle, tTrack *theTrack, const char *material)
{
	const int BUFSIZE = 256;
	char path[BUFSIZE];
	tTrackSurface *curSurf;

	// Search within existing surfaces
	curSurf = theTrack->surfaces;
	while (curSurf) {
		if (strcmp(curSurf->material, material) == 0) {
			return curSurf;
		}
		curSurf = curSurf->next;
	}

	// Create a new surface
	curSurf = (tTrackSurface*)malloc(sizeof(tTrackSurface));
	if (!curSurf) {
		GfFatal("AddTrackSurface: Memory allocation failed\n");
	}

	curSurf->material = material;
	snprintf(path, BUFSIZE, "%s/%s", TRK_SECT_SURFACES, material);
	curSurf->kFriction     = GfParmGetNum(trackHandle, path, TRK_ATT_FRICTION, (char*)NULL, 0.8f);
	curSurf->kRollRes      = GfParmGetNum(trackHandle, path, TRK_ATT_ROLLRES, (char*)NULL, 0.001f);
	curSurf->kRoughness    = GfParmGetNum(trackHandle, path, TRK_ATT_ROUGHT, (char*)NULL, 0.0f) /  2.0f;
	curSurf->kRoughWaveLen = 2.0 * PI / GfParmGetNum(trackHandle, path, TRK_ATT_ROUGHTWL, (char*)NULL, 1.0f);
	curSurf->kDammage      = GfParmGetNum(trackHandle, path, TRK_ATT_DAMMAGE, (char*)NULL, 10.0f);
	curSurf->kRebound      = GfParmGetNum(trackHandle, path, TRK_ATT_REBOUND, (char*)NULL, 0.5f);

	curSurf->next = theTrack->surfaces;
	theTrack->surfaces = curSurf;

	return curSurf;
}


/** Reads default side/border track segment parameters.

	@param[in] trackHandle Handle to track parameter set
	@param[in,out] theTrack Pointer to track structure
 */
static void readDefaultSideParameters(void *trackHandle, tTrack *theTrack)
{
	int side;
	const char *style;
	const int BUFSIZE = 256;
	char path[BUFSIZE];
    
	// Side: right and left (see SectSide, SectBorder, SectBarrier)
    for (side = 0; side < 2; side++) {
		// Side parameters
		snprintf(path, BUFSIZE, "%s/%s", TRK_SECT_MAIN, SectSide[side]);
		sideMaterial[side] = GfParmGetStr(trackHandle, path, TRK_ATT_SURF, TRK_VAL_GRASS);
		sideSurface[side] = getTrackSurface(trackHandle, theTrack, sideMaterial[side]);
		sideEndWidth[side] = GfParmGetNum(trackHandle, path, TRK_ATT_WIDTH, (char*)NULL, 0.0);
		
		// Banking of sides
		if (strcmp(TRK_VAL_LEVEL, GfParmGetStr(trackHandle, path, TRK_ATT_BANKTYPE, TRK_VAL_LEVEL)) == 0) {
			sideBankType[side] = 0;
		} else {
			sideBankType[side] = 1;
		}

		// Border parameters
		snprintf(path, BUFSIZE, "%s/%s", TRK_SECT_MAIN, SectBorder[side]);
		borderMaterial[side] = GfParmGetStr(trackHandle, path, TRK_ATT_SURF, TRK_VAL_GRASS);
		borderSurface[side] = getTrackSurface(trackHandle, theTrack, borderMaterial[side]);
		borderWidth[side] = GfParmGetNum(trackHandle, path, TRK_ATT_WIDTH, (char*)NULL, 0.0);
		borderHeight[side] = GfParmGetNum(trackHandle, path, TRK_ATT_HEIGHT, (char*)NULL, 0.0);
		style = GfParmGetStr(trackHandle, path, TRK_ATT_STYLE, TRK_VAL_PLAN);
		
		if (strcmp(style, TRK_VAL_PLAN) == 0) {
			borderStyle[side] = TR_PLAN;
		} else if (strcmp(style, TRK_VAL_CURB) == 0) {
			borderStyle[side] = TR_CURB;
		} else {
			borderStyle[side] = TR_WALL;
		}

		// Barrier parameters
		snprintf(path, BUFSIZE, "%s/%s", TRK_SECT_MAIN, SectBarrier[side]);
		barrierMaterial[side] = GfParmGetStr(trackHandle, path, TRK_ATT_SURF, TRK_VAL_BARRIER);
		barrierSurface[side] = getTrackSurface(trackHandle, theTrack, barrierMaterial[side]);
		barrierHeight[side] = GfParmGetNum(trackHandle, path, TRK_ATT_HEIGHT, (char*)NULL, 0.6f);
		style = GfParmGetStr(trackHandle, path, TRK_ATT_STYLE, TRK_VAL_FENCE);
		if (strcmp(style, TRK_VAL_FENCE) == 0) {
			barrierStyle[side] = TR_FENCE;
			barrierWidth[side] = 0;
		} else {
			barrierStyle[side] = TR_WALL;
			barrierWidth[side] = GfParmGetNum(trackHandle, path, TRK_ATT_WIDTH, (char*)NULL, 0.5);
		}
    }
}


/** Adds the specified side/border/barrier segments to given main track segement.
	
	@param[in,out] curSeg Main track segment to process
	@param[in] trackHandle Handle to track parameter set
	@param[in] theTrack Pointer to track structure
	@param[in] curStep Current step (for segment subdivision)
	@param[in] steps Number of steps (segment subdivisions)
 */
static void addSides(tTrackSeg *curSeg, void *trackHandle, tTrack *theTrack, int curStep, int steps)
{
	tTrackSeg *mSeg;
	tTrackBarrier *curBarrier;
	tdble w, sw, ew, bw;
	int side;
	const char *style;
	tdble Kew;
	const int BUFSIZE = 256;
	char path[BUFSIZE];
	char path2[BUFSIZE];
	char *segName;

	mSeg = curSeg;

	snprintf(path, BUFSIZE, "%s/%s", TRK_SECT_MAIN, TRK_LST_SEGMENTS);
	segName = GfParmListGetCurEltName(trackHandle, path);
	snprintf(path, BUFSIZE, "%s/%s/%s", TRK_SECT_MAIN, TRK_LST_SEGMENTS, segName);
	
	for (side = 0; side < 2; side++) {
		curSeg = mSeg;

		if (curStep == 0) {
			// Initialize on the first step
			// Side parameters
			snprintf(path2, BUFSIZE, "%s/%s", path, SectSide[side]);
			sw = GfParmGetNum(trackHandle, path2, TRK_ATT_SWIDTH, (char*)NULL, sideEndWidth[side]);
			w = GfParmGetNum(trackHandle, path2, TRK_ATT_WIDTH, (char*)NULL, sw);
			ew = GfParmGetNum(trackHandle, path2, TRK_ATT_EWIDTH, (char*)NULL, w);
			
			sideStartWidth[side] = sw;
			sideEndWidth[side] = ew;
			sideMaterial[side] = GfParmGetStr(trackHandle, path2, TRK_ATT_SURF, sideMaterial[side]);
			sideSurface[side] = getTrackSurface(trackHandle, theTrack, sideMaterial[side]);

			// Border parameters
			snprintf(path2, BUFSIZE, "%s/%s", path, SectBorder[side]);
			bw = GfParmGetNum(trackHandle, path2, TRK_ATT_WIDTH, (char*)NULL, borderWidth[side]);
			borderWidth[side] = bw;
			borderHeight[side] = GfParmGetNum(trackHandle, path2, TRK_ATT_HEIGHT, (char*)NULL, borderHeight[side]);
			borderMaterial[side] = GfParmGetStr(trackHandle, path2, TRK_ATT_SURF, borderMaterial[side]);
			borderSurface[side] = getTrackSurface(trackHandle, theTrack, borderMaterial[side]);
			style = GfParmGetStr(trackHandle, path2, TRK_ATT_STYLE, ValStyle[borderStyle[side]]);
			
			if (strcmp(style, TRK_VAL_PLAN) == 0) {
				borderStyle[side] = TR_PLAN;
			} else if (strcmp(style, TRK_VAL_CURB) == 0) {
				borderStyle[side] = TR_CURB;
			} else {
				borderStyle[side] = TR_WALL;
			}

			// Barrier parameters
			snprintf(path2, BUFSIZE, "%s/%s", path, SectBarrier[side]);
			barrierMaterial[side] = GfParmGetStr(trackHandle, path2, TRK_ATT_SURF, barrierMaterial[side]);
			barrierSurface[side] = getTrackSurface(trackHandle, theTrack, barrierMaterial[side]);
			barrierHeight[side] = GfParmGetNum(trackHandle, path2, TRK_ATT_HEIGHT, (char*)NULL, barrierHeight[side]);
			style = GfParmGetStr(trackHandle, path2, TRK_ATT_STYLE, ValStyle[barrierStyle[side]]);
			
			if (strcmp(style, TRK_VAL_FENCE) == 0) {
				barrierStyle[side] = TR_FENCE;
				barrierWidth[side] = 0;
			} else {
				barrierStyle[side] = TR_WALL;
				barrierWidth[side] = GfParmGetNum(trackHandle, path2, TRK_ATT_WIDTH, (char*)NULL, barrierWidth[side]);
			}
		} else {
			// Reset sw, ew, bw for steps after initialization
			sw = sideStartWidth[side];
			ew = sideEndWidth[side];
			bw = borderWidth[side];
		}

		// Calculate parameters considering the current step
		Kew = (ew - sw) / (tdble)steps;
		ew = sw + (tdble)(curStep+1) * Kew;
		sw = sw + (tdble)(curStep) * Kew;

		// Add borders
		if (bw > 0.0f) {
			curSeg = commonSideInit(curSeg, side, TR_LBORDER, TR_RBORDER, sideBankType[side], bw, bw,
				borderSurface[side], borderHeight[side], borderStyle[side]);
		}

		// Add sides
		if ((sw > 0.0f) || (ew > 0.0f)) {
			commonSideInit(curSeg, side, TR_LSIDE, TR_RSIDE, sideBankType[side], sw, ew,
				sideSurface[side], 0.0f, TR_PLAN);
		}

		// Add barriers
		curBarrier = (tTrackBarrier*)malloc(sizeof(tTrackBarrier));
		if (!curBarrier) {
			GfFatal("AddSides: memory allocation error");
		}
		curBarrier->style = barrierStyle[side];
		curBarrier->width = barrierWidth[side];
		curBarrier->height = barrierHeight[side];
		curBarrier->surface = barrierSurface[side];

		// Compute normal of barrier for side collisions.
		tTrackSeg *bseg = mSeg;
		int bstart, bend;
		float bsign;

		if (side == TR_SIDE_LFT) {
			bstart = TR_SL;
			bend = TR_EL;
			bsign = -1.0f;
		} else {
			bstart = TR_SR;
			bend = TR_ER;
			bsign = 1.0f;
		}

		while (bseg->side[side] != NULL) {
			bseg = bseg->side[side];
		}

		vec2f n(
			-(bseg->vertex[bend].y - bseg->vertex[bstart].y)*bsign,
			(bseg->vertex[bend].x - bseg->vertex[bstart].x)*bsign
		);

		n.normalize();
		curBarrier->normal = n;

		mSeg->barrier[side] = curBarrier;
    }
}


/** Segment coordinate normalization, track is moved to the origin of the coordinate system.

	@param[in,out] curSeg Segment to normalize
 */
static void normSeg(tTrackSeg *curSeg)
{
	curSeg->vertex[TR_SR].x -= xmin;
	curSeg->vertex[TR_SR].y -= ymin;
	curSeg->vertex[TR_SR].z -= zmin;
	curSeg->vertex[TR_SL].x -= xmin;
	curSeg->vertex[TR_SL].y -= ymin;
	curSeg->vertex[TR_SL].z -= zmin;
	curSeg->vertex[TR_ER].x -= xmin;
	curSeg->vertex[TR_ER].y -= ymin;
	curSeg->vertex[TR_ER].z -= zmin;
	curSeg->vertex[TR_EL].x -= xmin;
	curSeg->vertex[TR_EL].y -= ymin;
	curSeg->vertex[TR_EL].z -= zmin;
	curSeg->center.x -= xmin;
	curSeg->center.y -= ymin;
}


static void CreateSegRing(void *TrackHandle, tTrack *theTrack, int ext)
{
    int		j;
    int		segread, curindex;
    tdble	radius, radiusend = 0, dradius;
    tdble	innerradius;
    tdble	arc;
    tdble	length;
    tTrackSeg	*curSeg;
    tTrackSeg	*root;
    tdble	alf;
    tdble	xr, yr, newxr, newyr;
    tdble	xl, yl, newxl, newyl;
    tdble	cenx, ceny;
    tdble	width, wi2;
    tdble	x1, x2, y1, y2;
    tdble	al, alfl;
    tdble	zsl, zsr, zel, zer, zs, ze;
    tdble	bankings, bankinge, dz; //dzl, dzr;
    tdble	etgt, stgt;
    tdble	etgtl, stgtl;
    tdble	etgtr, stgtr;
    tdble	stepslg = 0;
    int		steps, curStep;
    const char        *segtype = (char*)NULL;
    const char	*material;
    tTrackSurface *surface;
    const char	*segName;
    int		type;
    const char	*profil;
    tdble	totLength;

    tdble	tl, dtl, T1l, T2l;
    tdble	tr, dtr, T1r, T2r;
    tdble	curzel, curzer, curArc, curLength, curzsl, curzsr;
    tdble	grade;

    void	*segNameHash = NULL;
	const int BUFSIZE = 256;
    char	path[BUFSIZE];
	
#define MAX_TMP_INTS	256
    int		mi[MAX_TMP_INTS];
    int		ind = 0;

    radius = arc = length = alf = xr = yr = newxr = newyr = xl = yl = 0;
    zel = zer = etgtl = etgtr = newxl = newyl = 0;
    type = 0;
    
    width = GfParmGetNum(TrackHandle, TRK_SECT_MAIN, TRK_ATT_WIDTH, (char*)NULL, 15.0);
    wi2 = width / 2.0;

    grade = -100000.0;
    root = (tTrackSeg*)NULL;
    totLength = 0;
    
    snprintf(path, BUFSIZE, "%s/%s", TRK_SECT_MAIN, TRK_LST_SEGMENTS);
	yl = width;
	zsl = zsr = zel = zer = zs = ze = 0.0;
	stgt = etgt = 0.0;
	stgtl = etgtl = 0.0;
	stgtr = etgtr = 0.0;    

    /* Main Track */
    material = GfParmGetStr(TrackHandle, TRK_SECT_MAIN, TRK_ATT_SURF, TRK_VAL_ASPHALT);
    surface = getTrackSurface(TrackHandle, theTrack, material);
    envIndex = 0;
    DoVfactor =1.0;
    readDefaultSideParameters(TrackHandle, theTrack);
    
    if (ext) {
		segNameHash = GfHashCreate(GF_HASH_TYPE_STR);
    }

    segread = 0;
    curindex = 0;
    GfParmListSeekFirst(TrackHandle, path);
    do {
		segtype = GfParmGetCurStr(TrackHandle, path, TRK_ATT_TYPE, NULL);
		if (segtype == 0) {
			continue;
		}
		segread++;
		
		zsl = zel;
		zsr = zer;
		TSTZ(zsl);
		TSTZ(zsr);
		
		/* Turn Marks */
		if (ext) {
			const char *marks = GfParmGetCurStr(TrackHandle, path, TRK_ATT_MARKS, NULL);
			ind = 0;
			if (marks) {
				char* tmpmarks = strdup(marks);
				char *s = strtok(tmpmarks, ";");
				while ((s != NULL) && (ind < MAX_TMP_INTS)) {
					mi[ind] = (int)strtol(s, NULL, 0);
					ind++;
					s = strtok(NULL, ";");
				}
				free(tmpmarks);
			}
		}
		
		/* surface change */
		material = GfParmGetCurStr(TrackHandle, path, TRK_ATT_SURF, material);
		surface = getTrackSurface(TrackHandle, theTrack, material);
		envIndex = (int) GfParmGetCurNum(TrackHandle, path, TRK_ATT_ENVIND, (char*)NULL, (float) (envIndex+1)) - 1;
		// TODO: is the (int) intended?
		DoVfactor = (float) ((int) GfParmGetCurNum(TrackHandle, path, TRK_ATT_DOVFACTOR, (char*)NULL, 1.0)) ;

		/* get segment type and length */
		if (strcmp(segtype, TRK_VAL_STR) == 0) {
			/* straight */
			length = GfParmGetCurNum(TrackHandle, path, TRK_ATT_LG, (char*)NULL, 0);
			type = TR_STR;
			radius = radiusend = 0;
		} else if (strcmp(segtype, TRK_VAL_LFT) == 0) {
			/* left curve */
			radius = GfParmGetCurNum(TrackHandle, path, TRK_ATT_RADIUS, (char*)NULL, 0);
			radiusend = GfParmGetCurNum(TrackHandle, path, TRK_ATT_RADIUSEND, (char*)NULL, radius);
			arc = GfParmGetCurNum(TrackHandle, path, TRK_ATT_ARC, (char*)NULL, 0);
			type = TR_LFT;
			length = (radius + radiusend) / 2.0 * arc;
		} else if (strcmp(segtype, TRK_VAL_RGT) == 0) {
			/* right curve */
			radius = GfParmGetCurNum(TrackHandle, path, TRK_ATT_RADIUS, (char*)NULL, 0);
			radiusend = GfParmGetCurNum(TrackHandle, path, TRK_ATT_RADIUSEND, (char*)NULL, radius);
			arc = GfParmGetCurNum(TrackHandle, path, TRK_ATT_ARC, (char*)NULL, 0);
			type = TR_RGT;
			length = (radius + radiusend) / 2.0 * arc;
		}
		segName = GfParmListGetCurEltName(TrackHandle, path);
		if (ext) {
			if (GfHashGetStr(segNameHash, segName)) {
			printf(">>>>>>>>> DUPLICATED SEGMENT NAME \"%s\" PLEASE CHANGE IT !!!!\n", segName);
			exit(1);
			}
			GfHashAddStr(segNameHash, segName, segName);
		}

		/* elevation and banking */
		zsl = GfParmGetCurNum(TrackHandle, path, TRK_ATT_ZSL, (char*)NULL, zsl);
		zsr = GfParmGetCurNum(TrackHandle, path, TRK_ATT_ZSR, (char*)NULL, zsr);
		zel = GfParmGetCurNum(TrackHandle, path, TRK_ATT_ZEL, (char*)NULL, zel);
		zer = GfParmGetCurNum(TrackHandle, path, TRK_ATT_ZER, (char*)NULL, zer);
		ze = zs = -100000.0;
		ze = GfParmGetCurNum(TrackHandle, path, TRK_ATT_ZE, (char*)NULL, ze);
		zs = GfParmGetCurNum(TrackHandle, path, TRK_ATT_ZS, (char*)NULL, zs);
		grade = GfParmGetCurNum(TrackHandle, path, TRK_ATT_GRADE, (char*)NULL, grade);
		if (zs != -100000.0) {
			zsr = zsl = zs;
		} else {
			zs = (zsl + zsr) / 2.0;
		}
		if (ze != -100000.0) {
			zer = zel = ze;
		} else if (grade != -100000.0) {
			ze = zs + length * grade;
		} else {
			ze = (zel + zer) / 2.0;
		}
		bankings = atan2(zsl - zsr, width);
		bankinge = atan2(zel - zer, width);
		bankings = GfParmGetCurNum(TrackHandle, path, TRK_ATT_BKS, (char*)NULL, bankings);
		bankinge = GfParmGetCurNum(TrackHandle, path, TRK_ATT_BKE, (char*)NULL, bankinge);
		dz = tan(bankings) * width / 2.0;
		zsl = zs + dz;
		zsr = zs - dz;
		dz = tan(bankinge) * width / 2.0;
		zel = ze + dz;
		zer = ze - dz;

		TSTZ(zsl);
		TSTZ(zsr);

		/* Get segment profil */
		profil = GfParmGetCurStr(TrackHandle, path, TRK_ATT_PROFIL, TRK_VAL_SPLINE);
		stgtl = etgtl;
		stgtr = etgtr;
		if (strcmp(profil, TRK_VAL_SPLINE) == 0) {
			steps = (int)GfParmGetCurNum(TrackHandle, path, TRK_ATT_PROFSTEPS, (char*)NULL, 1.0);
			if (steps == 1) {
			stepslg = GfParmGetCurNum(TrackHandle, path, TRK_ATT_PROFSTEPSLEN, (char*)NULL, GlobalStepLen);
			if (stepslg) {
				steps = (int)(length / stepslg) + 1;
			} else {
				steps = 1;
			}
			}
			stgtl = GfParmGetCurNum(TrackHandle, path, TRK_ATT_PROFTGTSL, (char*)NULL, stgtl);
			etgtl = GfParmGetCurNum(TrackHandle, path, TRK_ATT_PROFTGTEL, (char*)NULL, etgtl);
			stgtr = GfParmGetCurNum(TrackHandle, path, TRK_ATT_PROFTGTSR, (char*)NULL, stgtr);
			etgtr = GfParmGetCurNum(TrackHandle, path, TRK_ATT_PROFTGTER, (char*)NULL, etgtr);
			stgt = etgt = -100000.0;
			stgt = GfParmGetCurNum(TrackHandle, path, TRK_ATT_PROFTGTS, (char*)NULL, stgt);
			etgt = GfParmGetCurNum(TrackHandle, path, TRK_ATT_PROFTGTE, (char*)NULL, etgt);
			if (stgt != -100000.0) {
			stgtl = stgtr = stgt;
			}
			if (etgt != -100000.0) {
			etgtl = etgtr = etgt;
			}
		} else {
			steps = 1;
			stgtl = etgtl = (zel - zsl) / length;
			stgtr = etgtr = (zer - zsr) / length;
		}
		GfParmSetCurNum(TrackHandle, path, TRK_ATT_ID, (char*)NULL, (tdble)curindex);
		
		//dzl = zel - zsl;
		//dzr = zer - zsr;
		T1l = stgtl * length;
		T2l = etgtl * length;
		tl = 0.0;
		dtl = 1.0 / (tdble)steps;
		T1r = stgtr * length;
		T2r = etgtr * length;
		tr = 0.0;
		dtr = 1.0 / (tdble)steps;

		curStep = 0;
		curzel = zsl;
		curzer = zsr;
		curArc = arc / (tdble)steps;
		curLength = length / (tdble)steps;
		dradius = (radiusend - radius) / (tdble)steps;
		if (radiusend != radius) {
			/* Compute the correct curLength... */
			if (steps != 1) {
			dradius = (radiusend - radius) / (tdble)(steps - 1);
			tdble tmpAngle = 0;
			tdble tmpRadius = radius;
			for (curStep = 0; curStep < steps; curStep++) {
				tmpAngle += curLength / tmpRadius;
				tmpRadius += dradius;
			}
			curLength *= arc / tmpAngle;
			}
		}
		curStep = 0;

		while (curStep < steps) {
		    
			tl += dtl;
			tr += dtr;

			curzsl = curzel;
			curzel = TrackSpline(zsl, zel, T1l, T2l, tl);
		    
			curzsr = curzer;
			curzer = TrackSpline(zsr, zer, T1r, T2r, tr);
		    
			if (dradius != 0) {
			curArc = curLength / radius;
			}
		    
			/* allocate a new segment */
			curSeg = (tTrackSeg*)calloc(1, sizeof(tTrackSeg));
			if (root == NULL) {
			root = curSeg;
			curSeg->next = curSeg;
			curSeg->prev = curSeg;
			} else {
			curSeg->next = root->next;
			curSeg->next->prev = curSeg;
			curSeg->prev = root;
			root->next = curSeg;
			root = curSeg;
			}
			curSeg->type2 = TR_MAIN;
			curSeg->name = segName;
			curSeg->id = curindex;
			curSeg->width = curSeg->startWidth = curSeg->endWidth = width;
			curSeg->surface = surface;
			curSeg->envIndex = envIndex;
			curSeg->DoVfactor = DoVfactor;
			/*printf("curseg id =%d factor =%f\n",curSeg->id,curSeg->DoVfactor);*/
			curSeg->lgfromstart = totLength;
		    
			if (ext && ind) {
			int	*mrks = (int*)calloc(ind, sizeof(int));
			tSegExt	*segExt = (tSegExt*)calloc(1, sizeof(tSegExt));

			memcpy(mrks, mi, ind*sizeof(int));
			segExt->nbMarks = ind;
			segExt->marks = mrks;
			curSeg->ext = segExt;
			ind = 0;
			}
			

			switch (type) {
			case TR_STR:
			/* straight */
			curSeg->type = TR_STR;
			curSeg->length = curLength;

			newxr = xr + curLength * cos(alf);      /* find end coordinates */
			newyr = yr + curLength * sin(alf);
			newxl = xl + curLength * cos(alf);
			newyl = yl + curLength * sin(alf);

			curSeg->vertex[TR_SR].x = xr;
			curSeg->vertex[TR_SR].y = yr;
			curSeg->vertex[TR_SR].z = curzsr;

			curSeg->vertex[TR_SL].x = xl;
			curSeg->vertex[TR_SL].y = yl;
			curSeg->vertex[TR_SL].z = curzsl;

			curSeg->vertex[TR_ER].x = newxr;
			curSeg->vertex[TR_ER].y = newyr;
			curSeg->vertex[TR_ER].z = curzer;

			curSeg->vertex[TR_EL].x = newxl;
			curSeg->vertex[TR_EL].y = newyl;
			curSeg->vertex[TR_EL].z = curzel;

			curSeg->angle[TR_ZS] = alf;
			curSeg->angle[TR_ZE] = alf;
			curSeg->angle[TR_YR] = atan2(curSeg->vertex[TR_ER].z - curSeg->vertex[TR_SR].z, curLength);
			curSeg->angle[TR_YL] = atan2(curSeg->vertex[TR_EL].z - curSeg->vertex[TR_SL].z, curLength);
			curSeg->angle[TR_XS] = atan2(curSeg->vertex[TR_SL].z - curSeg->vertex[TR_SR].z, width);
			curSeg->angle[TR_XE] = atan2(curSeg->vertex[TR_EL].z - curSeg->vertex[TR_ER].z, width);
		    
			curSeg->Kzl = tan(curSeg->angle[TR_YR]);
			curSeg->Kzw = (curSeg->angle[TR_XE] - curSeg->angle[TR_XS]) / curLength;
			curSeg->Kyl = 0;

			curSeg->rgtSideNormal.x = -sin(alf);
			curSeg->rgtSideNormal.y = cos(alf);

			TSTX(newxr); TSTX(newxl);
			TSTY(newyr); TSTY(newyl);

			break;
		    
			case TR_LFT:
			/* left curve */
			curSeg->type = TR_LFT;
			curSeg->radius = radius;
			curSeg->radiusr = radius + wi2;
			curSeg->radiusl = radius - wi2;
			curSeg->arc = curArc;
			curSeg->length = curLength;
		    
			innerradius = radius - wi2; /* left side aligned */
			cenx = xl - innerradius * sin(alf);  /* compute center location: */
			ceny = yl + innerradius * cos(alf);
			curSeg->center.x = cenx;
			curSeg->center.y = ceny;

			curSeg->angle[TR_ZS] = alf;
			curSeg->angle[TR_CS] = alf - PI / 2.0;
			alf += curArc;
			curSeg->angle[TR_ZE] = alf;

			newxl = cenx + innerradius * sin(alf);   /* location of end */
			newyl = ceny - innerradius * cos(alf);
			newxr = cenx + (innerradius + width) * sin(alf);   /* location of end */
			newyr = ceny - (innerradius + width) * cos(alf);

			curSeg->vertex[TR_SR].x = xr;
			curSeg->vertex[TR_SR].y = yr;
			curSeg->vertex[TR_SR].z = curzsr;

			curSeg->vertex[TR_SL].x = xl;
			curSeg->vertex[TR_SL].y = yl;
			curSeg->vertex[TR_SL].z = curzsl;

			curSeg->vertex[TR_ER].x = newxr;
			curSeg->vertex[TR_ER].y = newyr;
			curSeg->vertex[TR_ER].z = curzer;

			curSeg->vertex[TR_EL].x = newxl;
			curSeg->vertex[TR_EL].y = newyl;
			curSeg->vertex[TR_EL].z = curzel;

			curSeg->angle[TR_YR] = atan2(curSeg->vertex[TR_ER].z - curSeg->vertex[TR_SR].z, curArc * (innerradius + width));
			curSeg->angle[TR_YL] = atan2(curSeg->vertex[TR_EL].z - curSeg->vertex[TR_SL].z, curArc * innerradius);
			curSeg->angle[TR_XS] = atan2(curSeg->vertex[TR_SL].z - curSeg->vertex[TR_SR].z, width);
			curSeg->angle[TR_XE] = atan2(curSeg->vertex[TR_EL].z - curSeg->vertex[TR_ER].z, width);

			curSeg->Kzl = tan(curSeg->angle[TR_YR]) * (innerradius + width);
			curSeg->Kzw = (curSeg->angle[TR_XE] - curSeg->angle[TR_XS]) / curArc;
			curSeg->Kyl = 0;
		    
			/* to find the boundary */
			al = curArc / 36.0;
			alfl = curSeg->angle[TR_CS];

			for (j = 0; j < 36; j++) {
				alfl += al;
				x1 = curSeg->center.x + (innerradius) * cos(alfl);   /* location of end */
				y1 = curSeg->center.y + (innerradius) * sin(alfl);
				x2 = curSeg->center.x + (innerradius + width) * cos(alfl);   /* location of end */
				y2 = curSeg->center.y + (innerradius + width) * sin(alfl);
				TSTX(x1); TSTX(x2);
				TSTY(y1); TSTY(y2);
			}

			break;
		    
			case TR_RGT:
			/* right curve */
			curSeg->type = TR_RGT;
			curSeg->radius = radius;
			curSeg->radiusr = radius - wi2;
			curSeg->radiusl = radius + wi2;
			curSeg->arc = curArc;
			curSeg->length = curLength;

			innerradius = radius - wi2; /* right side aligned */
			cenx = xr + innerradius * sin(alf);  /* compute center location */
			ceny = yr - innerradius * cos(alf);
			curSeg->center.x = cenx;
			curSeg->center.y = ceny;

			curSeg->angle[TR_ZS] = alf;
			curSeg->angle[TR_CS] = alf + PI / 2.0;
			alf -= curSeg->arc;
			curSeg->angle[TR_ZE] = alf;

			newxl = cenx - (innerradius + width) * sin(alf);   /* location of end */
			newyl = ceny + (innerradius + width) * cos(alf);
			newxr = cenx - innerradius * sin(alf);   /* location of end */
			newyr = ceny + innerradius * cos(alf);

			curSeg->vertex[TR_SR].x = xr;
			curSeg->vertex[TR_SR].y = yr;
			curSeg->vertex[TR_SR].z = curzsr;

			curSeg->vertex[TR_SL].x = xl;
			curSeg->vertex[TR_SL].y = yl;
			curSeg->vertex[TR_SL].z = curzsl;

			curSeg->vertex[TR_ER].x = newxr;
			curSeg->vertex[TR_ER].y = newyr;
			curSeg->vertex[TR_ER].z = curzer;

			curSeg->vertex[TR_EL].x = newxl;
			curSeg->vertex[TR_EL].y = newyl;
			curSeg->vertex[TR_EL].z = curzel;

			curSeg->angle[TR_YR] = atan2(curSeg->vertex[TR_ER].z - curSeg->vertex[TR_SR].z, curArc * innerradius);
			curSeg->angle[TR_YL] = atan2(curSeg->vertex[TR_EL].z - curSeg->vertex[TR_SL].z, curArc * (innerradius + width));
			curSeg->angle[TR_XS] = atan2(curSeg->vertex[TR_SL].z - curSeg->vertex[TR_SR].z, width);
			curSeg->angle[TR_XE] = atan2(curSeg->vertex[TR_EL].z - curSeg->vertex[TR_ER].z, width);

			curSeg->Kzl = tan(curSeg->angle[TR_YR]) * innerradius;
			curSeg->Kzw = (curSeg->angle[TR_XE] - curSeg->angle[TR_XS]) / curArc;
			curSeg->Kyl = 0;

			/* to find the boundaries */
			al = curSeg->arc / 36.0;
			alfl = curSeg->angle[TR_CS];

			for (j = 0; j < 36; j++) {
				alfl -= al;
				x1 = curSeg->center.x + (innerradius + width) * cos(alfl);   /* location of end */
				y1 = curSeg->center.y + (innerradius + width) * sin(alfl);
				x2 = curSeg->center.x + innerradius * cos(alfl);   /* location of end */
				y2 = curSeg->center.y + innerradius * sin(alfl);
				TSTX(x1); TSTX(x2);
				TSTY(y1); TSTY(y2);
			}
			break;

			}

			addSides(curSeg, TrackHandle, theTrack, curStep, steps);

			totLength += curSeg->length;
			xr = newxr;
			yr = newyr;
			xl = newxl;
			yl = newyl;
			curindex++;
			curStep++;
			if (type != TR_STR) {
	/* 		printf("radius = %f arc = %f steps %d, length %f, stepslg %f\n", radius, RAD2DEG(curArc), steps, length, curLength); */
			radius += dradius;
			}
		}

    } while (GfParmListSeekNext(TrackHandle, path) == 0);

    if (ext) {
	GfHashRelease(segNameHash, NULL);
    }

    /* printf("\n"); */
    

    theTrack->seg = root;
    theTrack->length = totLength;
    theTrack->nseg = curindex;
}




/*
 * Read version 4 track segments
 */
void 
ReadTrack4(tTrack *theTrack, void *TrackHandle, tRoadCam **camList, int ext)
{
    int			i;
    tTrackSeg		*curSeg = NULL, *mSeg;
    tTrackSeg		*curSeg2;
    tTrackSeg		*pitEntrySeg = NULL;
    tTrackSeg		*pitExitSeg = NULL;
    tTrackSeg		*pitStart = NULL;
    tTrackSeg		*pitEnd = NULL;
    tTrackSeg		*curPitSeg = NULL;
    tTrackPitInfo	*pits;
    const char		*segName;
    int			segId;
    tRoadCam		*curCam;
    tTrkLocPos		trkPos;
    int			found = 0;
    const char		*paramVal;
    const int BUFSIZE = 256;
	char		path[BUFSIZE];
    char		path2[BUFSIZE];
    int			changeSeg;
    tdble		offset = 0;
    tdble		toStart;

    xmin = xmax = ymin = ymax = zmin = zmax = 0.0;

    GlobalStepLen = GfParmGetNum(TrackHandle, TRK_SECT_MAIN, TRK_ATT_PROFSTEPSLEN, (char*)NULL, 0);
    
    CreateSegRing(TrackHandle, theTrack, ext);

    /* PITS */
    pits = &(theTrack->pits);
    snprintf(path2, BUFSIZE, "%s/%s", TRK_SECT_MAIN, TRK_SECT_PITS);
    segName = GfParmGetStr(TrackHandle, path2, TRK_ATT_ENTRY, NULL);
    
    if (segName != 0) {
	snprintf(path, BUFSIZE, "%s/%s/%s", TRK_SECT_MAIN, TRK_LST_SEGMENTS, segName);
	segId = (int)GfParmGetNum(TrackHandle, path, TRK_ATT_ID, (char*)NULL, -1);
	pitEntrySeg = theTrack->seg;
	found = 0;
	for(i = 0; i < theTrack->nseg; i++)  {
	    if (pitEntrySeg->id == segId) {
		found = 1;
	    } else if (found) {
		pitEntrySeg = pitEntrySeg->next;
		break;
	    }
	    pitEntrySeg = pitEntrySeg->prev;
	}
	if (!found) {
	    pitEntrySeg = NULL;
	}

	/* Pits Exit */
	segName = GfParmGetStr(TrackHandle, path2, TRK_ATT_EXIT, NULL);
	if (segName != 0) {
	    /* Search backward the last segment with that name */
	    pitExitSeg = theTrack->seg; /* last track segment */
	    found = 0;
	    for(i = 0; i < theTrack->nseg; i++)  {
		/* set the flag on the last segment of pit_exit */
		if (!strcmp(segName, pitExitSeg->name)) {
		    found = 1;
		    break;
		}
		pitExitSeg = pitExitSeg->prev;
	    }
	    if (!found) {
		pitExitSeg = NULL;
	    }
	}

	/* Pits Start */
	segName = GfParmGetStr(TrackHandle, path2, TRK_ATT_START, NULL);
	if (segName != 0) {
	    pitStart = theTrack->seg;
	    found = 0;
	    for(i = 0; i < theTrack->nseg; i++)  {
		if (!strcmp(segName, pitStart->name)) {
		    found = 1;
		} else if (found) {
		    pitStart = pitStart->next;
		    break;
		}
		pitStart = pitStart->prev;
	    }
	    if (!found) {
		pitStart = NULL;
	    }
	}

	/* Pits End */
	segName = GfParmGetStr(TrackHandle, path2, TRK_ATT_END, NULL);
	if (segName != 0) {
	    /* Search backward the last segment with that name */
	    pitEnd = theTrack->seg; /* last track segment */
	    found = 0;
	    for(i = 0; i < theTrack->nseg; i++)  {
		if (!strcmp(segName, pitEnd->name)) {
		    found = 1;
		    break;
		}
		pitEnd = pitEnd->prev;
	    }
	    if (!found) {
		pitEnd = NULL;
	    }
	}
	paramVal = GfParmGetStr(TrackHandle, path2, TRK_ATT_SIDE, "right");
	if (strcmp(paramVal, "right") == 0) {
	    pits->side = TR_RGT;
	} else {
	    pits->side = TR_LFT;
	}
	pits->speedLimit = GfParmGetNum(TrackHandle, path2, TRK_ATT_SPD_LIM, (char*)NULL, 25.0);
	if ((pitEntrySeg != NULL) && (pitExitSeg != NULL) && (pitStart != NULL) && (pitEnd != NULL)) {
	    pits->pitEntry = pitEntrySeg;
	    pits->pitExit  = pitExitSeg;
	    pits->pitStart = pitStart;
	    pits->pitEnd = pitEnd;
	    pitEntrySeg->raceInfo |= TR_PITENTRY;
	    pitExitSeg->raceInfo |= TR_PITEXIT;
	    pits->len   = GfParmGetNum(TrackHandle, path2, TRK_ATT_LEN, (char*)NULL, 15.0);
	    pits->width = GfParmGetNum(TrackHandle, path2, TRK_ATT_WIDTH, (char*)NULL, 5.0);
	    found = 1;
	} else {
	    found = 0;
	}
    }

    if (found) {
	pits->type     = TR_PIT_ON_TRACK_SIDE;
	pits->nPitSeg  = 0;
	if (pitStart->lgfromstart > pitEnd->lgfromstart) {
	    pits->nMaxPits = (int)((theTrack->length - pitStart->lgfromstart +
				    pitEnd->lgfromstart + pitEnd->length + pits->len / 2.0) / pits->len);
	} else {
	    pits->nMaxPits = (int)((pitEnd->lgfromstart + pitEnd->length
				    - pitStart->lgfromstart + pits->len / 2.0) / pits->len);
	}
	pits->driversPits = (tTrackOwnPit*)calloc(pits->nMaxPits, sizeof(tTrackOwnPit));

	mSeg = pitStart->prev;
	changeSeg = 1;
	toStart = 0;
	i = 0; 
	while (i < pits->nMaxPits) {
	    pits->driversPits[i].pos.type = TR_LPOS_MAIN;
	    if (changeSeg) {
		changeSeg = 0;
		offset = 0;
		mSeg = mSeg->next;
		if (toStart >= mSeg->length) {
		    toStart -= mSeg->length;
		    changeSeg = 1;
		    continue;
		}
		switch (pits->side) {
		case TR_RGT:
		    curPitSeg = mSeg->rside;
		    if (curPitSeg->rside) {
			offset = curPitSeg->width;
			curPitSeg = curPitSeg->rside;
		    }
		    break;
		case TR_LFT:
		    curPitSeg = mSeg->lside;
		    if (curPitSeg->lside) {
			offset = curPitSeg->width;
			curPitSeg = curPitSeg->lside;
		    }
		    break;
		}
	    }
	    
	    pits->driversPits[i].pos.seg = mSeg;
	    pits->driversPits[i].pos.toStart = toStart + pits->len / 2.0;
	    switch (pits->side) {
	    case TR_RGT:
		pits->driversPits[i].pos.toRight  = -offset - RtTrackGetWidth(curPitSeg, toStart) + pits->width / 2.0;
		pits->driversPits[i].pos.toLeft   = mSeg->width - pits->driversPits[i].pos.toRight;
		pits->driversPits[i].pos.toMiddle = mSeg->width / 2.0 - pits->driversPits[i].pos.toRight;
		break;
	    case TR_LFT:
		pits->driversPits[i].pos.toLeft   = -offset - RtTrackGetWidth(curPitSeg, toStart) + pits->width / 2.0;
		pits->driversPits[i].pos.toRight  = mSeg->width - pits->driversPits[i].pos.toLeft;
		pits->driversPits[i].pos.toMiddle = mSeg->width / 2.0 - pits->driversPits[i].pos.toLeft;
		break;
	    }
	    toStart += pits->len;
	    if (toStart >= mSeg->length) {
		toStart -= mSeg->length;
		changeSeg = 1;
	    }
	    i++;
	}

	for (mSeg = pitStart->prev; mSeg != pitEnd->next->next; mSeg = mSeg->next) {
	    curSeg2 = NULL;
	    switch(pits->side) {
	    case TR_RGT:
		curSeg = mSeg->rside;
		curSeg2 = curSeg->rside;
		if ((mSeg != pitStart->prev) && (mSeg != pitEnd->next)) {
		    mSeg->barrier[0]->style = TR_PITBUILDING;
		}
		break;
	    case TR_LFT:
		curSeg = mSeg->lside;
		curSeg2 = curSeg->lside;
		if ((mSeg != pitStart->prev) && (mSeg != pitEnd->next)) {
		    mSeg->barrier[1]->style = TR_PITBUILDING;
		}
		break;
	    }
	    if ((mSeg != pitStart->prev) && (mSeg != pitEnd->next)) {
		curSeg->raceInfo |= TR_PIT | TR_SPEEDLIMIT;
		if (curSeg2) {
		    curSeg2->raceInfo |= TR_PIT | TR_SPEEDLIMIT;
		}
	    } else if (mSeg == pitStart->prev) {
		curSeg->raceInfo |= TR_PITSTART;
		if (curSeg2) {
		    curSeg2->raceInfo |= TR_PITSTART;
		}
	    } else if (mSeg == pitEnd->next) {
		curSeg->raceInfo |= TR_PITEND;
		if (curSeg2) {
		    curSeg2->raceInfo |= TR_PITEND;
		}
	    }
	}
    }

    /* 
     * camera definitions
     */
    if (GfParmListSeekFirst(TrackHandle, TRK_SECT_CAM) == 0) {
	do {
	    curCam = (tRoadCam*)calloc(1, sizeof(tRoadCam));
	    if (!curCam) {
		GfFatal("ReadTrack3: Memory allocation error");
	    }
	    if (*camList == NULL) {
		*camList = curCam;
		curCam->next = curCam;
	    } else {
		curCam->next = (*camList)->next;
		(*camList)->next = curCam;
		*camList = curCam;
	    }
	    curCam->name = GfParmListGetCurEltName(TrackHandle, TRK_SECT_CAM);
	    segName = GfParmGetCurStr(TrackHandle, TRK_SECT_CAM, TRK_ATT_SEGMENT, NULL);
	    if (segName == 0) {
		GfFatal("Bad Track Definition: in Camera %s %s is missing\n", curCam->name, TRK_ATT_SEGMENT);
	    }
	    snprintf(path2, BUFSIZE, "%s/%s/%s", TRK_SECT_MAIN, TRK_LST_SEGMENTS, segName);
	    segId = (int)GfParmGetNum(TrackHandle, path2, TRK_ATT_ID, (char*)NULL, 0);
	    curSeg = theTrack->seg;
	    for(i=0; i<theTrack->nseg; i++)  {
		if (curSeg->id == segId) {
		    break;
		}
		curSeg = curSeg->next;
	    }

	    trkPos.seg = curSeg;
	    trkPos.toRight = GfParmGetCurNum(TrackHandle, TRK_SECT_CAM, TRK_ATT_TORIGHT, (char*)NULL, 0);
	    trkPos.toStart = GfParmGetCurNum(TrackHandle, TRK_SECT_CAM, TRK_ATT_TOSTART, (char*)NULL, 0);
	    TrackLocal2Global(&trkPos, &(curCam->pos.x), &(curCam->pos.y));
	    curCam->pos.z = TrackHeightL(&trkPos) + GfParmGetCurNum(TrackHandle, TRK_SECT_CAM, TRK_ATT_HEIGHT, (char*)NULL, 0);

	    segName = GfParmGetCurStr(TrackHandle, TRK_SECT_CAM, TRK_ATT_CAM_FOV, NULL);
	    if (segName == 0) {
		GfFatal("Bad Track Definition: in Camera %s %s is missing\n", curCam->name, TRK_ATT_CAM_FOV);
	    }
	    snprintf(path2, BUFSIZE, "%s/%s/%s", TRK_SECT_MAIN, TRK_LST_SEGMENTS, segName);
	    segId = (int)GfParmGetNum(TrackHandle, path2, TRK_ATT_ID, (char*)NULL, 0);
	    curSeg = theTrack->seg;
	    for(i=0; i<theTrack->nseg; i++)  {
		if (curSeg->id == segId) {
		    break;
		}
		curSeg = curSeg->next;
	    }
	    segName = GfParmGetCurStr(TrackHandle, TRK_SECT_CAM, TRK_ATT_CAM_FOVE, NULL);
	    if (segName == 0) {
		GfFatal("Bad Track Definition: in Camera %s %s is missing\n", curCam->name, TRK_ATT_CAM_FOVE);
	    }
	    snprintf(path2, BUFSIZE, "%s/%s/%s", TRK_SECT_MAIN, TRK_LST_SEGMENTS, segName);
	    segId = (int)GfParmGetNum(TrackHandle, path2, TRK_ATT_ID, (char*)NULL, 0);
	
	    do {
		curSeg->cam = curCam;
		curSeg = curSeg->next;
	    } while (curSeg->id != segId);
	} while (GfParmListSeekNext(TrackHandle, TRK_SECT_CAM) == 0);
    }

    /* Update the coord to be positives */
    theTrack->min.x = 0;
    theTrack->min.y = 0;
    theTrack->min.z = 0;
    theTrack->max.x = xmax - xmin;
    theTrack->max.y = ymax - ymin;
    theTrack->max.z = zmax - zmin;

    curSeg = theTrack->seg;
    for(i=0; i<theTrack->nseg; i++)  {         /* read the segment data: */
	if ((curSeg->lgfromstart + curSeg->length) > (theTrack->length - 50.0)) {
	    curSeg->raceInfo |= TR_LAST;
	} else if (curSeg->lgfromstart < 50.0) {
	    curSeg->raceInfo |= TR_START;
	} else {
	    curSeg->raceInfo |= TR_NORMAL;
	}
	normSeg(curSeg);
	if (curSeg->lside) {
	    normSeg(curSeg->lside);
	    if (curSeg->lside->lside) {
		normSeg(curSeg->lside->lside);
	    }
	}
	if (curSeg->rside) {
	    normSeg(curSeg->rside);
	    if (curSeg->rside->rside) {
		normSeg(curSeg->rside->rside);
	    }
	}
	curSeg = curSeg->next;
    }
    

    if (*camList != NULL) {
	curCam = *camList;
	do {
	    curCam = curCam->next;
	    curCam->pos.x -= xmin;
	    curCam->pos.y -= ymin;
	    curCam->pos.z -= zmin;
	} while (curCam != *camList);
    }
}


/** Update global min/max values for turns (discretized approximation).

    @param[in] curBorder Border segment
	@param[in] radius Radius to hand over, either curBorder->radiusl or curBorder->radiusr
	@param[in] sign Sign to consider turn type ([TR_LFT](@ref TR_LFT) or [TR_RGT](@ref TR_RGT))
	@param[in] z Z value
 */
static void updateMinMaxForTurn(const tTrackSeg* const curBorder, const tdble radius, const tdble sign, const tdble z)
{
	tdble al, alfl, x, y;
	int j;

	// to find the boundary (global min/max, approximation)
	al = curBorder->arc / 36.0 * sign;
	alfl = curBorder->angle[TR_CS];

	for (j = 0; j < 36; j++) {
		alfl += al;
		x = curBorder->center.x + radius * cos(alfl);   // location of end
		y = curBorder->center.y + radius * sin(alfl);
		TSTX(x);
		TSTY(y);
	}
	TSTZ(z);
}


/** Set up border segment angles and gradients.

    @param[in,out] curBorder Border segment to set up
	@param[in] startwidth Start width of the border
	@param[in] endwidth End width of the border
 */
static void initAnglesAndGradients(tTrackSeg* const curBorder, const tdble startwidth, const tdble endwidth) {
	curBorder->angle[TR_YR] = atan2(curBorder->vertex[TR_ER].z - curBorder->vertex[TR_SR].z,
		curBorder->arc * curBorder->radiusr);
	curBorder->angle[TR_YL] = atan2(curBorder->vertex[TR_EL].z - curBorder->vertex[TR_SL].z,
		curBorder->arc * curBorder->radiusl);

	curBorder->Kzl = tan(curBorder->angle[TR_YR]) * curBorder->radiusr;
	curBorder->Kzw = (curBorder->angle[TR_XE] - curBorder->angle[TR_XS]) / curBorder->arc;
	curBorder->Kyl = (endwidth - startwidth) / curBorder->arc;
}


/** Set up side and border track segments for turns.
 
	Border segments are the ones touching the main segment (e.g. curbs), side segments are the segments touching the barrier.
	
	@param[in] turntype Turn type, either [TR_LFT](@ref TR_LFT) or [TR_RGT](@ref TR_RGT)
	@param[in,out] curBorder Border segment to set up
	@param[in] curSeg Connecting inner segment (e.g. main track segment for borders)
	@param[in] side 0 for right side, 1 for left side ([TRK_SECT_RSIDE](@ref TRK_SECT_RSIDE), [TRK_SECT_LSIDE](@ref TRK_SECT_LSIDE))
	@param[in] bankingtype For [TRK_VAL_TANGENT](@ref TRK_VAL_TANGENT) 1, for [TRK_VAL_LEVEL](@ref TRK_VAL_LEVEL) 0
	@param[in] startwidth Start width of the border
	@param[in] endwidth End width of the border
	@param[in] maxwidth Maximum width of the border
 */
static void initSideForTurn(
	const int turntype,
	tTrackSeg* const curBorder,
	const tTrackSeg* const curSeg,
	const int side,
	const int bankingtype,
	const tdble startwidth,
	const tdble endwidth,
	const tdble maxwidth
)
{
	tdble sign, z;

	if (turntype == TR_LFT) {
		sign = 1.0f;
	} else {
		sign = -1.0f;
	}

	curBorder->center.x = curSeg->center.x;
	curBorder->center.y = curSeg->center.y;

	// Combining code for case 0 (TRK_SECT_RSIDE) and 1 (TRK_SECT_LSIDE)?
	// Tried it, but the setup code is then as long as the copy and less readable because of the additional 8 variables
	switch(side) {
		case 1:
			curBorder->radius = curSeg->radiusl - sign * startwidth / 2.0;
			curBorder->radiusr = curSeg->radiusl;
			curBorder->radiusl = curSeg->radiusl - sign * maxwidth;
			curBorder->arc = curSeg->arc;
			curBorder->length = curBorder->radius * curBorder->arc;

			curBorder->vertex[TR_SL].x = curBorder->vertex[TR_SR].x - sign * startwidth * cos(curBorder->angle[TR_CS]);
			curBorder->vertex[TR_SL].y = curBorder->vertex[TR_SR].y - sign * startwidth * sin(curBorder->angle[TR_CS]);
			curBorder->vertex[TR_SL].z = curBorder->vertex[TR_SR].z + (tdble)bankingtype * startwidth * tan(curSeg->angle[TR_XS]);
			curBorder->vertex[TR_EL].x = curBorder->vertex[TR_ER].x - sign * endwidth * cos(curBorder->angle[TR_CS] + sign * curBorder->arc);	    
			curBorder->vertex[TR_EL].y = curBorder->vertex[TR_ER].y - sign * endwidth * sin(curBorder->angle[TR_CS] + sign * curBorder->arc);
			z = curBorder->vertex[TR_EL].z = curBorder->vertex[TR_ER].z + (tdble)bankingtype * endwidth * tan(curSeg->angle[TR_XE]);

			initAnglesAndGradients(curBorder, startwidth, endwidth);
			updateMinMaxForTurn(curBorder, curBorder->radiusl, sign, z);
			break;

		case 0:
			curBorder->radius = curSeg->radiusr + sign * startwidth / 2.0;
			curBorder->radiusl = curSeg->radiusr;
			curBorder->radiusr = curSeg->radiusr + sign * maxwidth;
			curBorder->arc = curSeg->arc;
			curBorder->length = curBorder->radius * curBorder->arc;

			curBorder->vertex[TR_SR].x = curBorder->vertex[TR_SL].x + sign * startwidth * cos(curBorder->angle[TR_CS]);
			curBorder->vertex[TR_SR].y = curBorder->vertex[TR_SL].y + sign * startwidth * sin(curBorder->angle[TR_CS]);
			curBorder->vertex[TR_SR].z = curBorder->vertex[TR_SL].z - (tdble)bankingtype * startwidth * tan(curSeg->angle[TR_XS]);
			curBorder->vertex[TR_ER].x = curBorder->vertex[TR_EL].x + sign * endwidth * cos(curBorder->angle[TR_CS] + sign * curBorder->arc);	    
			curBorder->vertex[TR_ER].y = curBorder->vertex[TR_EL].y + sign * endwidth * sin(curBorder->angle[TR_CS] + sign * curBorder->arc);
			z = curBorder->vertex[TR_ER].z = curBorder->vertex[TR_EL].z - (tdble)bankingtype * endwidth * tan(curSeg->angle[TR_XE]);

			initAnglesAndGradients(curBorder, startwidth, endwidth);
			updateMinMaxForTurn(curBorder, curBorder->radiusr, sign, z);
			break;
	}
}


/** Set up side and border track segments for straight segments.
 
	Border segments are the ones touching the main segment (e.g. curbs), side segments are the segments touching the barrier.
	
	@param[in,out] curBorder Border segment to set up
	@param[in] curSeg Connecting inner segment (e.g. main track segment for borders)
	@param[in] side 0 for right side, 1 for left side ([TRK_SECT_RSIDE](@ref TRK_SECT_RSIDE), [TRK_SECT_LSIDE](@ref TRK_SECT_LSIDE))
	@param[in] bankingtype For [TRK_VAL_TANGENT](@ref TRK_VAL_TANGENT) 1, for [TRK_VAL_LEVEL](@ref TRK_VAL_LEVEL) 0
	@param[in] startwidth Start width of the border
	@param[in] endwidth End width of the border
 */
static void initSideForStraight(
	tTrackSeg* const curBorder,
	const tTrackSeg* const curSeg,
	const int side,
	const int bankingtype,
	const tdble startwidth,
	const tdble endwidth
)
{
	tdble z, x, y;
	x = y = z = 0.0f;

	switch(side) {
		case 1:
			curBorder->vertex[TR_SL].x = curBorder->vertex[TR_SR].x + startwidth * curSeg->rgtSideNormal.x;
			curBorder->vertex[TR_SL].y = curBorder->vertex[TR_SR].y + startwidth * curSeg->rgtSideNormal.y;
			curBorder->vertex[TR_SL].z = curBorder->vertex[TR_SR].z + (tdble) bankingtype * startwidth * tan(curSeg->angle[TR_XS]);
			x = curBorder->vertex[TR_EL].x = curBorder->vertex[TR_ER].x + endwidth * curSeg->rgtSideNormal.x;	    
			y = curBorder->vertex[TR_EL].y = curBorder->vertex[TR_ER].y + endwidth * curSeg->rgtSideNormal.y;
			z = curBorder->vertex[TR_EL].z = curBorder->vertex[TR_ER].z + (tdble) bankingtype * endwidth * tan(curSeg->angle[TR_XE]);
			break;
		case 0:
			curBorder->vertex[TR_SR].x = curBorder->vertex[TR_SL].x - startwidth * curSeg->rgtSideNormal.x;
			curBorder->vertex[TR_SR].y = curBorder->vertex[TR_SL].y - startwidth * curSeg->rgtSideNormal.y;
			curBorder->vertex[TR_SR].z = curBorder->vertex[TR_SL].z - (tdble) bankingtype * startwidth * tan(curSeg->angle[TR_XS]);
			x = curBorder->vertex[TR_ER].x = curBorder->vertex[TR_EL].x - endwidth * curSeg->rgtSideNormal.x;	    
			y = curBorder->vertex[TR_ER].y = curBorder->vertex[TR_EL].y - endwidth * curSeg->rgtSideNormal.y;
			z = curBorder->vertex[TR_ER].z = curBorder->vertex[TR_EL].z - (tdble) bankingtype * endwidth * tan(curSeg->angle[TR_XE]);
			break;
	}

	curBorder->angle[TR_YR] = atan2(curBorder->vertex[TR_ER].z - curBorder->vertex[TR_SR].z, curBorder->length);
	curBorder->angle[TR_YL] = atan2(curBorder->vertex[TR_EL].z - curBorder->vertex[TR_SL].z, curBorder->length);

	curBorder->Kzl = tan(curBorder->angle[TR_YR]);
	curBorder->Kzw = (curBorder->angle[TR_XE] - curBorder->angle[TR_XS]) / curBorder->length;
	curBorder->Kyl = (endwidth - startwidth) / curBorder->length;

	curBorder->rgtSideNormal.x = curSeg->rgtSideNormal.x;
	curBorder->rgtSideNormal.y = curSeg->rgtSideNormal.y;

	TSTX(x);
	TSTY(y);
	TSTZ(z);
}


/** Common side/border allocation and initialization for straights and turns.

	@param[in,out] curSeg Segment to attach the new side/border
	@param[in] side 0 for right side, 1 for left side ([TRK_SECT_RSIDE](@ref TRK_SECT_RSIDE), [TRK_SECT_LSIDE](@ref TRK_SECT_LSIDE))
	@param[in] lefttype For side: [TR_LSIDE](@ref TR_LSIDE), for border: [TR_LBORDER](@ref TR_LBORDER)
	@param[in] righttype For side: [TR_RSIDE](@ref TR_RSIDE), for border: [TR_RBORDER](@ref TR_RBORDER)
	@param[in] bankingtype For [TRK_VAL_TANGENT](@ref TRK_VAL_TANGENT) 1, for [TRK_VAL_LEVEL](@ref TRK_VAL_LEVEL) 0
	@param[in] startwidth Start width of the side/border
	@param[in] endwidth End width of the side/border
	@param[in] surface Surface for the side/border
	@param[in] borderheight Border height for borders, should usually be 0.0 for sides
	@param[in] borderstyle Type of segment, for sides always [TR_PLAN](@ref TR_PLAN), for borders either [TR_PLAN](@ref TR_PLAN),
	           [TR_CURB](@ref TR_CURB) or [TR_WALL](@ref TR_WALL)
	@return Pointer to the newly created side/border segment
 */
static tTrackSeg* commonSideInit(
	tTrackSeg* const curSeg,
	const int side,
	const int lefttype,
	const int righttype,
	const int bankingtype,
	const tdble startwidth,
	const tdble endwidth,
	tTrackSurface* surface,
	const tdble borderheight,
	const int borderstyle
)
{
	tTrackSeg* curBorder = (tTrackSeg*)calloc(1, sizeof(tTrackSeg));
	if (side == 1) {
		curSeg->lside = curBorder;
		curBorder->vertex[TR_SR] = curSeg->vertex[TR_SL];
		curBorder->vertex[TR_ER] = curSeg->vertex[TR_EL];
		curBorder->type2 = lefttype;
	} else {
		curSeg->rside = curBorder;
		curBorder->vertex[TR_SL] = curSeg->vertex[TR_SR];
		curBorder->vertex[TR_EL] = curSeg->vertex[TR_ER];
		curBorder->type2 = righttype;
	}

	curBorder->startWidth = startwidth;
	curBorder->endWidth = endwidth;
	curBorder->width = MIN(startwidth, endwidth);
	tdble maxWidth = MAX(startwidth, endwidth);

	curBorder->type = curSeg->type;
	curBorder->surface = surface;
	curBorder->height = borderheight;
	curBorder->style = borderstyle;
	curBorder->envIndex = envIndex;		// TODO: Global?
	curBorder->DoVfactor = DoVfactor;	// TODO: Global?
	curBorder->angle[TR_XS] = curSeg->angle[TR_XS] * (tdble) bankingtype;
	curBorder->angle[TR_XE] = curSeg->angle[TR_XE] * (tdble) bankingtype;
	curBorder->angle[TR_ZS] = curSeg->angle[TR_ZS];
	curBorder->angle[TR_ZE] = curSeg->angle[TR_ZE];
	curBorder->angle[TR_CS] = curSeg->angle[TR_CS];

	switch(curSeg->type) {
		case TR_STR:
			curBorder->length = curSeg->length;
			initSideForStraight(curBorder, curSeg, side, bankingtype, startwidth, endwidth);
			break;
		case TR_LFT:
			initSideForTurn(TR_LFT, curBorder, curSeg, side, bankingtype, startwidth, endwidth, maxWidth);
			break;					
		case TR_RGT:
			initSideForTurn(TR_RGT, curBorder, curSeg, side, bankingtype, startwidth, endwidth, maxWidth);
			break;
	}

	return curBorder;
}
