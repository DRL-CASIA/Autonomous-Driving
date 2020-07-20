/***************************************************************************

    file                 : torcsdoc.h
    created              : Sat Jul 27 14:24:31 CEST 2002
    copyright            : (C) 2001-2014 by Eric Espie, Bernhard Wymann
    email                : Eric.Espie@torcs.org
    version              : $Id: torcsdoc.h,v 1.3.2.12 2016/05/18 18:23:10 berniw Exp $

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
    @author Bernhard Wymann, Eric Espie
    @version $Id: torcsdoc.h,v 1.3.2.12 2016/05/18 18:23:10 berniw Exp $
*/

#ifndef _TORCSDOC_H_
#define _TORCSDOC_H_

/**
   @defgroup ctrl Control Device Management API
   This is the API to use control devices (Keyboard, mouse and joysticks).
*/

/**
   @defgroup dir Directory Management API
   This is used for directory manipulation.
*/

/**
   @defgroup module Dynamic Modules API
        
    This is the interface to load/unload the shared libraries (or DLLs).
    Two modes are allowed, the access by filename, of the access by the entire directory.
    When the directory mode is used, the filenames are not known in advance, this allows
    more flexibility at runtime.

    The generic information can be retrieved, without keeping the DLL loaded.
    The gfid parameter is used to differentiate the modules using different includes. This
    functionality is not used yet. This API is not used for shared libraries linked
    staticaly at compilation time.
*/

/**
   @defgroup gui GUI Management API
   This is an interface to manage menus.
   @note	The screen size is fiwed to 640x480 and the origin is in the lower-left corner.
*/

/**
   @defgroup hash Hash Tables API
   Hash tables API.
*/

/**
   @defgroup img Image Management API
   Load and store png images with easy interface.
*/


/**
    @defgroup params Parameter Handling API
    @brief With this API you can handle parameter sets in TORCS, this includes
    manipulation of data in memory, and writing into/reading from XML files.
    
    The parameters are structured in:
    <br><b>section</b> - containing a familly of parameters on the same topic.
    <br><b>list</b> (subsections) - containing <b>elements</b>
    of <b>numerical attributes</b> and <b>string attributes</b>.
    <br>
    <br>The API is not thread safe, because parameter sets and handles carry internal
    state regarding iteration and XML parsing. The parameter sets are reference counted,
	so usually multiple handles can refer to the same parameter set.
*/
/**
   @defgroup conf Parameter Handling API Setup
   Initialize and shutdown the Parameter Handling API
   @ingroup params
*/
/**
   @defgroup paramsfile Parameter File Handling API
   Create, read and write parameter files
   @ingroup params
*/
/**
   @defgroup paramsdata Parameter Data Handling API
   Handling of data in parameter sets 
   @ingroup params
*/
/**
   @defgroup paramslist Parameter List Handling API
   Handling of multiple sections or subsections and the elements in parameter sets
   @ingroup params
*/
/**
   @defgroup paramshelper Parameter Helper Functions
   Internal (not exported) helper functions, not part of the API
   @ingroup params
*/

/**
    @defgroup racemantools Race Managers API
    Common functions for race managers.
*/

/**
   @defgroup robottools Robottools API
   API for gathering track information and handling of car setups
*/

/**
   @defgroup screen Screen Management API
*/

/**
   @defgroup tailq	Tail Queue Management API
   This is the management of tail queues.
*/

/**
   @defgroup definitions Global Definitions
   This is the global definitions used in TORCS.
*/
/**
   @defgroup trackstruct Track Structure
   This is the tracks structure definition.
   <br><b>tTrack</b> is the main track structure.
   <br><b>trackSeg</b> is the segment structure.
   @ingroup definitions
*/
/**
   @defgroup carstruct Car Structure
   This is the cars structure definition.
   <br><b>CarElt</b> is the main car structure.
   @ingroup definitions
*/
/**
   @defgroup raceinfo Race Information Structure
   This is the race information structures definition.
   @ingroup definitions
*/

/**
   @defgroup modint	Modules Interfaces
   These are the interfaces for dynamic modules.
*/

/** 
    @defgroup OS OS Dependant Functions
    OS specific function table
*/

/**
   @defgroup trace		Trace Management
   Allow the trace in the file <tt>trace.txt</tt>
*/

/**
    @mainpage	Welcome to the TORCS Documentation

    @section archlist Architecture
    
    - [Architecture Overview](@ref architecture)
    
    @section doclist Other Documentation
    
    - [Robot Tutorial](http://www.berniw.org/tutorials/robot)
    - [Track Tutorial](http://www.berniw.org/aboutme/publications/build_your_trocs_track_in_20_minutes_v2.odt)
    - [Car Creation Tutorial](http://www.youtube.com/playlist?list=PLq-L0hhLuRI04BLdNW0QrSxhvuyPml8zp)
    - [Research Papers](http://scholar.google.com/scholar?q=torcs+car)

    The most recent links and news can be found on the [TORCS site](http://www.torcs.org).
    
    @section robotlist Robots API
    
    - [Robot Module Interface](@ref robotmodint)
    - [Robottools API](@ref robottools)
    - [Parameter Handling API](@ref params)
    - [Track Structure](@ref trackstruct)
    - [Car Structure](@ref carstruct)

    @section racemanlist Race Manager API

    - [Race Managers API](@ref racemantools)
    - [Parameter Handling API](@ref params)
    - [Tracks structure](@ref trackstruct)
    
    @section modlist Module Interfaces
    - [Graphic Module Interface](@ref graphicmodint)
    - [Robot Module Interface](@ref robotmodint)
    - [Simulation Module Interface](@ref simumodint)
    - [Track Loader Module Interface](@ref trackmodint)

*/

#endif /* _TORCSDOC_H_ */ 



