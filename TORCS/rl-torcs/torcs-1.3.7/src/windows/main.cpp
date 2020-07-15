/***************************************************************************

    file                 : main.cpp
    created              : Sat Sep  2 10:40:47 CEST 2000
    copyright            : (C) 2000-2014 by Patrice & Eric Espie, Bernhard Wymann
    email                : torcs@free.fr
    version              : $Id: main.cpp,v 1.9.2.3 2014/05/23 08:38:30 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef WIN32
#include <windows.h>
#include <stdlib.h>
#include <shlobj.h>
#endif
#include <GL/glut.h>
#include <tgfclient.h>
#include <client.h>
#include <portability.h>
#include <raceinit.h>

#include "windowsspec.h"


// replace '\' with '/'
static void convertDelimiter(char* buf, const int bufsize)
{
	int i;
	for (i = 0; i < bufsize && buf[i] != '\0'; i++) {
		if (buf[i] == '\\') {
			buf[i] = '/';
		}
	}
}


static void copyFileIfNotExists(const char* filepath)
{
	static const int BUFSIZE = 1024;
	char target[BUFSIZE];
	snprintf(target, BUFSIZE, "%s%s", GetLocalDir(), filepath);
	target[BUFSIZE-1] = '\0';

	DWORD attr = GetFileAttributes(target);
	DWORD error = GetLastError();
	if (
		attr == INVALID_FILE_ATTRIBUTES &&
		(error == ERROR_FILE_NOT_FOUND || error == ERROR_PATH_NOT_FOUND)
	) {
		if (GfCreateDirForFile(target) == GF_DIR_CREATED) {
			char source[BUFSIZE];
			snprintf(source, BUFSIZE, "%s%s", GetDataDir(), filepath);
			source[BUFSIZE-1] = '\0';
			if (SUCCEEDED(CopyFile(source, target, TRUE))) {
				GfOut("Copy success: %s\n", target);
			} else {
				GfError("Copy failed: %s\n", target);
			}
		} else {
			GfError("Target directory creation failed: %s\n", target);
		}
	}	
}


// Check/prepare expected files in LocalDir
static void prepareLocalDir()
{
	const char* file[] = {
		"drivers/human/car.xml",
		"drivers/human/human.xml",
		"drivers/human/preferences.xml",
		"config/graph.xml",
		"config/raceengine.xml",
		"config/screen.xml",
		"config/sound.xml",
		"config/style.xsl"
	};

	int i;
	for (i = 0; i < sizeof(file)/sizeof(file[0]); i++) {
		copyFileIfNotExists(file[i]);
	}

	// Copy race manager configurations
	static const int BUFSIZE = 1024;
	char path[BUFSIZE];

	const char* racemanpath = "config/raceman";
	tFList *racemanList = GfDirGetListFiltered(racemanpath, "xml");
	tFList* racemanCur = racemanList;

	if (racemanList) {
		do {
			snprintf(path, BUFSIZE, "%s/%s", racemanpath, racemanCur->name);
			path[BUFSIZE-1] = '\0';
			copyFileIfNotExists(path);
			racemanCur = racemanCur->next;
		} while (racemanCur != racemanList);		
	}

	GfDirFreeList(racemanList, NULL, true, true);
}


// Try to set the users local application data directory
static void setUserLocalDir(char* buf, const int bufsize)
{
	TCHAR szLocalAppDataPath[MAX_PATH];
	if (SUCCEEDED(SHGetFolderPath(NULL, CSIDL_LOCAL_APPDATA | CSIDL_FLAG_CREATE, NULL, 0, szLocalAppDataPath))) {
		convertDelimiter(szLocalAppDataPath, MAX_PATH);			
		snprintf(buf, bufsize, "%s/torcs/", szLocalAppDataPath);
		if (GfCreateDir(buf) == GF_DIR_CREATED) {
			SetLocalDir(buf);
			prepareLocalDir();
		}
	}
}


static void init_args(int argc, char **argv, const char** raceconfig)
{
	int i = 1;
	
	while (i < argc) {
		if ((strncmp(argv[i], "-s", 2) == 0) || (strncmp(argv[i], "/s", 2) == 0)) {
			i++;
			SetSingleTextureMode ();
		} else if ((strncmp(argv[i], "-r", 2) == 0) || (strncmp(argv[i], "/r", 2) == 0)) {
			i++;
			*raceconfig = "";
			if (i < argc) {
				convertDelimiter(argv[i], strlen(argv[i]));
				*raceconfig = argv[i];
				i++;
			}

			if ((strlen(*raceconfig) == 0) || (strstr(*raceconfig, ".xml") == 0)) {
				printf("Please specify a race configuration xml when using -r\n");
				exit(1);
			}
		} else {
			i++;		// Ignore unknown arguments
		}
	}

	static const int BUFSIZE = 1024;
	char buf[BUFSIZE];
	strncpy(buf, argv[0], BUFSIZE);
	buf[BUFSIZE-1] = '\0';	// Guarantee zero termination for next operation.
	char *end = strrchr(buf, '\\');

	// Did we find the last '\' and do we get a complete path?
	if (end != NULL && buf[1] == ':') {
		end++;
		*(end) = '\0';
		convertDelimiter(buf, BUFSIZE);

		// Set fallback for local directory
		SetLocalDir(buf);
		// Set library and data directory
		SetDataDir(buf);
		SetLibDir("");
		setUserLocalDir(buf, BUFSIZE);
	} else {
		if (_fullpath(buf, argv[0], BUFSIZE) != NULL &&
			(strcmp(argv[0], "wtorcs") == 0 ||
			 strcmp(argv[0], "wtorcs.exe") == 0)
		   )
		{
			end = strrchr(buf, '\\');
			end++;
			*(end) = '\0';
			convertDelimiter(buf, BUFSIZE);

			// Set fallback for local directory
			SetLocalDir(buf);
			// Set library and data directory
			SetDataDir(buf);
			SetLibDir("");
			setUserLocalDir(buf, BUFSIZE);
		} else {
			printf("Run wtorcs.exe either from the GUI or from the directory which contains wtorcs.exe\n");
			exit(1);
		}
	}
}

/*
 * Function
 *	main
 *
 * Description
 *	Win32 entry point of TORCS
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
int
main(int argc, char *argv[])
{
	const char* raceconfig = "";

	WindowsSpecInit();			/* init specific windows functions */
	init_args(argc, argv, &raceconfig);

	if (strlen(raceconfig) == 0) {
		GfScrInit(argc, argv);	/* init screen */
		TorcsEntry();			/* launch TORCS */
		glutMainLoop();			/* event loop of glut */
	} else {
		// Run race from console, no Window, no OpenGL/OpenAL etc.
		// Thought for blind scripted AI training
		ReRunRaceOnConsole(raceconfig);
	}

	return 0;			/* just for the compiler, never reached */
}

