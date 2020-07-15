/***************************************************************************
                 params.cpp -- configuration parameters management
                             -------------------                                         
    created              : Fri Aug 13 22:27:57 CEST 1999
    copyright            : (C) 1999-2014 by Eric Espie, Bernhard Wymann
    email                : torcs@free.fr   
    version              : $Id: params.cpp,v 1.30.2.26 2014/05/23 08:38:31 berniw Exp $
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
    Parameter handling API
    @author Bernhard Wymann, Eric Espie
    @version	$Id: params.cpp,v 1.30.2.26 2014/05/23 08:38:31 berniw Exp $
*/

#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <math.h>

#include <xmlparse.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <tgf.h>
#include <assert.h>
#include <portability.h>


#define LINE_SZ			1024
#define PARAM_CREATE	0x01

#define P_NUM			0
#define P_STR			1

/** @brief Structure to hold linked list of within options */
struct within
{
	char *val;	/**< Value of within option */
	GF_TAILQ_ENTRY (struct within) linkWithin;	/**< Link to next entry */
};

GF_TAILQ_HEAD (withinHead, struct within);



/** Parameter header structure, a parameter can either carry a numeric or a string value,
 *  numeric value is constraint by min and max, string value by options in within
 *  @see GfParmUnit2SI
 */
struct param
{
	char				*name;		/**< Name of the parameter  */
	char				*fullName;	/**< Name of the parameter including the full section name ('/' separated) */
	char				*value;		/**< String value of the parameter */
	tdble				valnum;		/**< Numeric value of the parameter */
	int					type;		/**< Type, either @ref P_NUM or @ref P_STR */
	char				*unit;		/**< Unit, see @ref GfParmUnit2SI for supported units */
	tdble				min;		/**< Minimum for numeric value */
	tdble				max;		/**< Maximum for numeric value */
	struct withinHead	withinList;	/**< Linked list containing the options for legal string values */
	GF_TAILQ_ENTRY (struct param)	linkParam;	/**< Next parameter in the same section */
};

GF_TAILQ_HEAD (paramHead, struct param);
struct section;
GF_TAILQ_HEAD (sectionHead, struct section);


/** Section header structure */
struct section
{
	char *fullName;									/**< Name of the section including full path ('/' separated) */
	struct paramHead paramList;						/**< List of the parameters in this section */
	GF_TAILQ_ENTRY (struct section)	linkSection;	/**< Next section at the same level */
	struct sectionHead subSectionList;				/**< List of sub-sections (linked by linkSection) */
	struct section *curSubSection;					/**< Current subsection, for iterations, see @ref GfParmListSeekFirst and @ref GfParmListSeekNext */
	struct section *parent;							/**< Parent section */
};


#define PARM_MAGIC	0x20030815

/** Configuration header structure */
struct parmHeader
{
	char				*filename;	/**< Name of the configuration file */
	char				*name;		/**< Name of the data */
	char				*dtd;		/**< Optional DTD location */
	char				*header;	/**< Optional header (comment, xsl...) */
	int					refcount;	/**< Use counter (number of conf handle) */
	struct section		*rootSection;	/**< List of sections at the first level */
	void				*paramHash;	/**< Hash table for parameter access */
	void				*sectionHash;	/**< Hash table for section access */
};

#define PARM_HANDLE_FLAG_PRIVATE		0x01
#define PARM_HANDLE_FLAG_PARSE_ERROR	0x02


/** Ouput control structure used for serializing parameter set into XML*/
struct parmOutput
{
	int state;						/**< Internal state */
	struct section *curSection;		/**< Current section */
	struct param *curParam;			/**< Current parameter */
	char *filename;					/**< Name of the output file */
	int indent;						/**< Keep track of indentation */
};


/** Parameter set handle structure, multiple handles can reference the same parameter set */
struct parmHandle
{
	int magic;					/**< Magic number (to detect wrong type casts and such) */
	struct parmHeader *conf;	/**< Header of the parameter set */
	int flag;					/**< Flag (@ref PARM_HANDLE_FLAG_PARSE_ERROR, @ref PARM_HANDLE_FLAG_PRIVATE) */
	XML_Parser parser;			/**< Parser */
	struct section *curSection;	/**< Current section, for iterations, see @ref GfParmListSeekFirst and @ref GfParmListSeekNext */
	struct parmOutput outCtrl;	/**< Ouput control structure used for serializing parameter set into XML */
	GF_TAILQ_ENTRY (struct parmHandle)	linkHandle;	/**< Next configuration handle */
};


GF_TAILQ_HEAD (parmHead, struct parmHandle);

static struct parmHead	parmHandleList;

static char *getFullName(const char *sectionName, const char *paramName);
static struct param *getParamByName (struct parmHeader *conf, const char *sectionName, const char *paramName, int flag);
static void removeParamByName (struct parmHeader *conf, const char *sectionName, const char *paramName);
static void removeParam (struct parmHeader *conf, struct section *section, struct param *param);
static struct param *addParam (struct parmHeader *conf, struct section *section, const char *paramName, const char *value);
static void removeSection (struct parmHeader *conf, struct section *section);
static struct section *addSection (struct parmHeader *conf, const char *sectionName);
static void parmClean (struct parmHeader *conf);
static void parmReleaseHandle (struct parmHandle *parmHandle);
static void parmReleaseHeader (struct parmHeader *conf);
static struct section *getParent (struct parmHeader *conf, const char *sectionName);
static void cleanUnusedSection (struct parmHeader *conf, struct section *section);


/** @brief Parameter set library API initialization, set up parameter set handle cache. 
 *  @ingroup conf
 */
void GfParmInit (void)
{
	GF_TAILQ_INIT (&parmHandleList);
}


/** @brief Parameter set library API shutdown, removes parameter set handle cache. 
 *  @ingroup conf
 */
void GfParmShutdown (void)
{
	struct parmHandle	*parmHandle;

	while ((parmHandle = GF_TAILQ_FIRST (&parmHandleList)) != GF_TAILQ_END (&parmHandleList)) {
		parmReleaseHandle (parmHandle);
	}
}


/** @brief Helper function to get the full name of a parameter (full name: "sectionName/paramName").
 * 
 *  @ingroup paramshelper
 *  @param[in] sectionName name of the section containing the parameter
 *  @param[in] paramName name of the parameter
 *  @return string
 *  <br>NULL on error
 *  @note Heap memory is allocated for the return value, so the caller is responsible for releasing the
 *  memory of the returned string.
 */
static char *getFullName (const char *sectionName, const char *paramName)
{
	char *fullName;
	unsigned long len = strlen (sectionName) + strlen (paramName) + 2;
	
	fullName = (char *) malloc(strlen (sectionName) + strlen (paramName) + 2);
	if (!fullName) {
		GfError ("getFullName: malloc (%lu) failed", len);
		return NULL;
	}
	snprintf(fullName, len, "%s/%s", sectionName, paramName);
	
	return fullName;
}


/** @brief Helper function to get (or create) a parameter by name.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] conf parameter set header
 *  @param[in] sectionName name of the section containing the parameter
 *  @param[in] paramName name of the parameter
 *  @param[in] flag if in flag the @ref PARAM_CREATE bit is set the parameter gets created if it is not found
 *  @return param
 *  <br>NULL on error or  not found
 */
static struct param* getParamByName(struct parmHeader *conf, const char *sectionName, const char *paramName, int flag)
{
	char *fullName;
	struct param *param;
	struct section *section;

	fullName = getFullName (sectionName, paramName);
	if (!fullName) {
		GfError ("getParamByName: getFullName failed\n");
		return NULL;
	}

	param = (struct param *)GfHashGetStr (conf->paramHash, fullName);
	free (fullName);
	if (param || ((flag & PARAM_CREATE) == 0)) {
		return param;
	}

	/* Parameter not found CREATE it */
	section = (struct section *)GfHashGetStr (conf->sectionHash, sectionName);
	if (!section) {
		section = addSection (conf, sectionName);
		if (!section) {
			GfError ("getParamByName: addSection failed\n");
			return NULL;
		}
	}
	param = addParam (conf, section, paramName, "");

	return param;
}


/** @brief Helper function to remove a parameter with given name @e paramName.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] conf parameter set header
 *  @param[in] sectionName name of the section containing the parameter
 *  @param[in] paramName name of the parameter
 *  @note @ref cleanUnusedSection is called after removing the parameter.
 */
static void removeParamByName(struct parmHeader *conf, const char *sectionName, const char *paramName)
{
	char *fullName;
	struct param *param;
	struct section *section;
	
	section = (struct section *)GfHashGetStr (conf->sectionHash, sectionName);
	if (!section) {
		return;
	}
	
	fullName = getFullName (sectionName, paramName);
	if (!fullName) {
		GfError("removeParamByName: getFullName failed\n");
		return;
	}
	
	param = (struct param *)GfHashGetStr (conf->paramHash, fullName);
	freez (fullName);
	if (param) {
		removeParam(conf, section, param);
	}
	
	cleanUnusedSection(conf, section);
}


/** @brief Helper function to clean up unused (empty) sections starting with given @e section.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] conf parameter set header
 *  @param[in] section section to start up cleaning
 *  @note A section is unused if it does not contain subsections or elements, or if the fullName
 *  property is empty. If the given section is in use nothing is changed. If the section is
 *  unused it gets cleaned up and the process continues with the parent section (could now be
 *  empty as well).
 */
static void cleanUnusedSection(struct parmHeader *conf, struct section *section)
{
	struct section *parent;

	if (
		!section->fullName ||
		(!GF_TAILQ_FIRST (&(section->paramList)) && !GF_TAILQ_FIRST (&(section->subSectionList)))
	) {
		parent = section->parent;
		removeSection (conf, section);
		if (parent) {
			/* check if the parent is unused */
			cleanUnusedSection (conf, parent);
		}
	}
}


/** @brief Helper function to remove given parameter.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] conf parameter set header
 *  @param[in,out] section section to remove parameter from
 *  @param[in] param parameter to remove
 */
static void removeParam(struct parmHeader *conf, struct section *section, struct param *param)
{
	GfHashRemStr (conf->paramHash, param->fullName);
	GF_TAILQ_REMOVE (&(section->paramList), param, linkParam);

	struct within *within;
	while ((within = GF_TAILQ_FIRST (&param->withinList)) != GF_TAILQ_END (&param->withinList)) {
		GF_TAILQ_REMOVE (&param->withinList, within, linkWithin);
		freez(within->val);
		free(within);
	}

	freez (param->name);
	freez (param->fullName);
	freez (param->value);
	freez (param->unit);
	freez (param);
}


/** @brief Helper function to add parameter, does not check for duplicated name.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] conf parameter set header
 *  @param[in,out] section section to add parameter to
 *  @param[in] paramName parameter name
 *  @param[in] value value of parameter
 *  @return param
 *  <br>NULL on error
 */
static struct param *addParam(struct parmHeader *conf, struct section *section, const char *paramName, const char *value)
{
	char *fullName;
	struct param *param = NULL;
	char *tmpVal = NULL;
	const unsigned long len = sizeof (struct param);
	
	tmpVal = strdup (value);
	if (!tmpVal) {
		GfError ("addParam: strdup (%s) failed\n", value);
		goto bailout;
	}
	
	param = (struct param *) calloc (1, len);
	if (!param) {
		GfError ("addParam: calloc (1, %lu) failed\n", len);
		goto bailout;
	}

	param->name = strdup (paramName);
	if (!param->name) {
		GfError ("addParam: strdup (%s) failed\n", paramName);
		goto bailout;
	}
	
	fullName = getFullName (section->fullName, paramName);
	if (!fullName) {
		GfError ("addParam: getFullName failed\n");
		goto bailout;
	}

	param->fullName = fullName;
	if (GfHashAddStr (conf->paramHash, param->fullName, param)) {
		goto bailout;
	}
	
	GF_TAILQ_INIT (&(param->withinList));
	
	/* Attach to section */
	GF_TAILQ_INSERT_TAIL (&(section->paramList), param, linkParam);
	
	freez (param->value);
	param->value = tmpVal;
	
	return param;
	
 bailout:
	if (param) {
		freez (param->name);
		freez (param->fullName);
		freez (param->value);
		free  (param);
	}
	freez (tmpVal);
	
	return NULL;
}


/** @brief Helper function to remove a section and its contents (subsections, elements).
 * 
 *  @ingroup paramshelper
 *  @param[in,out] conf parameter set header
 *  @param[in] section section to remove
 */
static void removeSection(struct parmHeader *conf, struct section *section)
{
	struct param *param;
	struct section *subSection;

	while ((subSection = GF_TAILQ_FIRST (&(section->subSectionList))) != NULL) {
		removeSection (conf, subSection);
	}

	if (section->fullName) {
		/* not the root section */
		GfHashRemStr (conf->sectionHash, section->fullName);
		GF_TAILQ_REMOVE (&(section->parent->subSectionList), section, linkSection);
		while ((param = GF_TAILQ_FIRST (&(section->paramList))) != GF_TAILQ_END (&(section->paramList))) {
			removeParam (conf, section, param);
		}
		freez (section->fullName);
	}
	freez (section);
}


/** @brief Helper function to get (or create if not found) parent section of section given in @e sectionName.
 * 
 *  @ingroup paramshelper
 *  @param[in] conf parameter set header
 *  @param[in] sectionName name of the section
 *  @return section
 *  <br>NULL on error
 */
static struct section *getParent(struct parmHeader *conf, const char *sectionName)
{
	struct section *section;
	char *tmpName;
	char *s;

	tmpName = strdup (sectionName);
	if (!tmpName) {
		GfError ("getParent: strdup (\"%s\") failed\n", sectionName);
		return NULL;
	}

	s = strrchr (tmpName, '/');
	if (s) {
		*s = '\0';
		section = (struct section *)GfHashGetStr (conf->sectionHash, tmpName);
		if (section) {
			goto end;
		}
		section = addSection (conf, tmpName);
		goto end;
	} else {
		section = conf->rootSection;
		goto end;
	}

end:
	free (tmpName);
	return section;
}


/** @brief Helper function to add a section to a parameter set.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] conf parameter set header
 *  @param[in] sectionName section name
 *  @return section on success
 *  <br>NULL on error
 */
static struct section *addSection(struct parmHeader *conf, const char *sectionName)
{
	struct section *section;
	struct section *parent;
	const unsigned long len = sizeof (struct section);

	if (GfHashGetStr (conf->sectionHash, sectionName)) {
		GfError ("addSection: duplicate section [%s]\n", sectionName);
		return NULL;
	}

	parent = getParent(conf, sectionName);
	if (!parent) {
		GfError ("addSection: Problem with getParent for section [%s]\n", sectionName);
		return NULL;
	}

	section = (struct section *) calloc (1, len);
	if (!section) {
		GfError ("addSection: calloc (1, %lu) failed\n", len);
		return NULL;
	}

	section->fullName = strdup(sectionName);
	if (!section->fullName) {
		GfError ("addSection: strdup (%s) failed\n", sectionName);
		goto bailout;
	}

	if (GfHashAddStr (conf->sectionHash, sectionName, section)) {
		GfError ("addSection: GfHashAddStr failed\n");
		goto bailout;
	}

	/* no more bailout call */
	section->parent = parent;
	GF_TAILQ_INIT (&(section->paramList));
	GF_TAILQ_INIT (&(section->subSectionList));
	GF_TAILQ_INSERT_TAIL (&(parent->subSectionList), section, linkSection);

	return section;

bailout:
	freez (section->fullName);
	freez (section);
	return NULL;
}


/** @brief Helper function for looking up parameter sets in the cache.
 * 
 *  @ingroup paramshelper
 *  @param file name of the file to look up
 *  @param mode opening mode is a mask of:
 *  <br>#GFPARM_RMODE_STD if the parameter set is already loaded and not private return
 *  a handle pointing to the existing parameter set (default)
 *  <br>#GFPARM_RMODE_REREAD re-read the parameters file
 *  <br>#GFPARM_RMODE_CREAT if the parameters file does not exist return a handle
 *  pointing to an empty parameter set (does not create a file on disk,
 *  this is done using @ref GfParmWriteFile).
 *  <br>#GFPARM_RMODE_PRIVATE mark handle as private
 *  @return	handle to parameter set
 *  <br>0 if not in cache
 *  @see GfParmReadFile
 */
 static struct parmHeader *getSharedHeader(const char *file, int mode)
{
	struct parmHeader *conf = NULL;
	struct parmHandle *parmHandle;

	/* Search for existing conf */
	if ((mode & GFPARM_RMODE_PRIVATE) == 0) {
		for (
			parmHandle = GF_TAILQ_FIRST (&parmHandleList);
			parmHandle != GF_TAILQ_END (&parmHandleList);
			parmHandle = GF_TAILQ_NEXT (parmHandle, linkHandle)
		) {
			if ((parmHandle->flag & PARM_HANDLE_FLAG_PRIVATE) == 0) {
				conf = parmHandle->conf;
				if (!strcmp(conf->filename, file)) {
					if (mode & GFPARM_RMODE_REREAD) {
						parmClean (conf);
					}
					conf->refcount++;
					return conf;
				}
			}
		}
	}

	return NULL;
}


/** @brief Helper function to create header for parameter set handle.
 * 
 *  @ingroup paramshelper
 *  @param[in] file filename
 *  @return parmHeader in case of success
 *  <br>NULL on error
 */
static struct parmHeader* createParmHeader (const char *file)
{
	struct parmHeader	*conf = NULL;
	const unsigned long parmheadersize = sizeof (struct parmHeader);
	const unsigned long sectionsize = sizeof (struct section);

	conf = (struct parmHeader *) calloc (1, parmheadersize);
	if (!conf) {
		GfError ("gfParmReadFile: calloc (1, %lu) failed\n", parmheadersize);
		return NULL;
	}

	conf->refcount = 1;

	conf->rootSection = (struct section *) calloc (1, sectionsize);
	if (!conf->rootSection) {
		GfError ("gfParmReadFile: calloc (1, %lu) failed\n", sectionsize);
		goto bailout;
	}
	
	GF_TAILQ_INIT (&(conf->rootSection->paramList));
	GF_TAILQ_INIT (&(conf->rootSection->subSectionList));

	conf->paramHash = GfHashCreate (GF_HASH_TYPE_STR);
	if (!conf->paramHash) {
		GfError ("gfParmReadFile: GfHashCreate (paramHash) failed\n");
		goto bailout;
	}

	conf->sectionHash = GfHashCreate (GF_HASH_TYPE_STR);
	if (!conf->sectionHash) {
		GfError ("gfParmReadFile: GfHashCreate (sectionHash) failed\n");
		goto bailout;
	}

	conf->filename = strdup (file);
	if (!conf->filename) {
		GfError ("gfParmReadFile: strdup (%s) failed\n", file);
		goto bailout;
	}

	return conf;

 bailout:
	freez (conf->rootSection);
	if (conf->paramHash) {
		GfHashRelease (conf->paramHash, NULL);
	}
	
	if (conf->sectionHash) {
		GfHashRelease (conf->sectionHash, NULL);
	}
	
	freez (conf->filename);
	freez (conf);

	return NULL;
}


/** @brief Helper function to add "within" options to parameter @e curParam.
 * 
 *  @ingroup paramshelper
 *  @param[in] curParam parameter to add "within" option
 *  @param[in] s1 option string
 */
static void addWithin (struct param *curParam, char *s1)
{
	struct within *curWithin;

	if (!s1 || ! strlen (s1)) {
		return;
	}

	curWithin = (struct within *) calloc (1, sizeof (struct within));
	curWithin->val = strdup (s1);
	GF_TAILQ_INSERT_TAIL (&(curParam->withinList), curWithin, linkWithin);
}


static int myStrcmp(const void *s1, const void * s2)
{
    return strcmp((const char *)s1, (const char *)s2);
}


/** @brief Helper function to parse number.
 * 
 *  @ingroup paramshelper
 *  @param[in] str number as string
 *  @return number
 */
static tdble getValNumFromStr (const char *str)
{
	tdble val;

	if (!str || !strlen (str)) {
		return 0.0;
	}

	if (strncmp (str, "0x", 2) == 0) {
		return (tdble)strtol(str, NULL, 0);
	}

	sscanf (str, "%g", &val);
	return val;
}


/** @brief Helper function to process opening XML elements.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] userData handle to parameter set to read data into
 *  @param[in] name name of the current XML element
 *  @param[in] atts attributes of the XML element 
 */
static void xmlStartElement (void *userData , const char *name, const char **atts)
{
	struct parmHandle *parmHandle = (struct parmHandle *)userData;
	struct parmHeader *conf = parmHandle->conf;
	struct param *curParam;

	int	nAtts;
	int	len;
	const char **p;
	const char *s1, *s2;
	char *fullName;
	const char *shortName;
	const char *val;
	const char *min;
	const char *max;
	const char *unit;
	char *within;
	char *sa, *sb;

	if (parmHandle->flag & PARM_HANDLE_FLAG_PARSE_ERROR) {
		// parse error occured, ignore.
		return;
	}

	p = atts;
	while (*p) {
		++p;
	}

	nAtts = (p - atts) >> 1;
	if (nAtts > 1) {
		qsort ((void *)atts, nAtts, sizeof(char *) * 2, myStrcmp);
	}

	if (!strcmp(name, "params")) {

		parmHandle->curSection = conf->rootSection;
		parmHandle->curSection->fullName = strdup ("");

		if (!parmHandle->curSection->fullName) {
			GfError ("xmlStartElement: strdup (\"\") failed\n");
			goto bailout;
		}

		while (*atts) {
			s1 = *atts++;
			s2 = *atts++;
			if (!strcmp(s1, "name")) {
				FREEZ (conf->name);
				conf->name = strdup(s2);
				if (!conf->name) {
					GfError ("xmlStartElement: strdup (\"%s\") failed\n", s2);
					goto bailout;
				}
				break;
			}
		}

		if (!conf->name) {
			GfOut ("xmlStartElement: Syntax error, missing \"name\" field in params definition\n");
			goto bailout;
		}

    } else if (!strcmp(name, "section")) {

		if (!parmHandle->curSection) {
			GfError ("xmlStartElement: Syntax error, missing \"params\" tag\n");
			goto bailout;
		}

		shortName = NULL;

		while (*atts) {
			s1 = *atts++;
			s2 = *atts++;
			if (!strcmp(s1, "name")) {
				shortName = s2;
				break;
			}
		}

		if (!shortName) {
			GfError ("xmlStartElement: Syntax error, missing \"name\" field in section definition\n");
			goto bailout;
		}

		if (strlen(parmHandle->curSection->fullName)) {
			len = strlen (shortName) + strlen (parmHandle->curSection->fullName) + 2;
			fullName = (char *) malloc (len);
			if (!fullName) {
				GfError ("xmlStartElement: malloc (%d) failed\n", len);
				goto bailout;
			}
		    snprintf (fullName, len, "%s/%s", parmHandle->curSection->fullName, shortName);
		} else {
			fullName = strdup (shortName);
		}

		parmHandle->curSection = addSection(conf, fullName);
		free(fullName);

		if (!parmHandle->curSection) {
			GfError ("xmlStartElement: addSection failed\n");
			goto bailout;
		}

	} else if (!strcmp(name, "attnum")) {

		if ((!parmHandle->curSection) || (!strlen (parmHandle->curSection->fullName))) {
	    	GfError ("xmlStartElement: Syntax error, missing \"section\" tag\n");
			goto bailout;
		}

		shortName = NULL;
		val = NULL;
		min = max = unit = NULL;

		while (*atts) {
			s1 = *atts++;
			s2 = *atts++;
			if (!strcmp(s1, "name")) {
				shortName = s2;
			} else if (!strcmp(s1, "val")) {
				val = s2;
			} else if (!strcmp(s1, "min")) {
				min = s2;
			} else if (!strcmp(s1, "max")) {
				max = s2;
			} else if (!strcmp(s1, "unit")) {
				unit = s2;
			}
		}

		if (!shortName) {
			GfError ("xmlStartElement: Syntax error, missing \"name\" field in %s definition\n", name);
			goto bailout;
		}

		if (!val) {
			GfError ("xmlStartElement: Syntax error, missing \"val\" field in %s definition\n", name);
			goto bailout;
		}

		if (!min) {
			min = val;
		}

		if (!max) {
			max = val;
		}

		curParam = addParam (conf, parmHandle->curSection, shortName, val);
		if (!curParam) {
			GfError ("xmlStartElement: addParam failed\n");
			goto bailout;
		}

		curParam->type = P_NUM;
		curParam->valnum = getValNumFromStr (val);
		curParam->min    = getValNumFromStr (min);
		curParam->max    = getValNumFromStr (max);

		if (curParam->min > curParam->valnum) {
			curParam->min = curParam->valnum;
		}

		if (curParam->max < curParam->valnum) {
			curParam->max = curParam->valnum;
		}

		if (unit) {
			curParam->unit = strdup (unit);
			curParam->valnum = GfParmUnit2SI ((char*)unit, curParam->valnum);
			curParam->min = GfParmUnit2SI ((char*)unit, curParam->min);
			curParam->max = GfParmUnit2SI ((char*)unit, curParam->max);
		}

    } else if (!strcmp(name, "attstr")) {

		if ((!parmHandle->curSection) || (!strlen (parmHandle->curSection->fullName))) {
			GfError ("xmlStartElement: Syntax error, missing \"section\" tag\n");
			goto bailout;
		}

		shortName = NULL;
		val = NULL;
		within = NULL;

		while (*atts) {
			s1 = *atts++;
			s2 = *atts++;
			if (!strcmp(s1, "name")) {
				shortName = s2;
			} else if (!strcmp(s1, "val")) {
				val = s2;
			} else if (!strcmp(s1, "in")) {
				within = (char *)s2;
			}
		}

		if (!shortName) {
			GfError ("xmlStartElement: Syntax error, missing \"name\" field in %s definition\n", name);
			goto bailout;
		}

		if (!val) {
			GfError ("xmlStartElement: Syntax error, missing \"val\" field in %s definition\n", name);
			goto bailout;
		}

		curParam = addParam (conf, parmHandle->curSection, shortName, val);
		if (!curParam) {
			GfError ("xmlStartElement: addParam failed\n");
			goto bailout;
		}

		curParam->type = P_STR;
		if (within) {
			sa = within;
			sb = strchr (sa, ',');
			while (sb) {
				*sb = 0;
				addWithin (curParam, sa);
				sa = sb + 1;
				sb = strchr (sa, ',');
			}
			addWithin (curParam, sa);
		}

    }

    return;

 bailout:
    parmHandle->flag |= PARM_HANDLE_FLAG_PARSE_ERROR;
    return;
}


/** @brief Helper function to process closing XML elements.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] userData handle to parameter set to read data into
 *  @param[in] name name of the current XML element 
 */
static void xmlEndElement (void *userData, const XML_Char *name)
{
	struct parmHandle *parmHandle = (struct parmHandle *)userData;

	if (parmHandle->flag & PARM_HANDLE_FLAG_PARSE_ERROR) {
		/* parse error occured, ignore */
		return;
	}

	if (!strcmp(name, "section")) {
		if ((!parmHandle->curSection) || (!parmHandle->curSection->parent)) {
			GfError ("xmlEndElement: Syntax error in \"%s\"\n", name);
			return;
		}
		parmHandle->curSection = parmHandle->curSection->parent;
	}
}


/** @brief Helper function to handle external XML entities (XML referencing over multiple files/URI's).
 * 
 *  @ingroup paramshelper
 * @param[in] mainparser parent XML parser
 * @param[in] openEntityNames space separated list of names of entities that are open for the parse of this entity
 * @param[in] base unused (base path for resolving system id)
 * @param[in] systemId path to external entity (SYSTEM in XML)
 * @param[in] publicId unused (public identifier of external entity, PUBLIC in XML)
 * @return 1 ok
 * <br>0 error
 */
static int xmlExternalEntityRefHandler(
	XML_Parser mainparser,
	const XML_Char *openEntityNames,
	const XML_Char *base,
	const XML_Char *systemId,
	const XML_Char *publicId)
{
	FILE *in;
	char buf[BUFSIZ];
	XML_Parser parser;
	int done;
	char fin[LINE_SZ];
	char *s;
	struct parmHandle *parmHandle;
	struct parmHeader *conf;

	parmHandle = (struct parmHandle *)XML_GetUserData (mainparser);
	conf = parmHandle->conf;

	parser = XML_ExternalEntityParserCreate (mainparser, openEntityNames, (const XML_Char *)NULL);

	if (systemId[0] == '/') {
		strncpy (fin, systemId, sizeof (fin));
		fin[LINE_SZ - 1] = 0;
	} else {
		/* relative path */
		strncpy (fin, conf->filename, sizeof (fin));
		fin[LINE_SZ - 1] = 0;
		s = strrchr (fin, '/');
		if (s) {
			s++;
		} else {
			s = fin;
		}
		strncpy (s, systemId, sizeof (fin) - (s - fin));
		fin[LINE_SZ - 1] = 0;
	}

	in = fopen (fin, "r");
	if (in == NULL) {
		perror (fin);
		GfError ("GfReadParmFile: file %s has pb\n", systemId);
		return 0;
	}

	XML_SetElementHandler (parser, xmlStartElement, xmlEndElement);
	do {
		size_t len = fread (buf, 1, sizeof(buf), in);
		done = len < sizeof (buf);
		if (!XML_Parse (parser, buf, len, done)) {
			GfError ("file: %s -> %s at line %d\n",
				systemId,
				XML_ErrorString(XML_GetErrorCode(parser)),
				XML_GetCurrentLineNumber(parser));
			fclose (in);
			return 0;
		}
	} while (!done);
	
	XML_ParserFree (parser);
	fclose(in);

	return 1; /* ok (0 for failure) */
}


/** @brief Helper function to parse one line of XML.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] parmHandle parameter set handle
 *  @param[in] buf line to parse
 *  @param[in] len buffer size
 *  @param[in] done this was the last slice, no more input available
 *  @return 0 ok
 *  <br>1 error
 */
static int parseXml(struct parmHandle *parmHandle, char *buf, int len, int done)
{
	if (!XML_Parse(parmHandle->parser, buf, len, done)) {
		GfError ("parseXml: %s at line %d\n",
			(char*)XML_ErrorString (XML_GetErrorCode (parmHandle->parser)),
			XML_GetCurrentLineNumber (parmHandle->parser));
		return 1;
	}

	if (done) {
		XML_ParserFree(parmHandle->parser);
		parmHandle->parser = 0;
	}

	return 0;
}


/** @brief Helper function to set up XML parser in @e parmHandle.
 * 
 *  @ingroup paramshelper
 *  @param[in] parmHandle parameter set handle
 */
static int parserXmlInit (struct parmHandle *parmHandle)
{
    parmHandle->parser = XML_ParserCreate((XML_Char*)NULL);
    XML_SetElementHandler(parmHandle->parser, xmlStartElement, xmlEndElement);
    XML_SetExternalEntityRefHandler(parmHandle->parser, xmlExternalEntityRefHandler);
    XML_SetUserData(parmHandle->parser, parmHandle);

    return 0;
}


/** @brief Read parameter set from memory buffer and return handle to parameter set.
 * 
 *  @ingroup paramsfile
 *  @param[in] buffer buffer to read the configuration from 
 *  @return	handle to the parameter set
 *  <br>0 if Error
 *  @see GfParmWriteBuf
 */
void* GfParmReadBuf (char *buffer)
{
	struct parmHeader *conf;
	struct parmHandle *parmHandle = NULL;
	const unsigned long parmhandlesize = sizeof (struct parmHandle);

	/* Conf Header creation */
	conf = createParmHeader ("");
	if (!conf) {
		GfError ("gfParmReadBuf: conf header creation failed\n");
		goto bailout;
	}

	/* Handle creation */
	parmHandle = (struct parmHandle *) calloc (1, parmhandlesize);
	if (!parmHandle) {
		GfError ("gfParmReadBuf: calloc (1, %lu) failed\n", parmhandlesize);
		goto bailout;
	}

	parmHandle->magic = PARM_MAGIC;
	parmHandle->conf = conf;
	parmHandle->flag = PARM_HANDLE_FLAG_PRIVATE;

	/* Parsers Initialization */
	if (parserXmlInit (parmHandle)) {
		GfError ("gfParmReadBuf: parserInit failed\n");
		goto bailout;
	}

	/* Parameters reading in buffer */
	if (parseXml (parmHandle, buffer, strlen (buffer), 1)) {
		GfError ("gfParmReadBuf: Parse failed for buffer\n");
		goto bailout;
	}

	GF_TAILQ_INSERT_HEAD (&parmHandleList, parmHandle, linkHandle);

	return parmHandle;

 bailout:
	freez (parmHandle);
	if (conf) {
		parmReleaseHeader (conf);
	}

	return NULL;
}


/** @brief Read parameter set from file and return handle to parameter set.
 * 
 *  @ingroup paramsfile
 *  @param[in] file name of the file to read
 *  @param mode opening mode is a mask of:
 *  <br>#GFPARM_RMODE_STD if the parameter set is already loaded and not private return
 *  a handle pointing to the existing parameter set (default)
 *  <br>#GFPARM_RMODE_REREAD re-read the parameters file
 *  <br>#GFPARM_RMODE_CREAT if the parameters file does not exist return a handle
 *  pointing to an empty parameter set (does not create a file on disk,
 *  this is done using @ref GfParmWriteFile).
 *  <br>#GFPARM_RMODE_PRIVATE mark handle as private
 *  @return	handle to parameter set
 *  <br>0 if error
 *  @see GfParmWriteFile
 */
void* GfParmReadFile(const char *file, int mode)
{
	FILE *in = NULL;
	struct parmHeader *conf;
	struct parmHandle *parmHandle = NULL;
	char buf[LINE_SZ];
	int len;
	int done;
	const unsigned long parmHandleSize = sizeof (struct parmHandle);

	/* search for an already openned header & clean the conf if necessary */
	conf = getSharedHeader (file, mode);

	/* Conf Header creation */
	if (conf == NULL) {
		conf = createParmHeader (file);
		if (!conf) {
			GfError ("gfParmReadFile: conf header creation failed\n");
			goto bailout;
		}
		mode |= GFPARM_RMODE_REREAD;
	}

	/* Handle creation */
	parmHandle = (struct parmHandle *) calloc (1, parmHandleSize);
	if (!parmHandle) {
		GfError ("gfParmReadFile: calloc (1, %lu) failed\n", parmHandleSize);
		goto bailout;
	}

	parmHandle->magic = PARM_MAGIC;
	parmHandle->conf = conf;
	if (mode & GFPARM_RMODE_PRIVATE) {
		parmHandle->flag = PARM_HANDLE_FLAG_PRIVATE;
	}

	/* File opening */
	if (mode & GFPARM_RMODE_REREAD) {
		in = fopen (file, "r");
		if (!in && ((mode & GFPARM_RMODE_CREAT) == 0)) {
			GfOut ("gfParmReadFile: fopen \"%s\" failed\n", file);
			goto bailout;
		}

		if (in) {
			/* Parsers Initialization */
			if (parserXmlInit (parmHandle)) {
				GfError ("gfParmReadBuf: parserInit failed for file \"%s\"\n", file);
				goto bailout;
			}
			/* Parameters reading */
			do {
				len = fread (buf, 1, sizeof(buf), in);
				done = len < (int)sizeof(buf);
				if (parseXml (parmHandle, buf, len, done)) {
					GfError ("gfParmReadFile: Parse failed in file \"%s\"\n", file);
					goto bailout;
				}
				if (parmHandle->flag & PARM_HANDLE_FLAG_PARSE_ERROR) {
					/* parse error occured, ignore */
					GfError ("gfParmReadFile: Parse failed in file \"%s\"\n", file);
					goto bailout;
				}
			} while (!done);

			fclose (in);
			in = NULL;
		}
	}

	GF_TAILQ_INSERT_HEAD (&parmHandleList, parmHandle, linkHandle);

	GfOut ("GfParmReadFile: Opening \"%s\" (%p)\n", file, parmHandle);

	return parmHandle;

	bailout:
		if (in) {
			fclose (in);
		}
		freez (parmHandle);
		if (conf) {
			parmReleaseHeader (conf);
		}

	return NULL;
}


/** @brief Helper function to convert the input line given in @e val into proper XML notation, the output goes into @e buf.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] buf buffer for the processed line
 *  @param[in] BUFSIZE buffer size
 *  @param[in] val input line
 *  @return pointer to given buffer @e buf
 */
static char* handleEntities(char *buf, const int BUFSIZE, const char* val)
{
	int i = 0;
	int len = strlen(val);
	const char *replacement;
	char *pos = buf;
	int rlen;
	
	for (i = 0; i < len; i++) {
		switch (val[i]) {
			case '<':
				replacement = "&lt;"; break;
			case '>':
				replacement = "&gt;"; break;
			case '&':
				replacement = "&amp;"; break;
			case '\'':
				replacement = "&apos;"; break;
			case '"':
				replacement = "&quot;"; break;	
			default:
				replacement = 0;		
		}
		
		if (replacement == 0) {
			replacement = &val[i];
			rlen = 1;
		} else {
			rlen = strlen(replacement);
		}
		
		if (pos-buf < BUFSIZE - rlen) {
			memcpy(pos, replacement, rlen*sizeof(char));
			pos += rlen;
		} else {
			GfError("handleEntities: buffer too small to convert %s", val);
			break;
		}
	}

	*pos = '\0';

	return buf;
}


/** @brief Helper function for indentation in the XML.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] buf buffer for the result
 *  @param[in] BUFSIZE buffer size
 *  @param[in] blanks number of blanks to write
 */
static void createIndent(char *buf, const int BUFSIZE, const int blanks)
{
	int pos = 0;
	while ((pos < BUFSIZE - 1) && (pos < blanks)) {
		*buf++ = ' ';
		pos++;
	}
	*buf = '\0';
}


/** @brief Helper function to support the serialization into the XML of the "within" attribute.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] buf buffer for the result
 *  @param[in] BUFSIZE buffer size
 *  @param[in] head head of the list with the within options
 */
static void createIn(char *buf, const int BUFSIZE, withinHead* head)
{
	const char* s = " in=\"";
	struct within* curWithin = GF_TAILQ_FIRST(head);
	int pos = 0;
	bool separator = false;
	*buf = '\0'; // Terminate for empty content
	
	while (curWithin != 0) {
		int len = strlen(s);
		if (pos < BUFSIZE - len - 1) {
			memcpy(buf, s, len*sizeof(char));
			buf += len;
			pos += len;
		} else {
			break;
		}
		
		if (separator) {
			curWithin = GF_TAILQ_NEXT(curWithin, linkWithin);
			if (curWithin != 0) {
				s = ",";
				separator = false;
			}
		} else {
			s = curWithin->val;
			separator = true;
		}
	}
	
	// Just terminate if we have written something
	if (pos > 0) {
		memcpy(buf, "\"", 2*sizeof(char));
	}
}


/** @brief Helper function to output one line of XML generated from the given parameter set.
 * 
 *  The parameter set handle @e parmHandle keeps track of the progress internally.
 * 
 *  @ingroup paramshelper
 *  @param[in,out] parmHandle parameter set handle
 *  @param[in,out] buffer buffer for the line
 *  @param[in] size buffer size
 *  @return	1 more lines available
 *  <br>0 done
 */
static int xmlGetOuputLine(struct parmHandle *parmHandle, char *buffer, int size)
{
	struct parmOutput *outCtrl = &(parmHandle->outCtrl);
	struct parmHeader *conf = parmHandle->conf;
	struct section *curSection;
	struct param *curParam;
	char *s;
	const int BUFSIZE = 1024;
	char buf[BUFSIZE];
	const int INDENTSIZE = 1024;
	char indent[INDENTSIZE];
	const int INSIZE = 1024;
	char in[INSIZE];
	const int NUMVALUE = 1024;
	char numvalue[NUMVALUE];

	while (1) {
		switch (outCtrl->state) {
		case 0:
			snprintf (buffer, size, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
			outCtrl->indent = 0;
			outCtrl->state = 1;
			return 1;

		case 1:
			if (conf->dtd == NULL) {
				conf->dtd = strdup("params.dtd");
			}
			if (conf->header == NULL) {
				conf->header = strdup("");
			}
			snprintf (buffer, size, "<!DOCTYPE params SYSTEM \"%s\">\n%s\n", conf->dtd, conf->header);
			outCtrl->indent = 0;
			outCtrl->state = 2;
			return 1;

		case 2:			/* Start Params */
			outCtrl->curSection = parmHandle->conf->rootSection;
			snprintf (buffer, size, "\n<params name=\"%s\">\n", parmHandle->conf->name);
			curSection = GF_TAILQ_FIRST (&(outCtrl->curSection->subSectionList));
			if (curSection) {
				outCtrl->curSection = curSection;
				outCtrl->indent += 2;
				outCtrl->state = 4;
			} else {     
				outCtrl->state = 3;
			}
			return 1;

		case 3:			/* End Params */
			snprintf (buffer, size, "</params>\n");
			outCtrl->state = 9;
			return 1;

		case 4:			/* Parse section attributes list */
			outCtrl->curParam = GF_TAILQ_FIRST (&(outCtrl->curSection->paramList));
			s = strrchr (outCtrl->curSection->fullName, '/');
			if (!s) {
				s = outCtrl->curSection->fullName;
			} else {
				s++;
			}
			
			createIndent(indent, INDENTSIZE, outCtrl->indent);
			handleEntities(buf, BUFSIZE, s);
			snprintf(buffer, size, "%s<section name=\"%s\">\n", indent, buf);
			
			outCtrl->indent += 2;
			outCtrl->state = 5;
			return 1;

		case 5:			/* Parse one attribute */
			if (!outCtrl->curParam) {
				outCtrl->state = 6;
				break;
			}

			curParam = outCtrl->curParam;
			if (curParam->type == P_STR) {
				createIndent(indent, INDENTSIZE, outCtrl->indent);
				createIn(in, INSIZE, &(curParam->withinList));
				handleEntities(buf, BUFSIZE, curParam->value);
				snprintf(buffer, size, "%s<attstr name=\"%s\"%s val=\"%s\"/>\n", indent, curParam->name, in, buf);
				
				outCtrl->curParam = GF_TAILQ_NEXT (curParam, linkParam);
				return 1;
			} else {
				if (curParam->unit) {
					if ((curParam->min != curParam->valnum) || (curParam->max != curParam->valnum)) {
						snprintf(numvalue, NUMVALUE, " min=\"%g\" max=\"%g\" unit=\"%s\" val=\"%g\"/>\n", 
							GfParmSI2Unit (curParam->unit, curParam->min),
							GfParmSI2Unit (curParam->unit, curParam->max),
							curParam->unit,
							GfParmSI2Unit (curParam->unit, curParam->valnum)
						);
					} else {
						snprintf(numvalue, NUMVALUE, " unit=\"%s\" val=\"%g\"/>\n", 
							curParam->unit,
							GfParmSI2Unit (curParam->unit, curParam->valnum)
						);
					}
				} else {
					if ((curParam->min != curParam->valnum) || (curParam->max != curParam->valnum)) {
						snprintf (numvalue, NUMVALUE, " min=\"%g\" max=\"%g\" val=\"%g\"/>\n", 
							curParam->min,
							curParam->max,
							curParam->valnum
						);
					} else {
						snprintf (numvalue, NUMVALUE, " val=\"%g\"/>\n", curParam->valnum);
					}
				}
				
				createIndent(indent, INDENTSIZE, outCtrl->indent);
				snprintf (buffer, size, "%s<attnum name=\"%s\"%s", indent, curParam->name, numvalue);

				outCtrl->curParam = GF_TAILQ_NEXT (curParam, linkParam);
				return 1;
			}

		case 6:			/* Parse sub-section list */
			curSection = GF_TAILQ_FIRST (&(outCtrl->curSection->subSectionList));
			if (curSection) {
				outCtrl->curSection = curSection;
				outCtrl->state = 4;
				break;
			}
			outCtrl->state = 7;
			break;

		case 7:			/* End Section */
			outCtrl->indent -= 2;
			createIndent(indent, INDENTSIZE, outCtrl->indent);
			snprintf (buffer, size, "%s</section>\n\n", indent);
			outCtrl->state = 8;
			return 1;

		case 8:			/* Parse next section at the same level */
			curSection = GF_TAILQ_NEXT (outCtrl->curSection, linkSection);
			if (curSection) {
				outCtrl->curSection = curSection;
				outCtrl->state = 4;
				break;
			}
			curSection = outCtrl->curSection->parent;
			outCtrl->indent -= 2;
			if (curSection->parent) {
				outCtrl->curSection = curSection;
				createIndent(indent, INDENTSIZE, outCtrl->indent);
				snprintf (buffer, size, "%s</section>\n\n", indent);
				return 1;
			}
			outCtrl->state = 3;
			break;
		case 9:
			return 0;
		}
	}
}


/** @brief Write a parameter set into a memory buffer.
 * 
 *  @ingroup paramsfile
 *  @param[in] handle parameter set handle
 *  @param[in,out] buf buffer to write the configuration to (must be big enough)
 *  @param[in] size buffer size
 *  @return	0 if ok
 *  <br>1 if error
 */
int GfParmWriteBuf(void *handle, char *buf, int size)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	char line[LINE_SZ];
	int len;
	int curSize;
	char *s;

	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("gfParmWriteBuf: bad handle (%p)\n", parmHandle);
		return 1;
	}

	parmHandle->outCtrl.state = 0;
	parmHandle->outCtrl.curSection = NULL;
	parmHandle->outCtrl.curParam = NULL;
	curSize = size;
	s = buf;

	while (curSize && xmlGetOuputLine (parmHandle, line, sizeof (line))) {
		len = strlen (line);
		if (len > curSize) {
			len = curSize;
		}
		memcpy (s, line, len);
		s += len;
		curSize -= len;
	}
	buf [size - 1] = 0;

	return 0;
}


/** @brief Set the dtd path and header.
 * 
 *  @ingroup paramsfile
 *  @param	parmHandle parameter set handle
 *  @param	dtd optional dtd path
 *  @param	header optional header
 */
void GfParmSetDTD(void *parmHandle, char *dtd, char*header)
{
	struct parmHandle *handle = (struct parmHandle *)parmHandle;
	struct parmHeader *conf = handle->conf;

	if (dtd) {
		FREEZ(conf->dtd);
		conf->dtd = strdup(dtd);
	}

	if (header) {
		FREEZ(conf->header);
		conf->header = strdup(header);
	}
}


/** @brief Write parameter set into file.
 * 
 *   @ingroup paramsfile
 *   @param[in] file if NULL the internally stored filename is used, if not NULL the given filename is used (it is not stored in the handle)
 *   @param[in,out] parmHandle parameter set handle
 *   @param[in] name if NULL the internally stored name is used, if not NULL the given name is used and stored in the handle
 *   @return 0 if ok
 *   <br>1 if Error
 */
int GfParmWriteFile(const char *file, void *parmHandle, const char *name)
{
	struct parmHandle *handle = (struct parmHandle *)parmHandle;
	struct parmHeader *conf = handle->conf;
	char line[LINE_SZ];
	FILE *fout;
	
	if (handle->magic != PARM_MAGIC) {
		GfFatal ("gfParmWriteFile: bad handle (%p)\n", parmHandle);
		return 1;
	}
	
	if (!file) {
		file = conf->filename;
		if (!file) {
			GfError ("gfParmWriteFile: bad file name\n");
			return 1;
		}
	}

	fout = fopen (file, "wb");
	if (!fout) {
		// GfError ("gfParmSetStr: fopen (%s, \"wb\") failed\n", file);
		return 1;
	}
	
	if (name) {
		FREEZ (conf->name);
		conf->name = strdup (name);
	}
	
	handle->outCtrl.state = 0;
	handle->outCtrl.curSection = NULL;
	handle->outCtrl.curParam = NULL;
	
	while (xmlGetOuputLine (handle, line, sizeof (line))) {
		fputs (line, fout);
	}
	
	GfOut ("GfParmWriteFile: %s file written\n", file);
	
	fclose (fout);
	
	return 0;
}


/** @brief Create directory for parameter set handle if it does not yet exist.
 * 
 *   @ingroup paramsfile
 *   @param[in] file if NULL the internally stored path is used, if not NULL the given path is used (it is not stored in the handle)
 *   @param[in,out] parmHandle parameter set handle
 *   @return 0 if ok
 *   <br>1 if Error
 */
int GfParmCreateDirectory(const char *file, void *parmHandle)
{
	struct parmHandle *handle = (struct parmHandle *)parmHandle;
	struct parmHeader *conf = handle->conf;
	
	if (handle->magic != PARM_MAGIC) {
		GfFatal ("GfParmCreateDirectory: bad handle (%p)\n", parmHandle);
		return 1;
	}
	
	if (!file) {
		file = conf->filename;
		if (!file) {
			GfError ("GfParmCreateDirectory: bad file name\n");
			return 1;
		}
	}

	if (GfCreateDirForFile(file) != GF_DIR_CREATED) {
		return 1;
	}

	return 0;
}


/** @brief Remove a parameter from a parameter set.
 * 
 *  @ingroup paramsdata
 *  @param parmHandle parameter set handle
 *  @param sectionName parameter section name
 *  @param paramName parameter name
 */
void GfParmRemove(void *parmHandle, char *sectionName, char *paramName)
{
	struct parmHandle *handle = (struct parmHandle *)parmHandle;
	struct parmHeader *conf;

	conf = handle->conf;

	if (handle->magic != PARM_MAGIC) {
		GfFatal ("gfParmRemove: bad handle (%p)\n", parmHandle);
		return;
	}

	removeParamByName(conf, sectionName, paramName);
}


/** @brief Helper function to release the parameter set content.
 * 
 *  @ingroup paramshelper
 *  @param[in] conf parameter set header
 */
static void parmClean(struct parmHeader *conf)
{
	struct section	*section;

	while ((section = GF_TAILQ_FIRST (&(conf->rootSection->subSectionList))) !=
		    GF_TAILQ_END (&(conf->rootSection->subSectionList)))
	{
		removeSection (conf, section);
	}
}


/** @brief Clean all the parameters of a parameter set.
 *
 *  Removes all contained parameters in the parameter set, the
 *  @e parmHande remains valid. 
 * 
 *  @ingroup paramsfile
 *  @param[in,out] parmHandle parameter set handle
 *  @return	0 if OK
 *  <br>-1 if Error
 */
void GfParmClean(void *parmHandle)
{
	struct parmHandle *handle = (struct parmHandle *)parmHandle;
	struct parmHeader *conf;

	conf = handle->conf;

	if (handle->magic != PARM_MAGIC) {
		GfFatal ("gfParmClean: bad handle (%p)\n", parmHandle);
		return;
	}

	parmClean (conf);
}


/** @brief Helper function to release the parameter set if the reference counter is 0.
 * 
 *  @ingroup paramshelper
 *  @param[in] conf parameter set header
 *  @see GfParmReleaseHandle
 *  @see GfParmWriteFile
 *  @see GfParmReadFile
 */
static void parmReleaseHeader(struct parmHeader *conf)
{
	conf->refcount--;
	if (conf->refcount > 0) {
		return;
	}

	GfOut ("parmReleaseHeader: refcount null free \"%s\"\n", conf->filename);

	parmClean (conf);

	freez (conf->filename);
	if (conf->paramHash) {
		GfHashRelease (conf->paramHash, NULL);
	}

	if (conf->sectionHash) {
		GfHashRelease (conf->sectionHash, NULL);
	}

	freez (conf->rootSection->fullName);
	freez (conf->rootSection);
	freez (conf->dtd);
	freez (conf->name);
	freez (conf->header);
	freez (conf);
}


/** @brief Helper function to release the handle and eventually the referenced parameter set (if the reference counter falls to 0).
 *
 *  @ingroup paramshelper
 *  @param[in] parmHandle parameter set handle
 *  @see GfParmReleaseHandle
 *  @see GfParmWriteFile
 *  @see GfParmReadFile
 */
static void parmReleaseHandle(struct parmHandle *parmHandle)
{
	struct parmHeader *conf = parmHandle->conf;

	GfOut ("parmReleaseHandle: release \"%s\" (%p)\n", conf->filename, parmHandle);

	GF_TAILQ_REMOVE (&parmHandleList, parmHandle, linkHandle);
	parmHandle->magic = 0;
	freez (parmHandle);

	parmReleaseHeader(conf);
}


/** @brief Release given parameter set handle @e parmHandle.
 * 
 *  Releases the parameter set handle and eventally the parameter set which it
 *  refers to. 
 * 
 *  The parameter sets are internally reused and reference counted, so if the
 *  parameter set has still a reference counter greater than 0, just the reference
 *  counter is decremented, if it reaches 0, the whole parameter set is deleted from
 *  memory.
 * 
 *  The parameter file is not written on release, write operations are done explicitely
 *  with @ref GfParmWriteFile.
 * 
 *  @ingroup paramsfile
 *  @param[in] parmHandle parameter set handle
 *  @see GfParmWriteFile
 *  @see GfParmReadFile
 */
void GfParmReleaseHandle(void *parmHandle)
{
	struct parmHandle *handle = (struct parmHandle *)parmHandle;

	if (handle->magic != PARM_MAGIC) {
		GfFatal ("gfParmReleaseHandle: bad handle (%p)\n", parmHandle);
		return;
	}

	parmReleaseHandle(handle);
}


/** @brief Support function to multiply or divide @e dest with unit conversion factor.
 * 
 *  This function is used by @ref GfParmUnit2SI and @ref GfParmSI2Unit. The given unit string
 *  gets split up and processed unit by unit with evalUnit.
 * 
 *  @ingroup paramshelper
 *  @param[in] unit unit name from dest
 *  @param[in,out] dest pointer to value to convert
 *  @param flg multiply (0) or divide (otherwise)
 *  @see GfParmUnit2SI
 *  @see GfParmSI2Unit
 */
static void evalUnit(char *unit, tdble *dest, int flg)
{
	tdble coeff = 1.0;

	if (strcmp(unit, "m") == 0) return;
	if (strcmp(unit, "kg") == 0) return;
	if (strcmp(unit, "s") == 0) return;
	if (strcmp(unit, "rad") == 0) return;
	if (strcmp(unit, "Pa") == 0) return;

	if ((strcmp(unit, "feet") == 0) || (strcmp(unit, "ft") == 0)) {
		coeff = 0.304801f; /* m */
	} else if (strcmp(unit, "deg") == 0) {
		coeff = (float) (M_PI/180.0); /* rad */
	} else if ((strcmp(unit, "h") == 0) || (strcmp(unit, "hour") == 0) || (strcmp(unit, "hours") == 0)) {
		coeff = 3600.0; /* s */
	} else if ((strcmp(unit, "day") == 0) || (strcmp(unit, "days") == 0)) {
		coeff = 24*3600.0; /* s */
	} else if (strcmp(unit, "km") == 0) {
		coeff = 1000.0; /* m */
	} else if (strcmp(unit, "mm") == 0) {
		coeff = 0.001f; /* m */
	} else if (strcmp(unit, "cm") == 0) {
		coeff = 0.01f; /* m */
	} else if ((strcmp(unit, "in") == 0) || (strcmp(unit, "inch") == 0) || (strcmp(unit, "inches") == 0)) {
		coeff = 0.0254f; /* m */
	} else if ((strcmp(unit, "lbs") == 0)  || (strcmp(unit, "lb") == 0)) {
		coeff = 0.45359237f; /* kg */
	} else if (strcmp(unit, "lbf") == 0) {
		coeff = 0.45359237f*G; /* N (kg*m/s^2) */
	} else if ((strcmp(unit, "slug") == 0) || (strcmp(unit, "slugs") == 0)) {
		coeff = 14.59484546f; /* kg */
	} else if (strcmp(unit, "kPa") == 0) {
		coeff = 1000.0; /* Pa */
	} else if (strcmp(unit, "MPa") == 0) {
		coeff = 1000000.0; /* Pa */
	} else if ((strcmp(unit, "PSI") == 0) || (strcmp(unit, "psi") == 0)){
		coeff = 6894.76f; /* Pa */
	} else if ((strcmp(unit, "rpm") == 0) || (strcmp(unit, "RPM") == 0)) {
		coeff = 0.104719755f; /* rad/s */
	} else if ((strcmp(unit, "percent") == 0) || (strcmp(unit, "%") == 0)) {
		coeff = 0.01f;
	} else if ((strcmp(unit, "mph") == 0) || (strcmp(unit, "MPH") == 0)) {
		coeff = 0.44704f; /* m/s */
	}

	if (flg) {
		*dest /= coeff;
	} else {
		*dest *= coeff;
	}

	return;
}


/** @brief Convert a value given in unit to SI.
 * 
 *   The units can be combined with "/" (divide), "." (multiply) and "2" (square), e.g. "lbf/in", "N.m", "kg.m/s2".  
 *   
 *   @ingroup paramsdata
 *   @param[in] unit unit name from val
 *   @param[in] val value in unit
 *   @return the value converted to SI
 *   @note	The supported units are:
 *   <br><ul>
 *   <li><b>feet</b> or <b>ft</b>  converted to <b>m</b></li>
 *   <li><b>inches</b>,<b>inch</b> or <b>in</b> converted to <b>m</b></li>
 *   <li><b>km</b> converted to <b>m</b></li>
 *   <li><b>cm</b> converted to <b>m</b></li>
 *   <li><b>mm</b> converted to <b>m</b></li>
 *   <li><b>lbs</b> converted to <b>kg</b></li>
 *   <li><b>slug</b> or <b>slugs</b> converted to <b>kg</b></li>
 *   <li><b>h</b>,<b>hour</b> or <b>hours</b> converted to <b>s</b></li>
 *   <li><b>day</b> or <b>days</b> converted to <b>s</b></li>
 *   <li><b>kPa</b> or <b>MPa</b> converted to <b>Pa</b></li>
 *   <li><b>PSI</b> or <b>psi</b> converted to <b>Pa</b></li>
 *   <li><b>deg</b> converted to <b>rad</b></li>
 *   <li><b>rpm</b> or <b>RPM</b> converted to <b>rad/s</b></li>
 *   <li><b>percent</b> or <b>%</b> divided by <b>100</b></li>
 *   <li><b>lbf</b> converted to <b>N</b></li>
 *   </ul>
 * 
 *   @see GfParmSI2Unit
 */
tdble GfParmUnit2SI(const char *unit, tdble val)
{
	char buf[256];
	int  idx;
	const char *s;
	int  inv;
	tdble dest = val;
	
	if ((unit == NULL) || (strlen(unit) == 0)) return dest;
	
	s = unit;
	buf[0] = 0;
	inv = 0;
	idx = 0;
	
	while (*s != 0) {
		switch (*s) {
			case '.':
				evalUnit(buf, &dest, inv);
				buf[0] = 0;
				idx = 0;
				break;
			case '/':
				evalUnit(buf, &dest, inv);
				buf[0] = 0;
				idx = 0;
				inv = 1;
				break;
			case '2':
				evalUnit(buf, &dest, inv);
				evalUnit(buf, &dest, inv);
				buf[0] = 0;
				idx = 0;
				break;	    
			default:
				buf[idx++] = *s;
				buf[idx] = 0;
				break;
		}
		s++;
	}

	evalUnit(buf, &dest, inv);
	return dest;
}


/** @brief Convert a value from SI to given unit.
 *  @ingroup paramsdata
 *  @param[in] unit unit name to convert to
 *  @param[in] val value in SI units to be converted to unit
 *  @return converted value in unit
 *  @see GfParmUnit2SI
 */
tdble GfParmSI2Unit(const char *unit, tdble val)
{
	char buf[256];
	int  idx;
	const char *s;
	int  inv;
	tdble dest = val;
	
	if ((unit == NULL) || (strlen(unit) == 0)) return dest;
	
	s = unit;
	buf[0] = 0;
	inv = 1;
	idx = 0;
	
	while (*s != 0) {
		switch (*s) {
			case '.':
				evalUnit(buf, &dest, inv);
				buf[0] = 0;
				idx = 0;
				break;
			case '/':
				evalUnit(buf, &dest, inv);
				buf[0] = 0;
				idx = 0;
				inv = 0;
				break;
			case '2':
				evalUnit(buf, &dest, inv);
				evalUnit(buf, &dest, inv);
				buf[0] = 0;
				idx = 0;
				break;	    
			default:
				buf[idx++] = *s;
				buf[idx] = 0;
				break;
		}
		s++;
	}

	evalUnit(buf, &dest, inv);
	return dest;
}



/** @brief Get the name property of the parameter set @e handle.
 *
 *  @ingroup paramsdata
 *  @param[in] handle parameter set handle
 *  @return Name
 *  <br>NULL if failed or not set
 *  @note	The pointer returned is for immediate use, if you plan
 *   		to keep the value for a long time, it is necessary to
 *   		copy the string, because manipulating the handle will
 *   		produce an incoherent pointer.
 */
char* GfParmGetName(void *handle)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;

	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmGetName: bad handle (%p)\n", parmHandle);
		return NULL;
	}

	return conf->name;
}


/** @brief Get the filename property of the parameter set @e handle.
 *
 *  @ingroup paramsfile
 *  @param[in] handle parameter set handle
 *  @return File name
 *  <br>NULL if failed or not set
 *  @note	The pointer returned is for immediate use, if you plan
 *   		to keep the value for a long time, it is necessary to
 *   		copy the string, because manipulating the handle will
 *   		produce an incoherent pointer.
 */
char* GfParmGetFileName(void *handle)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;

	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmGetFileName: bad handle (%p)\n", parmHandle);
		return NULL;
	}

	return conf->filename;
}


/** @brief Count the number of subsections in a section in the parameter set @e handle.
 * 
 *  A subsection can have any name and structure, any section element enclosed by the section given in
 *  the @e path is a subsection.
 * 
 *  @ingroup paramslist
 *  @param[in] handle parameter set handle
 *  @param[in] path path of the section containing the subsections to count
 *  @return element count
 */
int GfParmGetEltNb(void *handle, const char *path)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct section	*section;
	int count;
	
	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmGetEltNb: bad handle (%p)\n", parmHandle);
		return 0;
	}
	
	section = (struct section *)GfHashGetStr (conf->sectionHash, path);
	if (!section) {
		return 0;
	}
	
	count = 0;
	section = GF_TAILQ_FIRST (&(section->subSectionList));
	while (section) {
		count++;
		section = GF_TAILQ_NEXT (section, linkSection);
	}
	
	return count;
}



/** @brief Go the the first subsection element in the parameter set @e handle.
 * 
 *  A subsection can have any name and structure, any section element enclosed by the section given in
 *  the @e path is a subsection.
 * 
 *  @ingroup paramslist
 *  @param[in,out] handle parameter set handle, interation state is internally stored
 *  @param[in] path path of the section containing the subsections to iterate through
 *  @return 0 Ok
 *  <br>-1 Failed
 *  @see GfParmListSeekNext
 *  @see GfParmListGetCurEltName
 */
int GfParmListSeekFirst(void *handle, const char *path)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct section *section;
	
	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmListSeekFirst: bad handle (%p)\n", parmHandle);
		return -1;
	}
	
	section = (struct section *)GfHashGetStr (conf->sectionHash, path);
	if (!section) {
		return -1;
	}
	
	section->curSubSection = GF_TAILQ_FIRST (&(section->subSectionList));
	
	return 0;
}


/** @brief Go the the next subsection element in the parameter set @e handle.
 * 
 *  A subsection can have any name and structure, any section element enclosed by the section given in
 *  the @e path is a subsection.
 * 
 *  @ingroup paramslist
 *  @param[in,out] handle parameter set handle, interation state is internally stored
 *  @param[in] path path of the section containing the subsections to iterate through
 *  @return	0 Ok
 *  <br>1 End of list reached
 *  <br>-1 Failed
 *  @see GfParmListSeekFirst
 *  @see GfParmListGetCurEltName
 */
int GfParmListSeekNext(void *handle, const char *path)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct section *section;
	
	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmListSeekNext: bad handle (%p)\n", parmHandle);
		return -1;
	}
	
	section = (struct section *)GfHashGetStr (conf->sectionHash, path);
	if ((!section) || (!section->curSubSection)) {
		return -1;
	}
	
	section->curSubSection = GF_TAILQ_NEXT (section->curSubSection, linkSection);
	
	if (section->curSubSection) {
		return 0;
	}
	
	return 1;
}


/** @brief Remove all the subsections in a section in the parameter set @e handle.
 * 
 *  A subsection can have any name and structure, any section element enclosed by the section given in
 *  the @e path is a subsection.
 * 
 *  @ingroup paramslist
 *  @param[in,out] handle parameter set handle
 *  @param[in] path path of the section containing the subsections to remove
 *  @return 0 Ok
 *	<br>-1 Error
 */
int GfParmListClean(void *handle, const char *path)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct section *listSection;
	struct section *section;
	
	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmListSeekNext: bad handle (%p)\n", parmHandle);
		return -1;
	}

	listSection = (struct section *)GfHashGetStr (conf->sectionHash, path);
	if (!listSection) {
		GfOut ("GfParmListClean: \"%s\" not found\n", path);
		return -1;
	}

	while ((section = GF_TAILQ_FIRST (&(listSection->subSectionList))) != NULL) {
		removeSection (conf, section);
	}
	
	return 0;
}


/** @brief Get current subsection name of the parameter set @e handle during subsection iteration.
 * 
 *  The internal state of the parameter set @e handle must point to a current subsection,
 *  this is done using @ref GfParmListSeekFirst or @ref GfParmListSeekNext. This call
 *  returns the name of the current subsection.
 * 
 *  A subsection can have any name and structure, any section element enclosed by the section given in
 *  the @e path is a subsection.
 * 
 *  @ingroup paramslist
 *  @param[in] handle parameter set handle
 *  @param[in] path path of the section used to iterate subsections
 *  @return	Name of the current subsection
 *  <br>NULL if failed
 *  @note	The pointer returned is for immediate use, if you plan
 *   		to keep the value for a long time, it is necessary to
 *   		copy the string, because removing the section will
 *   		produce an incoherent pointer.
 *  @see GfParmListSeekFirst
 *  @see GfParmListSeekNext
 */
char* GfParmListGetCurEltName(void *handle, const char *path)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct section *section;
	char *s;

	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmListGetCurEltName: bad handle (%p)\n", parmHandle);
		return NULL;
	}

	section = (struct section *)GfHashGetStr (conf->sectionHash, path);
	if ((!section) || (!section->curSubSection)) {
		return NULL;
	}

	s = strrchr(section->curSubSection->fullName, '/');
	if (s) {
		s++;
		return s;
	}

	return section->curSubSection->fullName;
}


/** @brief Get a string parameter from the parameter set @e handle.
 * 
 *  If the parameter does not yet exist the given default is returned.
 * 
 *  @ingroup paramsdata
 *  @param[in] parmHandle parameter set handle
 *  @param[in] path path of the parameter
 *  @param[in] key parameter key name
 *  @param[in] deflt default string
 *  @return parameter value
 *  <br>deflt if error or not found
 *  @note	The pointer returned is for immediate use, if you plan
 *   		to keep the value for a long time, it is necessary to
 *   		copy the string, because removing the attribute will
 *   		produce an incoherent pointer.
 */
const char* GfParmGetStr(void *parmHandle, const char *path, const char *key, const char *deflt)
{
	struct param *param;
	struct parmHandle *handle = (struct parmHandle *)parmHandle;
	struct parmHeader *conf = handle->conf;

	if (handle->magic != PARM_MAGIC) {
		GfFatal ("gfParmGetStr: bad handle (%p)\n", parmHandle);
		return deflt;
	}

	param = getParamByName (conf, path, key, 0);
	if (!param || !(param->value) || !strlen (param->value) || (param->type != P_STR)) {
		return deflt;
	}

	return param->value;
}


/** @brief Get a string parameter from the parameter set @e handle based on subsection iteration.
 * 
 *  The internal state of the parameter set @e handle must point to a current subsection,
 *  this is done using @ref GfParmListSeekFirst or @ref GfParmListSeekNext. If the parameter
 *  does not yet exist the given default is returned.
 * 
 *  A subsection can have any name and structure, any section element enclosed by the section given in
 *  the @e path is a subsection.
 * 
 *  @ingroup paramslist
 *  @param[in] handle parameter set handle
 *  @param[in] path path of the section used to iterate subsections
 *  @param[in] key parameter key name
 *  @param[in] deflt default string
 *  @return parameter value
 *  <br>deflt if error or not found
 *  @note	The pointer returned is for immediate use, if you plan
 *   		to keep the value for a long time, it is necessary to
 *   		copy the string, because removing the attribute will
 *   		produce an incoherent pointer.
 *  @see GfParmListSeekFirst
 *  @see GfParmListSeekNext
 */
const char* GfParmGetCurStr(void *handle, const char *path, const char *key, const char *deflt)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct section *section;
	struct param *param;
	
	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmGetCurStr: bad handle (%p)\n", parmHandle);
		return deflt;
	}

	section = (struct section *)GfHashGetStr (conf->sectionHash, path);
	if ((!section) || (!section->curSubSection)) {
		return deflt;
	}
	
	param = getParamByName (conf, section->curSubSection->fullName, key, 0);
	if (!param || !(param->value) || !strlen (param->value) || (param->type != P_STR)) {
		return deflt;
	}
	
	return param->value;
}


/** @brief Get a numerical parameter from the parameter set @e handle.
 *  
 *  If the parameter does not exist the given default value is returned without unit conversion.
 * 
 *  @ingroup paramsdata
 *  @param[in] handle parameter set handle
 *  @param[in] path path of the parameter
 *  @param[in] key parameter key name
 *  @param[in] unit unit to convert the result to (NULL if SI is desired)
 *  @param[in] deflt default value
 *  @return	parameter value
 */
tdble GfParmGetNum(void *handle, const char *path, const char *key, const char *unit, tdble deflt)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct param *param;

	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmGetNum: bad handle (%p)\n", parmHandle);
		return deflt;
	}

	param = getParamByName (conf, path, key, 0);
	if (!param ||  (param->type != P_NUM)) {
		return deflt;
	}

	if (unit) {
		return GfParmSI2Unit(unit, param->valnum);
	}
	
	return  param->valnum;
}


/** @brief Get a numerical parameter from the parameter set @e handle based on subsection iteration.
 * 
 *  The internal state of the parameter set @e handle must point to a current subsection,
 *  this is done using @ref GfParmListSeekFirst or @ref GfParmListSeekNext. If the parameter
 *  does not exist the given default value is returned without unit conversion.
 * 
 *  A subsection can have any name and structure, any section element enclosed by the section given in
 *  the @e path is a subsection.
 * 
 *  @ingroup paramslist
 *  @param[in] handle parameter set handle
 *  @param[in] path path of the section used to iterate subsections
 *  @param[in] key parameter key name
 *  @param[in] unit unit to convert the result to (NULL if SI is desired)
 *  @param[in] deflt default value
 *  @return	parameter value
 *  @see GfParmListSeekFirst
 *  @see GfParmListSeekNext
 */
tdble GfParmGetCurNum(void *handle, const char *path, const char *key, const char *unit, tdble deflt)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct section *section;
	struct param *param;
	
	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmGetCurNum: bad handle (%p)\n", parmHandle);
		return deflt;
	}

	section = (struct section *)GfHashGetStr (conf->sectionHash, path);
	if ((!section) || (!section->curSubSection)) {
		return deflt;
	}
	
	param = getParamByName(conf, section->curSubSection->fullName, key, 0);
	if (!param || (param->type != P_NUM)) {
		return deflt;
	}
	
	if (unit) {
		return GfParmSI2Unit(unit, param->valnum);
	}
	
	return  param->valnum;
}


/** @brief Set a string parameter in the parameter set @e handle.
 * 
 *  If the parameter  does not yet exist it is created. The within constraint is not checked.
 * 
 *  @ingroup paramsdata
 *  @param[in,out] handle parameter set handle
 *  @param[in] path path of the parameter
 *  @param[in] key parameter key name
 *  @param[in] val value (NULL or empty string to remove the parameter)
 *  @return 0 ok
 *  <br>-1 error
 */
int GfParmSetStr(void *handle, const char *path, const char *key, const char *val)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct param *param;
	
	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmSetStr: bad handle (%p)\n", parmHandle);
		return -1;
	}
	
	if (!val || !strlen (val)) {
		/* Remove the entry */
		removeParamByName (conf, path, key);
		return 0;
	}
	
	param = getParamByName (conf, path, key, PARAM_CREATE);
	if (!param) {
		return -1;
	}
	
	param->type = P_STR;
	freez (param->value);
	param->value = strdup (val);
	
	if (!param->value) {
		GfError ("gfParmSetStr: strdup (%s) failed\n", val);
		removeParamByName (conf, path, key);
		return -1;
	}
	
	return 0;
}


/** @brief Set a string parameter in the parameter set @e handle based on subsection iteration.
 * 
 *  The internal state of the parameter set @e handle must point to a current subsection,
 *  this is done using @ref GfParmListSeekFirst or @ref GfParmListSeekNext. If the parameter
 *  does not yet exist it is created. The within constraint is not checked.
 * 
 *  A subsection can have any name and structure, any section element enclosed by the section given in
 *  the @e path is a subsection.
 * 
 *  @ingroup paramslist
 *  @param[in,out] handle parameter set handle
 *  @param[in] path path of the section used to iterate subsections
 *  @param[in] key parameter key name
 *  @param[in] val value (NULL or empty string to remove the parameter)
 *  @return 0 ok
 *  <br>-1 error
 *  @see GfParmListSeekFirst
 *  @see GfParmListSeekNext
 */
int GfParmSetCurStr(void *handle, const char *path, const char *key, const char *val)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct section *section;
	struct param *param;

	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmSetCurStr: bad handle (%p)\n", parmHandle);
		return -1;
	}

	section = (struct section *)GfHashGetStr (conf->sectionHash, path);
	if ((!section) || (!section->curSubSection)) {
		return -1;
	}

	if (!val || !strlen (val)) {
		/* Remove the entry */
		removeParamByName (conf, section->curSubSection->fullName, key);
		return 0;
	}

	param = getParamByName (conf, section->curSubSection->fullName, key, PARAM_CREATE);
	if (!param) {
		return -1;
	}
	
	param->type = P_STR;
	freez (param->value);
	param->value = strdup (val);
	if (!param->value) {
		GfError ("gfParmSetStr: strdup (%s) failed\n", val);
		removeParamByName (conf, section->curSubSection->fullName, key);
		return -1;
	}

	return 0;
}


/** @brief Set a numerical parameter in the parameter set @e handle.
 * 
 *  If the parameter does not yet exist it is created. The value is assigned to the value, min and max.
 * 
 *  @ingroup	paramsdata
 *  @param[in,out]	handle	parameter set handle
 *  @param[in]	path	path of the parameter
 *  @param[in]	key	parameter key name
 *  @param[in]	unit	unit to convert the result to (NULL if SI desired)	
 *  @param[in]	val	value to set	
 *  @return	0	ok
 *  <br>-1	error
 */
int GfParmSetNum(void *handle, const char *path, const char *key, const char *unit, tdble val)
{
	struct parmHandle	*parmHandle = (struct parmHandle *)handle;
	struct parmHeader	*conf = parmHandle->conf;
	struct param	*param;
	
	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmSetNum: bad handle (%p)\n", parmHandle);
		return -1;
	}
	
	param = getParamByName (conf, path, key, PARAM_CREATE);
	if (!param) {
		return -11;
	}

	param->type = P_NUM;
	FREEZ (param->unit);
	if (unit) {
		param->unit = strdup (unit);
	}
	
	val = GfParmUnit2SI (unit, val);
	param->valnum = val;
	param->min = val;
	param->max = val;
	
	return 0;
}


/** @brief Set a numerical parameter in the parameter set @e handle including min and max.
 *  @ingroup	paramsdata
 *  @param[in,out]	handle	parameter set handle
 *  @param[in]	path	path of the parameter
 *  @param[in]	key	parameter key name
 *  @param[in]	unit	unit to convert the result to (NULL if SI desired)	
 *  @param[in]	val	value to set
 *  @param[in]	min	min value to set
 *  @param[in]	max	max value to set
 *  @return	0	ok
 *   		<br>-1	error
 */
int GfParmSetNumEx(void *handle, const char *path, const char *key, const char *unit, tdble val, tdble min, tdble max)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct param *param;

	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmSetNumEx: bad handle (%p)\n", parmHandle);
		return -1;
	}

	param = getParamByName (conf, path, key, PARAM_CREATE);
	if (!param) {
		return -1;
	}
	
	param->type = P_NUM;
	FREEZ (param->unit);
	if (unit) {
		param->unit = strdup (unit);
	}

	param->valnum = GfParmUnit2SI (unit, val);
	param->min = GfParmUnit2SI (unit, min);
	param->max = GfParmUnit2SI (unit, max);

	return 0;
}


/** @brief Set a numerical parameter in the parameter set @e handle based on subsection iteration.
 * 
 *  The internal state of the parameter set @e handle must point to a current subsection,
 *  this is done using @ref GfParmListSeekFirst or @ref GfParmListSeekNext. If the parameter
 *  does not yet exist it is created. The value is assigned to the value, min and max.
 * 
 *  A subsection can have any name and structure, any section element enclosed by the section given in
 *  the @e path is a subsection.
 * 
 *  @ingroup paramslist
 *  @param[in,out]	handle	parameter set handle
 *  @param[in]	path	path of the section used to iterate subsections
 *  @param[in]	key	parameter key name	
 *  @param[in]	unit	unit to convert the result to (NULL if SI is desired)	
 *  @param[in]	val	value to set	
 *  @return	0	ok
 *  <br>-1	error
 *  @see GfParmListSeekFirst
 *  @see GfParmListSeekNext
 */
int GfParmSetCurNum(void *handle, const char *path, const char *key, const char *unit, tdble val)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct section *section;
	struct param *param;

	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmSetCurNum: bad handle (%p)\n", parmHandle);
		return -1;
	}

	section = (struct section *)GfHashGetStr (conf->sectionHash, path);
	if ((!section) || (!section->curSubSection)) {
		return -1;
	}

	param = getParamByName(conf, section->curSubSection->fullName, key, PARAM_CREATE);
	if (!param) {
		return -1;
	}

	param->type = P_NUM;
	FREEZ (param->unit);
	if (unit) {
		param->unit = strdup (unit);
	}

	val = GfParmUnit2SI (unit, val);
	param->valnum = val;
	param->min = val;
	param->max = val;

	return 0;
}



/** @brief Check the values in the parameter set @e tgt against the min/max/within definitions in the @e ref parameter set.
 * 
 *  @ingroup	paramsfile
 *  @param[in]	ref	reference parameter set handle for check (min/max/within)
 *  @param[in]	tgt	target parameter set handle for check (values) 
 *  @return	0 All checked values are ok
 *	<br>-1 Some values are out of bounds
 *  @note	Only the parameters present in both sets, @e tgt and @e ref, are tested.
 *  Min/max/within values eventually present in @e tgt are not checked.
 *  @see	GfParmMergeHandles
 */
int GfParmCheckHandle(void *ref, void *tgt)
{
	struct parmHandle *parmHandleRef = (struct parmHandle *)ref;
	struct parmHandle *parmHandle = (struct parmHandle *)tgt;
	struct parmHeader *confRef = parmHandleRef->conf;
	struct parmHeader *conf = parmHandle->conf;
	struct section *curSectionRef;
	struct section *nextSectionRef;
	struct param *curParamRef;
	struct param *curParam;
	struct within *curWithinRef;
	int found;
	int error = 0;

	if ((parmHandleRef->magic != PARM_MAGIC) || (parmHandle->magic != PARM_MAGIC)) {
		GfFatal ("GfParmCheckHandle: bad handle (%p)\n", parmHandle);
		return -1;
	}

	/* Traverse all the reference tree */
	curSectionRef = GF_TAILQ_FIRST (&(confRef->rootSection->subSectionList));
	while (curSectionRef) {
		curParamRef = GF_TAILQ_FIRST (&(curSectionRef->paramList));
		while (curParamRef) {
			/* compare params */
			curParam = getParamByName (conf, curSectionRef->fullName, curParamRef->name, 0);
			if (curParam) {
				if (curParamRef->type != curParam->type) {
					GfError("GfParmCheckHandle: type mismatch for parameter \"%s\" in (\"%s\" - \"%s\")\n",
						curParamRef->fullName, conf->name, conf->filename);
					error = -1;
				} else if (curParamRef->type == P_NUM) {
					if ((curParam->valnum < curParamRef->min) || (curParam->valnum > curParamRef->max)) {
						GfError("GfParmCheckHandle: parameter \"%s\" out of bounds: min:%g max:%g val:%g in (\"%s\" - \"%s\")\n",
							curParamRef->fullName, curParamRef->min, curParamRef->max, curParam->valnum, conf->name, conf->filename);
					}
				} else {
					curWithinRef = GF_TAILQ_FIRST (&(curParamRef->withinList));
					found = 0;
					while (!found && curWithinRef) {
						if (!strcmp (curWithinRef->val, curParam->value)) {
							found = 1;
						} else {
							curWithinRef = GF_TAILQ_NEXT (curWithinRef, linkWithin);
						}
					}
					if (!found && strcmp (curParamRef->value, curParam->value)) {
						GfError("GfParmCheckHandle: parameter \"%s\" value:\"%s\" not allowed in (\"%s\" - \"%s\")\n",
							curParamRef->fullName, curParam->value, conf->name, conf->filename);
					}
				}
			}
			curParamRef = GF_TAILQ_NEXT (curParamRef, linkParam);
		}
		
		nextSectionRef = GF_TAILQ_NEXT (curSectionRef, linkSection);
		while (!nextSectionRef) {
			nextSectionRef = curSectionRef->parent;
			if (!nextSectionRef) {
				/* Reached the root */
				break;
			}
			curSectionRef = nextSectionRef;
			nextSectionRef = GF_TAILQ_NEXT (curSectionRef, linkSection);
		}
		curSectionRef = nextSectionRef;
	}

	return error;
}


/** @brief Helper function to merge a parameter into a parameter set.
 * 
 *  If the parameter @e param already exists in @e paramHandle, the values are overwritten with the values from @e param.
 *  If the parameter @e param does not yet exist in @e paramHandle, it gets created. The value and restrictions (min, max, within)
 *  in @e param are checked against the restrictions given by @e parmRef and adjusted if required. 
 * 
 *  @ingroup paramshelper
 *  @param[in,out]	parmHandle	parameter set handle
 *  @param[in]	path	path to the parameter
 *  @param[in]	paramRef reference parameter for min/max boundaries or string set restrictions
 *  @param[in]	param	parameter
 *  @see	GfParmMergeHandles
 * 
 */
static void insertParamMerge(struct parmHandle *parmHandle, char *path, struct param *paramRef, struct param *param)
{
	struct parmHeader *conf = parmHandle->conf;
	struct param *paramNew;
	struct within *withinRef;
	struct within *within;
	tdble num;
	char *str;

	paramNew = getParamByName (conf, path, param->name, PARAM_CREATE);
	if (!paramNew) {
		return;
	}
	
	if (param->type == P_NUM) {
		paramNew->type = P_NUM;
		FREEZ (paramNew->unit);
		if (param->unit) {
			paramNew->unit = strdup (param->unit);
		}
		
		if (param->min < paramRef->min) {
			num = paramRef->min;
		} else {
			num = param->min;
		}
		paramNew->min = num;
		
		if (param->max > paramRef->max) {
			num = paramRef->max;
		} else {
			num = param->max;
		}
		paramNew->max = num;
		num = param->valnum;
		
		if (num < paramNew->min) {
			num = paramNew->min;
		}
		
		if (num > paramNew->max) {
			num = paramNew->max;
		}
		paramNew->valnum = num;
	} else {
		paramNew->type = P_STR;
		FREEZ (paramNew->value);
		within = GF_TAILQ_FIRST (&(param->withinList));
		
		while (within) {
			withinRef = GF_TAILQ_FIRST (&(paramRef->withinList));
			while (withinRef) {
				if (!strcmp (withinRef->val, within->val)) {
					addWithin (paramNew, within->val);
					break;
				}
				withinRef = GF_TAILQ_NEXT (withinRef, linkWithin);
			}
			within = GF_TAILQ_NEXT (within, linkWithin);
		}
		str = NULL;
		withinRef = GF_TAILQ_FIRST (&(paramRef->withinList));
		
		while (withinRef) {
			if (!strcmp (withinRef->val, param->value)) {
				str = param->value;
				break;
			}
			withinRef = GF_TAILQ_NEXT (withinRef, linkWithin);
		}
		
		if (!str) {
			str = paramRef->value;
		}
		
		paramNew->value = strdup (str);
	}
}


/** @brief Helper function to insert a parameter into a parameter set.
 * 
 *  If the parameter @e param already exists in @e paramHandle, the values are overwritten with the values from @e param.
 *  If the parameter @e param does not yet exist in @e paramHandle, it gets created.
 * 
 *  @ingroup paramshelper
 *  @param[in,out]	parmHandle	parameter set handle
 *  @param[in]	path	path to the parameter
 *  @param[in]	param	parameter
 *  @see	GfParmMergeHandles
 * 
 */
static void insertParam(struct parmHandle *parmHandle, char *path, struct param *param)
{
	struct parmHeader *conf = parmHandle->conf;
	struct param *paramNew;
	struct within *within;

	paramNew = getParamByName (conf, path, param->name, PARAM_CREATE);
	if (!paramNew) {
		return;
	}
	
	if (param->type == P_NUM) {
		paramNew->type = P_NUM;
		FREEZ (paramNew->unit);
		if (param->unit) {
			paramNew->unit = strdup (param->unit);
		}
		paramNew->valnum = param->valnum;
		paramNew->min = param->min;
		paramNew->max = param->max;
	} else {
		paramNew->type = P_STR;
		FREEZ (paramNew->value);
		paramNew->value = strdup (param->value);
		within = GF_TAILQ_FIRST (&(param->withinList));
		while (within) {
			addWithin (paramNew, within->val);
			within = GF_TAILQ_NEXT (within, linkWithin);
		}
	}
}


/** @brief Merge two parameter sets into a new one, either containing parameters from @e ref, @e tgt or from both sets, the @e ref and @e tgt sets are not changed.
 *   
 *  Used to create a new parameter set from two exising ones, e.g. like the car category and car. If #GFPARM_MMODE_SRC
 *  mode is used, all parameterers from @e ref will exist in the new parameter set. If #GFPARM_MMODE_DST
 *  mode is used, all parameterers from @e tgt will exist in the new parameter set. You can combine #GFPARM_MMODE_SRC and
 *  #GFPARM_MMODE_DST to get all parameters from both sets into the new set.
 *   
 *  The parameter value is taken from the @e tgt set if the parameter exists in the @e tgt set. If a parameter exists in
 *  both sets (@e ref and @e tgt) and has different min/max values, then the greater min value and the smaller max value
 *  is selected, so with combining parameters it is only possible to shrink the possible range. If the parameter value
 *  does not fit the new min/max range it is adjusted.
 *   
 *  @ingroup	paramsfile
 *  @param[in]	ref	reference parameter set handle for merge
 *  @param[in]	tgt	target parameter set handle for merge
 *  @param[in]	mode	merge mode, can be any combination (binary or opearator, "|") of:
 *	<br>#GFPARM_MMODE_SRC Use parameters from @e ref and modify parameters existing in @e tgt with @e tgt
 *	<br>#GFPARM_MMODE_DST Use parameters from @e tgt and verify parameters existing in @e ref against @e ref
 *	<br>#GFPARM_MMODE_RELSRC Release @e ref handle after the merge
 *	<br>#GFPARM_MMODE_RELDST Release @e tgt handle after the merge
 *  @return	The new handle containing the merge
 *  @see	GfParmCheckHandle
 */
void *GfParmMergeHandles(void *ref, void *tgt, int mode)
{
	struct parmHandle *parmHandleRef = (struct parmHandle *)ref;
	struct parmHandle *parmHandleTgt = (struct parmHandle *)tgt;
	struct parmHandle *parmHandleOut;
	struct parmHeader *confRef = parmHandleRef->conf;
	struct parmHeader *confTgt = parmHandleTgt->conf;
	struct parmHeader *confOut;
	struct section *curSectionRef;
	struct section *nextSectionRef;
	struct section *curSectionTgt;
	struct section *nextSectionTgt;
	struct param *curParamRef;
	struct param *curParamTgt;
	const unsigned long parmHandleSize = sizeof (struct parmHandle);

	GfOut ("Merging \"%s\" and \"%s\" (%s - %s)\n", confRef->filename, confTgt->filename, ((mode & GFPARM_MMODE_SRC) ? "SRC" : ""), ((mode & GFPARM_MMODE_DST) ? "DST" : ""));

	if (parmHandleRef->magic != PARM_MAGIC) {
		GfFatal ("GfParmMergeHandles: bad handle (%p)\n", parmHandleRef);
		return NULL;
	}
	if (parmHandleTgt->magic != PARM_MAGIC) {
		GfFatal ("GfParmMergeHandles: bad handle (%p)\n", parmHandleTgt);
		return NULL;
	}

		/* Conf Header creation */
	confOut = createParmHeader ("");
	if (!confOut) {
		GfError ("gfParmReadBuf: conf header creation failed\n");
		return NULL;
	}

	/* Handle creation */
	parmHandleOut = (struct parmHandle *) calloc (1, parmHandleSize);
	if (!parmHandleOut) {
		GfError ("gfParmReadBuf: calloc (1, %lu) failed\n", parmHandleSize);
		parmReleaseHeader (confOut);
		return NULL;
	}

	parmHandleOut->magic = PARM_MAGIC;
	parmHandleOut->conf = confOut;
	parmHandleOut->flag = PARM_HANDLE_FLAG_PRIVATE;

	if (mode & GFPARM_MMODE_SRC) {
		/* Traverse all the reference tree */
		curSectionRef = GF_TAILQ_FIRST (&(confRef->rootSection->subSectionList));
		while (curSectionRef) {
			curParamRef = GF_TAILQ_FIRST (&(curSectionRef->paramList));
			while (curParamRef) {
				/* compare params */
				curParamTgt = getParamByName (confTgt, curSectionRef->fullName, curParamRef->name, 0);
				if (curParamTgt) {
					insertParamMerge (parmHandleOut, curSectionRef->fullName, curParamRef, curParamTgt);
				} else {
					insertParam (parmHandleOut, curSectionRef->fullName, curParamRef);
				}
				curParamRef = GF_TAILQ_NEXT (curParamRef, linkParam);
			}
			nextSectionRef = GF_TAILQ_FIRST (&(curSectionRef->subSectionList));
			if (nextSectionRef) {
				curSectionRef = nextSectionRef;
			} else {
				nextSectionRef = GF_TAILQ_NEXT (curSectionRef, linkSection);
				while (!nextSectionRef) {
					nextSectionRef = curSectionRef->parent;
					if (!nextSectionRef) {
						/* Reached the root */
						break;
					}
					curSectionRef = nextSectionRef;
					nextSectionRef = GF_TAILQ_NEXT (curSectionRef, linkSection);
				}
				curSectionRef = nextSectionRef;
			}
		}
	}

	if (mode & GFPARM_MMODE_DST) {
		/* Traverse all the target tree */
		curSectionTgt = GF_TAILQ_FIRST (&(confTgt->rootSection->subSectionList));
		while (curSectionTgt) {
			curParamTgt = GF_TAILQ_FIRST (&(curSectionTgt->paramList));
			while (curParamTgt) {
				/* compare params */
				curParamRef = getParamByName (confRef, curSectionTgt->fullName, curParamTgt->name, 0);
				if (curParamRef) {
					insertParamMerge (parmHandleOut, curSectionTgt->fullName, curParamRef, curParamTgt);
				} else {
					insertParam (parmHandleOut, curSectionTgt->fullName, curParamTgt);
				}
				curParamTgt = GF_TAILQ_NEXT (curParamTgt, linkParam);
			}
			
			nextSectionTgt = GF_TAILQ_FIRST (&(curSectionTgt->subSectionList));
			if (nextSectionTgt) {
				curSectionTgt = nextSectionTgt;
			} else {
				nextSectionTgt = GF_TAILQ_NEXT (curSectionTgt, linkSection);
				while (!nextSectionTgt) {
					nextSectionTgt = curSectionTgt->parent;
					if (!nextSectionTgt) {
						/* Reached the root */
						break;
					}
					curSectionTgt = nextSectionTgt;
					nextSectionTgt = GF_TAILQ_NEXT (curSectionTgt, linkSection);
				}
				curSectionTgt = nextSectionTgt;
			}
		}
	}

	if (mode & GFPARM_MMODE_RELSRC) {
		GfParmReleaseHandle(ref);
	}

	if (mode & GFPARM_MMODE_RELDST) {
		GfParmReleaseHandle(tgt);
	}

	GF_TAILQ_INSERT_HEAD (&parmHandleList, parmHandleOut, linkHandle);

	return (void*)parmHandleOut;
}


/** @brief Get the min and max of a numerical parameter from the parameter set @e handle.
 *  @ingroup	paramsdata
 *  @param[in]	handle	parameter set handle
 *  @param[in]	path	path of the parameter
 *  @param[in]	key	parameter key name	
 *  @param[out]	min	Receives the min value
 *  @param[out]	max	Receives the max value
 *  @return	0 Ok
 *	<br>-1 Parameter does not exist
 */
int GfParmGetNumBoundaries(void *handle, const char *path, const char *key, tdble *min, tdble *max)
{
	struct parmHandle *parmHandle = (struct parmHandle *)handle;
	struct parmHeader *conf = parmHandle->conf;
	struct param *param;

	if (parmHandle->magic != PARM_MAGIC) {
		GfFatal ("GfParmGetNumBoundaries: bad handle (%p)\n", parmHandle);
		return -1;
	}

	param = getParamByName (conf, path, key, 0);
	if (!param || (param->type != P_NUM)) {
		return -1;
	}

	*min = param->min;
	*max = param->max;

	return 0;
}

 
