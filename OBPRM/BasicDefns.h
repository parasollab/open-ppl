// $Id$
/////////////////////////////////////////////////////////////////////
//
//  BasicDefns.h
//
//  General Description
//      This is a set of VERY BASIC useful definitions.
//	They shouldn't be particularly specific to any application.
//
//  Created
//      8/25/98  Lucia K. Dale
//  Last Modified By:
//      8/29/98  Lucia K. Dale
//
/////////////////////////////////////////////////////////////////////

#ifndef BasicDefns_h
#define BasicDefns_h

#define PI              3.14159
#ifndef USE_CSTK
#ifndef USE_VCLIP
#ifndef USE_RAPID
#error You have to specify at least one collision detection library.
#endif
#endif
#endif
#define TWOPI           (PI*2.0)

/*
#ifndef bool
 typedef int bool;
 #define true 1
 #define false 0
#endif
brc
*/ 
//added true false brc
#define TRUE 1
#define FALSE 0

#ifndef OK
#define OK  0
#endif

/* brc look for maxfloat */

#ifndef MAXFLOAT 
#define MAXFLOAT 99999999999999999.99999
#endif
#ifndef ERROR
#define ERROR 1
#endif

//---------------------------------------------------------------
// Legal types of collision detection data structures
//---------------------------------------------------------------
                        // Collision Detecters
enum cd_predefined {    //--------------------
#ifdef USE_CSTK
        CSTK,           // c-space toolkit
#endif
#ifdef USE_VCLIP
        VCLIP,          // voronoi clip 
#endif
#ifdef USE_RAPID
	RAPID,		// Robust and Accurate Polygon Interference Detection
#endif
        CD_USER1};      // first user defined cd set, if any


#endif
