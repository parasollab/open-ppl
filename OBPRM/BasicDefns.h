// $Id$

/**@file BasicDefns.h.
   This is a set of VERY BASIC useful definitions.
   They shouldn't be particularly specific to any application.
   @date 8/25/98
   @author Lucia K. Dale
*/


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

#define TRUE 1
#define FALSE 0

#ifndef OK
#define OK  0
#endif

#ifndef MAXFLOAT
#define MAXFLOAT 99999999999999999.99999
#endif
#ifndef ERROR
#define ERROR 1
#endif


/// Lega Types of Collision Detecters
enum cd_predefined {

/// c-space toolkit
#ifdef USE_CSTK
        CSTK,    
#endif
/**< enum CSTK */


/// voronoi clip
#ifdef USE_VCLIP
        VCLIP, 
#endif
/**< enum VCLIP */


/// Robust and Accurate Polygon Interference Detection
#ifdef USE_RAPID
    RAPID,
#endif
/**< enum RAPID */

/// first user defined cd set, if any
        CD_USER1};    
/**< enum CD_USER1 */


#endif
