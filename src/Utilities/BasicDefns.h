/**@file BasicDefns.h.
   This is a set of VERY BASIC useful definitions.
   They shouldn't be particularly specific to any application.
   @date 8/25/98
   @author Lucia K. Dale
*/

#ifndef BasicDefns_h
#define BasicDefns_h

/////////////////////////////////////////////////////////////////////
//Include standard headers
#ifdef HPUX
#include <sys/io.h>
#endif

using namespace std;
#include <iostream>

//////////////////////////////////////////////////////////////////////////
// Check which CD library will be used
#ifndef USE_CSTK
#ifndef USE_VCLIP
#ifndef USE_RAPID
#ifndef USE_PQP
#ifndef USE_SOLID
  #ifndef NO_CD_USE
    #error You have to specify at least one collision detection library.
  #endif
#endif
#endif
#endif
#endif
#endif

/// Legal Types of Collision Detecters
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

/// Proximity Query Package
#ifdef USE_PQP
        PQP,
#endif
/**< enum PQP */

/// SOLID
#ifdef USE_SOLID
        SOLID,
#endif
/**< enum SOLID */

/// first user defined cd set, if any
        CD_USER1};    
/**< enum CD_USER1 */

/////////////////////////////////////////////////////////////////////
//-----------------------------------
// Constants
//---------------------------------
/** @name Constants used in OBPRM.*/
//@{
///////////////////////////////////////////////////////////////////////////
#ifndef PI
#define PI              3.141592653589793
#endif
#define TWOPI           (PI*2.0)

/////////////////////////////////////////////////////////////////////////
#define ORIENTATION_RES             0.05    ///<Resolution for Orientation
#define POSITION_RES_FACTOR         0.05    ///<Resolution for Position

#define NULL_WT_INFO -999               ///< to pad weight fields for graph conversions

#define INVALID_LP -999                 ///< invalid local planner id
#define INVALID_RNGSEED -999            ///< invalid seed value for Random Number Generator

#define MAX_INT  999999999
#define INVALID_INT -999
#define MAX_DBL  999999999.99999
#define INVALID_DBL -999
//@}

#endif
