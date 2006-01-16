/////////////////////////////////////////////////////////////////////
/**@file OBPRM.h
*This is a set of definitions useful to the OBPRM application specifically.
*Almost all the OBPRM specific files will need 
*to include them.
*
*@author Lucia K. Dale
*@date  8/25/98
*/
/////////////////////////////////////////////////////////////////////

#ifndef OBPRMDef_h
#define OBPRMDef_h

/////////////////////////////////////////////////////////////////////
//Include standard headers
#ifdef HPUX
#include <sys/io.h>
#endif
#include "Defines.h"

/////////////////////////////////////////////////////////////////////
//Include OBPRM header
#include "BasicDefns.h"

/////////////////////////////////////////////////////////////////////
//-----------------------------------
// Constants
//---------------------------------
/** @name Constants used in OBPRM.*/
//@{
#define ORIENTATION_RES             0.05    ///<Resolution for Orientation
#define POSITION_RES_FACTOR         0.05    ///<Resolution for Position

#define NULL_WT_INFO -999               ///< to pad weight fields for graph conversions

#define INVALID_SID -999                ///<Invalid Set ID
#define INVALID_EID -999                ///<Invalid Element ID

#define INVALID_DM -999                 ///< invalid dm id
#define INVALID_CD -999                 ///< invalid cd id
#define INVALID_LP -999                 ///< invalid local planner id
#define INVALID_GN -999                 ///< invalid generate node id
#define INVALID_CN -999                 ///< invalid connect node id
#define INVALID_RNGSEED -999            ///< invalid seed value for Random Number Generator

#define ARGSTRING_LENGTH  256
#define MAX_CN             10
#define MAX_GN             10
#define MAX_LP             10
#define MAX_CD             10
#define MAX_DM             10
#define MAX_CFG        10

//@}

/** @name NMA: for edge weights.
* @warning Not good values or placement (in BasicDefns?).
*/
//@{
#define MAX_INT  999999999
#define INVALID_INT -999
#define MAX_DBL  999999999
#define INVALID_DBL -999
//@}

#endif
