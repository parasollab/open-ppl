// $Id$
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

#ifndef OBPRM_h
#define OBPRM_h

/////////////////////////////////////////////////////////////////////
//Include standard headers
#ifdef HPUX
#include <sys/io.h>
#endif
#include "Defines.h"
//#include <vector.h> //for bool definition in CC
//#include <iostream.h>
#include <assert.h>


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

//-----------------------------------
// c-space representations
//-----------------------------------
//#include "Cfg.h"  

//-----------------------------------
// choose represention for edge weights in roadmap graph, some samples
//-----------------------------------

class WeightObject;

//-----------------------------------
// now, actually choose weight type 
//-----------------------------------
#ifndef WEIGHT
typedef WeightObject WEIGHT;
#endif


#endif
