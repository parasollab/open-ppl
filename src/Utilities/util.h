/**@file util.h
  *
  *This is a file containing many kinds of methods.
  *
  *@date 7/17/98
  *@author Daniel Vallejo
  */

#ifndef util_h
#define util_h

/////////////////////////////////////////////////////////////////////////////////////////
//Include standard headers

#ifdef HPUX
#include <sys/io.h>
#endif

#include <ctype.h>
#include <vector>
#include <iostream>
#include <fstream>
using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "IOUtils.h"
#include "Basic.h"
#include "RoadmapGraph.h"
/////////////////////////////////////////////////////////////////////////////////////////
class Cfg;
class Environment;
/////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////
//
//
//
// Basic Utility.
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////

/**@name Basic Utility */
//@{

/**Calculate the minimum DIRECTED angular distance 
  *between two angles normalized to 1.0 
  */
double DirectedAngularDistance(double a,double b);

#ifndef _H_UTILITY
///Return minimun between a and b.
inline double min(double a, double b){
    return a < b ? a : b;
}

///Return maximun between a and b.
inline double max(double a, double b){
    return a > b ? a : b;
}

/// Return the square of a.
inline double sqr(double a)
{
    return a*a;
}
#endif

double GaussianDistribution(double m, double s);

//@}

/////////////////////////////////////////////////////////////////////////////////////////
//
//
//
// Random Number Generator
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////


  //return non-negative double-prevision floating-point values 
  //uniformly distributed over the interval [0.0, 1.0)
  //call drand48()
  double OBPRM_drand();
  
  //return non-negative long integers uniformly distributed over the interval [0, 2**31)
  //call lrand48()
  long OBPRM_lrand();

  //return signed long integers uniformly distributed over the interval [-2**31, 2**31)
  //call mrand48()
  long OBPRM_mrand();

  // normally(gaussian) distributed random number generator.
  // when reset is 1, it reset the internal static variable and return 0.0
  double OBPRM_grand(bool reset = false);
  
  /* use seedval as the seed
   */
  long OBPRM_srand(long seedval = 0x1234ABCD);
  
  /* "baseSeed" is a static variable in this function
     we use baseSeed, methodName and nextNodeIndex to generate a deterministic seed,
     then call seed48()
     when reset is 1, baseSeed will be reset
   */
  long OBPRM_srand(std::string methodName, int nextNodeIndex, long base = 0x1234ABCD, bool reset = false);

/////////////////////////////////////////////////////////////////////////////////////////
//
//
//
// implementation for the template function.
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
//Modified for VC
/////////////////////////////////////////////////////////////////////////////////////////

#ifdef _WIN32

////////////////////////////////////////////////////////////////////////////////////////
// Following functions define M_PI and drand48, which are not starndard c library and 
// definitions. In addition, rint used to round off float points to int is also here.
/////////////////////////////////////////////////////////////////////////////////////////

#define M_PI PI //reference PI above

extern "C" {
//Implementation of these functions are located in util.cpp
double drand48();
double erand48(register unsigned short *xsubi);
long irand48(register unsigned short m);
long krand48(register unsigned short *xsubi, unsigned short m);
long lrand48();
long mrand48();
static void next();
void srand48(long seedval);
unsigned short * seed48(unsigned short seed16v[3]);
void lcong48(unsigned short param[7]);
long nrand48(register unsigned short *xsubi);
long jrand48(register unsigned short *xsubi);

/**Round to closest integer.
  *The rint() function rounds x to an integer value according
  *to the prevalent rounding mode.  The default rounding mode
  *is to round to the nearest integer.
  *@return The  rint() function returns the integer value as a float-
  *ing-point number.
  */
double rint(double x);

} //end extern "C"

#endif //_WIN32
/////////////////////////////////////////////////////////////////////////////////////////

#endif
