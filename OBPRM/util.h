// $Id$
/////////////////////////////////////////////////////
//
//	util.h
//
//      Created 7/17/98         Daniel Vallejo
//
//  Last Modified By:   
//      08/24/98  Daniel Vallejo
//      1/13/99   Guang Song
//	06/29/2000 Guang. rewrite some functions.
//
////////////////////////////////////////////////////

#ifndef util_h
#define util_h


#ifdef HPUX
#include <sys/io.h>
#endif

#include "Body.h"
#include "OBPRM.h"
#include <vector.h>

class Environment;

const double Pi = 3.1415927;
const double BETA_EPSILON = 1.0e-30;

typedef double t44[4][4];


// Calculate the minimum DIRECTED angular distance between two angles
// normalized to 1.0

double DirectedAngularDistance(double a,double b);


inline double min(double a, double b){
    return a < b ? a : b;
}

inline double max(double a, double b){
    return a > b ? a : b;
}

// Return the square of a.
inline double sqr(double a)
{
    return a*a;
}


void
WritePathTranformationMatrices(char output_file[80], vector<Cfg> path, Environment *env);

#endif
