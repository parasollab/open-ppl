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
//
////////////////////////////////////////////////////

#ifndef util_h
#define util_h


//#include <cstkSmallAPI.h>
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

// normalize a value to [0,b]
inline double Normalize_wrt_b(double a, double b){
    //double c;
    //c = a;
    while(a >= b) a -= b;
    while (a < 0) a += b;
    return a;
}

// Calculate the minimum DIRECTED angular distance between two angles
// normalized to 1.0
inline double DirectedAngularDistance(double a,double b)
{
    double Mdpos,Mdneg;
    if(a > b){
        Mdneg = a-b;
        Mdpos = 1.0-Mdneg;
    }
    else{
        Mdpos = b-a;
        Mdneg = 1.0-Mdpos;
    }
    Normalize_wrt_b(Mdneg,1.0);
    Normalize_wrt_b(Mdpos,1.0);
    if(Mdneg < Mdpos){
        return -Mdneg;
    }
    else{
        return Mdpos;
    }
}


void
WritePathTranformationMatrices(char output_file[80], vector<Cfg> path, Environment *env);


inline double min(double a, double b){
    if(a < b) return a;
    else return b;
}

inline double max(double a, double b){
    if(a > b) return a;
    else return b;
}

// Return the square of a.
inline double sqr(double a)
{
    return a*a;
}


#endif
