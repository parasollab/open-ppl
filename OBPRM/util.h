// $Id$

/**@file util.h
   @date 7/17/98
   @author Daniel Vallejo
*/

#ifndef util_h
#define util_h


#ifdef HPUX
#include <sys/io.h>
#endif

#include "Body.h"
#include "OBPRM.h"
#include <vector.h>
#include <ctype.h>

class Environment;


/** Calculate the minimum DIRECTED angular distance between two angles
normalized to 1.0 */
double DirectedAngularDistance(double a,double b);


inline double min(double a, double b){
    return a < b ? a : b;
}

inline double max(double a, double b){
    return a > b ? a : b;
}

/// Return the square of a.
inline double sqr(double a)
{
    return a*a;
}

// Cfgs input & output from/to files.
void WritePathTranformationMatrices(char output_file[80], 
     const vector<Cfg> &path, Environment *env);
void ReadCfgs(char *filename, vector<Cfg> &cfgs);

// general functions.
bool VerifyFileExists(char *_fname);
template <class T> bool readfield (istream &_is, T *element);



//-----------------------------------------------------------
// implementation for the template function.
//-----------------------------------------------------------
#define COMMENT_DELIMITER '#'
#define LINEMAX 256
template <class T> bool readfield (istream &_is, T *element) {

  char c;
  char ThrowAwayLine[LINEMAX];

  while ( _is.get(c) ) {
    if (c == '#')
        _is.getline(ThrowAwayLine,LINEMAX,'\n');
    else if (! isspace(c) ) {
        _is.putback(c);
        if (_is >> *element) return true;
        else               break;
    }
  }
  
  // could not read correctly ...
  cout << "Error in reading!!! at util::readfield. " << endl;
  return false;
}


#endif
