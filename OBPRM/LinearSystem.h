// $Id$
////////////////////////////////////////////////////////////////////////////
//  LinearSystem.h
//
//  Created   6/ 4/98  Wookho Son
////////////////////////////////////////////////////////////////////////////

#ifndef LinearSystem_h
#define LinearSystem_h

#include <stdlib.h>
#include <fstream.h>
#include <iostream.h>

#include <nrutil.h>

#include "Matrix.h"


class LinearSystem {
public:
    //----------------------------------------------------------------------
    //  Constructors and Destructor
    //----------------------------------------------------------------------
    LinearSystem(double * _m, int _nrow, int _ncol, double *_rhs, int n);
    LinearSystem(Matrix & _m, double *_rhs);
    ~LinearSystem();

    //----------------------------------------------------------------------
    //  Methods
    //----------------------------------------------------------------------
    void Solve();
    void PrintSolution();
    double * GetSolution();

protected:
    // Note that class, "LinearSystem", is a friend of the class, "Matrix"
    // Friend class allows this class an access to all the data members of "Matrix" class
    Matrix  lhs;  // left-hand-side is the matrix "m" itself
    double  *rhs; // right-hand-side
    double  *s;   // Singular values, if needed
    double  *x;   // Solution

};


//==========================================================================
//  GetSolution
//==========================================================================
inline double * LinearSystem::GetSolution() {
   return  x;
}

#endif


