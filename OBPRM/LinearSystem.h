// $Id$

/**@file LinearSystem.h
   @date 6/4/98
   @author Wookho Son
*/

#ifndef LinearSystem_h
#define LinearSystem_h

#include "Matrix.h"


/**
Note that class, "LinearSystem", is a friend of the class, "Matrix"
Friend class allows this class an access to all the data members of "Matrix" class
*/
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
    /// left-hand-side is the matrix "m" itself
    Matrix  lhs;
    /// right-hand-side
    double  *rhs;
    /// Singular values, if needed
    double  *s;
    /// Solution
    double  *x;

};


//==========================================================================
//  GetSolution
//==========================================================================
inline double * LinearSystem::GetSolution() {
   return  x;
}

#endif


