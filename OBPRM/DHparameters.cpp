// $Id$
/////////////////////////////////////////////////////////////////////
//  DHparameters.c
//
//  Created   3/ 1/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////
#include <math.h>

#include "DHparameters.h"
#include "Transformation.h"

#define DEGTORAD (3.1415926535/180.0)

//===================================================================
//  Constructors and Destructor
//===================================================================
DHparameters::DHparameters(double _alpha, double _a, double _d, double _theta) {
    alpha = _alpha;
    a     = _a;
    d     = _d;
    theta = _theta;
}

DHparameters::DHparameters(Transformation & _t) {
    //---------------------------------------------------------------
    // This assumes that the transformation _t can be decomposed
    // into DHparameters.
    //
    // Craig Eq. 3.6
    //---------------------------------------------------------------
    alpha = atan2(-_t.orientation.matrix[1][2], _t.orientation.matrix[2][2])/DEGTORAD;
    a = _t.position.getX();
    d = -_t.position.getY()/sin(alpha*DEGTORAD);
    theta = atan2(-_t.orientation.matrix[0][1], _t.orientation.matrix[0][0])/DEGTORAD;
}

DHparameters::~DHparameters() {
}

//===================================================================
//  Read
//===================================================================
void DHparameters::Read(ifstream & _is) {
    _is >> alpha;
    _is >> a;
    _is >> d;
    _is >> theta;
}

//===================================================================
//  Write
//===================================================================
void DHparameters::Write(ostream & _os) {
    _os << alpha << " ";
    _os << a << " ";
    _os << d << " ";
    _os << theta << " ";
}

