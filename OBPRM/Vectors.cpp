// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Vectors.c
//
//  General Description
//
//	Specifies the 
//      "standard" input & output operations on each new class 
//	that have been typedef'ed in the "Vector.h" file included.
//
//  Created
//      7/26/98  Lucia K. Dale
/////////////////////////////////////////////////////////////////////

#include <iomanip.h>
#include "Vectors.h"

//---------------------------------------------
// Input/Output operators for Vector3D
//---------------------------------------------
istream& operator>> (istream&s, Vector3D &pt){
	pt = Vector3D(s);
        return s;
};
ostream& operator<< (ostream&s, const Vector3D &pt){
        return s<<setw(4)<<pt.getX()<<' '
                <<setw(4)<<pt.getY()<<' '
                <<setw(4)<<pt.getZ();
};

