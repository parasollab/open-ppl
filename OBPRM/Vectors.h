// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Vectors.h
//
//  General Description
//      This are the actual typedef's for the 2 vector classes we
//	have a use for so far.
//	
//	Vector3D: is used for vertices of polyhedrons
//
//	Cfg	: is used by OBPRM for c-space representations of
//			moving objects (ie, robots) in the workplace.
//
//	"Standard" input & output operations on each new class have 
//	been provided as well.
//
//  Created
//      7/26/98  Lucia K. Dale
/////////////////////////////////////////////////////////////////////

#ifndef Vectors_h
#define Vectors_h

#include "VectorConstantSize.h"

//-------------------------------------------------
// Vector3D:
//	 class is declared with "std" I/O operators
//-------------------------------------------------
typedef Vector3<double> Vector3D;
ostream& operator<< (ostream&, const Vector3D &);
istream& operator>> (istream&, Vector3D &);

#endif

