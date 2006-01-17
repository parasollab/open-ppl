// $Id$

/**@file Vectors.h
  *This are the actual typedef's for the 2 vector classes we
  *have a use for so far.
  *Vector3D: is used for vertices of polyhedrons
  *
  *Cfg : is used by OBPRM for c-space representations of
  *       moving objects (ie, robots) in the workplace.
  *
  *"Standard" input & output operations on each new class have
  *been provided as well.
  *
  *@date 7/26/98
  *@author Lucia K. Dale
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Vectors_h
#define Vectors_h
////////////////////////////////////////////////////////////////////////////////////////////
#include "VectorConstantSize.h"

/**
  *Vector3D: class is declared with "std" I/O operators
  *A 3D vector with double elements.
  */
typedef Vector3<double> Vector3D;

///Output to given output stream.
ostream& operator<< (ostream&, const Vector3D &);
///Read from given input stream. Read 3 double float-point numbers.
istream& operator>> (istream&, Vector3D &);

typedef Vector6<double> Vector6D;

#endif

