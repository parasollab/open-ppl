// $Id$
/////////////////////////////////////////////////////////////////////
//  Transformation.c
//
//  Created   3/ 1/98 Aaron Michalk
//  Modified  3/18/98 Aaron Michalk
//  Modified  6/ 4/98 Wookho Son
/////////////////////////////////////////////////////////////////////

#include "Transformation.h"
#include <math.h>

#define DEGTORAD 3.1415926535/180.0

//===================================================================
//  Static member initialization
//===================================================================
const Transformation Transformation::Identity = Transformation(Orientation(IdentityMatrix), Vector3D(0.0, 0.0, 0.0));

//===================================================================
//  Constructors and Destructor
//===================================================================
Transformation::Transformation() :
    position(0.0, 0.0, 0.0),
    orientation(Orientation::Matrix)
{
}

Transformation::Transformation(const Orientation & _orientation, const Vector3D & _position) :
    position(_position),
    orientation(_orientation)
{
}

//==============================================================================
// Function: Create a transformation corresponding to the given DH parameters
//
// Refer Craig Eq 3.6 (page 84)
//==============================================================================
Transformation::Transformation(const DHparameters & _dh) :
    position(_dh.a, -sin(_dh.alpha*DEGTORAD)*_dh.d, cos(_dh.alpha*DEGTORAD)*_dh.d),
    orientation(Orientation::Matrix)
{
    orientation.matrix[0][0] = cos(_dh.theta*DEGTORAD);
    orientation.matrix[0][1] = -sin(_dh.theta*DEGTORAD);
    orientation.matrix[0][2] = 0.0;
    orientation.matrix[1][0] = sin(_dh.theta*DEGTORAD)*cos(_dh.alpha*DEGTORAD);
    orientation.matrix[1][1] = cos(_dh.theta*DEGTORAD)*cos(_dh.alpha*DEGTORAD);
    orientation.matrix[1][2] = -sin(_dh.alpha*DEGTORAD);
    orientation.matrix[2][0] = sin(_dh.theta*DEGTORAD)*sin(_dh.alpha*DEGTORAD);
    orientation.matrix[2][1] = cos(_dh.theta*DEGTORAD)*sin(_dh.alpha*DEGTORAD);
    orientation.matrix[2][2] = cos(_dh.alpha*DEGTORAD);
}

Transformation::Transformation(const Transformation & _t) :
    position(_t.position),
    orientation(_t.orientation)
{
}

Transformation::~Transformation() {
}

//===================================================================
//  Operators
//===================================================================
Transformation & Transformation::operator+(const Transformation & _transformation) {
    orientation = orientation + _transformation.orientation;
    position = position + _transformation.position;

    return *this;
}

Transformation Transformation::operator-(const Transformation & _transformation) {
    orientation = orientation - _transformation.orientation;
    position = position - _transformation.position;

    return *this;
}

Vector3D Transformation::operator*(const Vector3D & _vector) {
    return orientation * _vector + position;
}

Transformation & Transformation::operator=(const Transformation & _t) {
    position = _t.position;
    orientation = _t.orientation;
    return *this;
}

//===================================================================
//  Refer to Craig Eq 2.45 
//===================================================================
Transformation Transformation::operator*(const Transformation & _t) {
    return Transformation(orientation * _t.orientation, orientation * _t.position + position);
}

//===================================================================
//  Inverse
//
//  Function: Creates the reverse transformation of "this"
//
//  Output: The reverse transformation
//
//  Leaves "this" transformation unchanged.
//  Refer to Craig Eq 2.45 
//===================================================================
Transformation Transformation::Inverse() {
    return Transformation(orientation.Inverse(), -(orientation.Inverse() * position));
}

//===================================================================
//  Invert
//
//  Inverts this transformation
//===================================================================
void Transformation::Invert() {
    orientation.Invert();
    position = -(orientation * position);
}


//===================================================================
//  Read
//===================================================================
void Transformation::Read(ifstream & _is) {
    position.Read(_is);
    orientation.Read(_is);
}

//===================================================================
//  Write
//===================================================================
void Transformation::Write(ostream & _os) {
    position.Write(_os);
    orientation.Write(_os);
}



