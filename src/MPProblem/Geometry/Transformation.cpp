/////////////////////////////////////////////////////////////////////
//  Transformation.c
/////////////////////////////////////////////////////////////////////

#include "Transformation.h"
#include "DHparameters.h"
#include <math.h>

//===================================================================
//  Static member initialization
//===================================================================
const Transformation Transformation::Identity = Transformation(Orientation(IdentityMatrix), Vector3D(0.0, 0.0, 0.0));

//===================================================================
//  Constructors and Destructor
//===================================================================
Transformation::Transformation() :
  m_position(0.0, 0.0, 0.0),
  m_orientation(Orientation::Matrix){
}

Transformation::Transformation(const Orientation& _orientation, const Vector3D& _position) :
  m_position(_position),
  m_orientation(_orientation){
}

Transformation::Transformation(const vector<double>& _configuration, Orientation::OrientationType _type){
  m_position[0] = _configuration[0];
  m_position[1] = _configuration[1];
  m_position[2] = _configuration[2];

  //alpha, beta, and gamma are in opposite order as FixedXYZ
  Orientation myOri(_type, _configuration[5], _configuration[4], _configuration[3]);
  m_orientation = myOri;
}

//==============================================================================
// Function: Create a transformation corresponding to the given DH parameters by
// shuffling around number in the matrix appropriately.
//==============================================================================
Transformation::Transformation(const DHparameters& _dh) :
  m_position(_dh.a, -sin(_dh.alpha)*_dh.d, cos(_dh.alpha)*_dh.d),
  m_orientation(Orientation::Matrix){
    m_orientation.matrix[0][0] = cos(_dh.theta);
    m_orientation.matrix[0][1] = -sin(_dh.theta);
    m_orientation.matrix[0][2] = 0.0;
    m_orientation.matrix[1][0] = sin(_dh.theta)*cos(_dh.alpha);
    m_orientation.matrix[1][1] = cos(_dh.theta)*cos(_dh.alpha);
    m_orientation.matrix[1][2] = -sin(_dh.alpha);
    m_orientation.matrix[2][0] = sin(_dh.theta)*sin(_dh.alpha);
    m_orientation.matrix[2][1] = cos(_dh.theta)*sin(_dh.alpha);
    m_orientation.matrix[2][2] = cos(_dh.alpha);
  }

Transformation::Transformation(const Transformation& _t) :
  m_position(_t.m_position),
  m_orientation(_t.m_orientation){
  }

Transformation::~Transformation() {
}

//===================================================================
//  Operators
//===================================================================
//this will take the existing transformation of the object we are operating on
//and add the transformation algebraically of the object specified in the
//parameter.
Transformation&
Transformation::operator+(const Transformation& _transformation) {
  m_orientation = m_orientation + _transformation.m_orientation;
  m_position = m_position + _transformation.m_position;

  return *this;
}

//this does the same thing as the operator above, but with subtraction.
Transformation&
Transformation::operator-(const Transformation& _transformation) {
  m_orientation = m_orientation - _transformation.m_orientation;
  m_position = m_position - _transformation.m_position;

  return *this;
}

//similar to the + operator, but adding position and orientation form a Vector3D
//object instead.
Vector3D
Transformation::operator*(const Vector3D& _vector) {
  return m_orientation * _vector + m_position;
}

//Does the same thing as the copy constructor.
Transformation&
Transformation::operator=(const Transformation& _t) {
  m_position = _t.m_position;
  m_orientation = _t.m_orientation;
  return *this;
}

//===================================================================
//  Refer to Craig Eq 2.45 
//===================================================================
//copy position and orientation from parameter.
Transformation
Transformation::operator*(const Transformation& _t) {
  return Transformation(m_orientation * _t.m_orientation, m_orientation * _t.m_position + m_position);
}

bool
Transformation::operator==(const Transformation& t) const {
  return m_position == t.m_position && m_orientation == t.m_orientation;
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
  return Transformation(m_orientation.Inverse(), -(m_orientation.Inverse() * m_position));
}

//===================================================================
//  Invert
//
//  Inverts this transformation
//===================================================================
void
Transformation::Invert() {
  m_orientation.Invert();
  m_position = -(m_orientation * m_position);
}

istream& 
operator>>(istream& _is, Transformation& _t){
  return _is >> _t.m_position >> _t.m_orientation;
}

ostream& operator<<(ostream& _os, const Transformation& _t){
  return _os << _t.m_position << _t.m_orientation;
}
