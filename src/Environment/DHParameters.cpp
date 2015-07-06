#include "DHParameters.h"

#include "Transformation.h"

DHParameters::
DHParameters(double _alpha, double _a, double _d, double _theta) :
  m_alpha(_alpha), m_a(_a), m_d(_d), m_theta(_theta) {
  }

Transformation
DHParameters::GetTransformation() const {
  Vector3d pos(m_a, -sin(m_alpha)*m_d, cos(m_alpha)*m_d);
  Matrix3x3 rot;
  getMatrix3x3(rot,
      cos(m_theta), -sin(m_theta), 0.0,
      sin(m_theta)*cos(m_alpha), cos(m_theta)*cos(m_alpha), -sin(m_alpha),
      sin(m_theta)*sin(m_alpha), cos(m_theta)*sin(m_alpha), cos(m_alpha));
  return Transformation(pos, Orientation(rot));
}

istream&
operator>>(istream& _is, DHParameters& _d){
  return _is >> _d.m_alpha >> _d.m_a >> _d.m_d >> _d.m_theta;
}

ostream&
operator<<(ostream& _os, const DHParameters& _d){
  return _os << _d.m_alpha << " " << _d.m_a << " "
    << _d.m_d << " " << _d.m_theta << " ";
}

