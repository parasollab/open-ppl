#include "DHparameters.h"
#include "Transformation.h"
#include <math.h>

DHparameters::DHparameters(double _alpha, double _a, double _d, double _theta) : 
  m_alpha(_alpha), m_a(_a), m_d(_d), m_theta(_theta) {
  }

istream& 
operator>>(istream& _is, DHparameters& _d){
  return _is >> _d.m_alpha >> _d.m_a >> _d.m_d >> _d.m_theta;
}

ostream& 
operator<<(ostream& _os, const DHparameters& _d){
  return _os << _d.m_alpha << " " << _d.m_a << " " 
    << _d.m_d << " " << _d.m_theta << " ";
}

bool 
DHparameters::operator==(const DHparameters& _d) const {
  return m_alpha == _d.m_alpha && m_a == _d.m_a && m_d == _d.m_d && m_theta == _d.m_theta;
}

Transformation
DHparameters::GetTransformation() const {
  Vector3d pos(m_a, -sin(m_alpha)*m_d, cos(m_alpha)*m_d);
  Matrix3x3 rot;
  getMatrix3x3(rot, 
      cos(m_theta), -sin(m_theta), 0.0,
      sin(m_theta)*cos(m_alpha), cos(m_theta)*cos(m_alpha), -sin(m_alpha),
      sin(m_theta)*sin(m_alpha), cos(m_theta)*sin(m_alpha), cos(m_alpha));
  return Transformation(pos, Orientation(rot));
}
