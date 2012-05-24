#include "DHparameters.h"
#include "Transformation.h"
#include <math.h>

DHparameters::DHparameters(double _alpha, double _a, double _d, double _theta) {
  alpha = _alpha;
  a     = _a;
  d     = _d;
  theta = _theta;
}

DHparameters::~DHparameters() {
}

istream& 
operator>>(istream& _is, DHparameters& _d){
  return _is >> _d.alpha >> _d.a >> _d.d >> _d.theta;
}

ostream& 
operator<<(ostream& _os, const DHparameters& _d){
  return _os << _d.alpha << " " << _d.a << " " 
    << _d.d << " " << _d.theta << " ";
}

bool 
DHparameters::operator==(const DHparameters& dh) const {
  return alpha == dh.alpha && a == dh.a && d == dh.d && theta == dh.theta;
}

