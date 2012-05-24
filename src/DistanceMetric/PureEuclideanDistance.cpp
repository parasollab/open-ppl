#include "PureEuclideanDistance.h"

PureEuclideanDistance::PureEuclideanDistance() : DistanceMetricMethod() {
  m_name = "pureEuclidean";
}
 
PureEuclideanDistance::~PureEuclideanDistance() {
}

void PureEuclideanDistance::PrintOptions(ostream& _os) const {
  _os << GetName();
  _os << endl;
}

double PureEuclideanDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double dist(0.0);
  for(size_t i=0; i<_c1.DOF(); ++i) {
      double diff = _c1.GetSingleParam(i) - _c2.GetSingleParam(i);
      dist += diff * diff;
  }
  dist = sqrt(dist);
  return dist;
}
