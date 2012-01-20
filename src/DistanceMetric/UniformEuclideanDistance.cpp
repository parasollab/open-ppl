#include "UniformEuclideanDistance.h"

UniformEuclideanDistance::
UniformEuclideanDistance(bool _useRotational) : DistanceMetricMethod(), m_useRotational(_useRotational) {
  cout << "UniformEuclideanDistance::UniformEuclideanDistance() - useRotational = " << _useRotational << endl;
  m_name = "uniformEuclidean";
}

UniformEuclideanDistance::~UniformEuclideanDistance() {
}

void UniformEuclideanDistance::PrintOptions(ostream& _os) const {
  _os << GetName() << ":: ";
  _os << "useRotational = " << m_useRotational;
  _os << endl;
}



double UniformEuclideanDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double dist(0.0);
  double maxRange(0.0);
  for(int i=0; i< _c1.PosDOF(); ++i) {
    std::pair<double,double> range = _env->GetBoundingBox()->GetRange(i);
    double tmpRange = range.second-range.first;
    if(tmpRange > maxRange) maxRange = tmpRange;
  }
  for(int i=0; i<_c1.DOF(); ++i) {
    // calculate distance for positional coordinate
    if (i < _c1.PosDOF()) {
      double diff = (_c1.GetSingleParam(i) - _c2.GetSingleParam(i))/maxRange;
      if(m_useRotational)
        diff *= 2*PI;
      dist += diff * diff;
    }
    else {// calculate distance for rotational coordinate
      if (m_useRotational) {  // multiplying by 2*PI to make this like MPNN
        double diff1 = 2*PI*(_c1.GetSingleParam(i) - _c2.GetSingleParam(i));
        if (diff1 < 0) diff1 *= -1;
        double diff2 = 2*PI - diff1;
        double diff = (diff1 < diff2) ? diff1 : diff2;
        dist += diff * diff;
      }
      else {  // not multiplying by 2*PI to make this like CGAL
        double diff = _c1.GetSingleParam(i) - _c2.GetSingleParam(i);
        dist += diff * diff;
      }
    }
  }
  dist = sqrt(dist);
  return dist;
}
