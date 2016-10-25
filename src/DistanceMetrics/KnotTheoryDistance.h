#ifndef KNOT_THEORY_DISTANCE_H_
#define KNOT_THEORY_DISTANCE_H_

#include "DistanceMetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// This computes the knot theory distance between two cfgs.
/// Taking the topological information into perspective.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class KnotTheoryDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    KnotTheoryDistance();
    KnotTheoryDistance(MPProblemType* _problem, XMLNode& _node);
    virtual ~KnotTheoryDistance();

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);

  private:
    virtual vector<Vector3d> GetCoordinatesForKnot(const CfgType& _c);
    double Knot(vector<Vector3d>& _c1, vector<Vector3d>& _c2);
};

template<class MPTraits>
KnotTheoryDistance<MPTraits>::
KnotTheoryDistance() : DistanceMetricMethod<MPTraits>() {
  this->SetName("KnotTheory");
}

template<class MPTraits>
KnotTheoryDistance<MPTraits>::
KnotTheoryDistance(MPProblemType* _problem, XMLNode& _node) :
  DistanceMetricMethod<MPTraits>(_problem, _node) {
    this->SetName("KnotTheory");
  }

template<class MPTraits>
KnotTheoryDistance<MPTraits>::
~KnotTheoryDistance() {
}

template<class MPTraits>
double
KnotTheoryDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  vector<Vector3d> c1 = GetCoordinatesForKnot(_c1);
  vector<Vector3d> c2 = GetCoordinatesForKnot(_c2);
  return Knot(c1, c2);
}

template<class MPTraits>
vector<Vector3d>
KnotTheoryDistance<MPTraits>::
GetCoordinatesForKnot(const CfgType& _c) {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  _c.ConfigureRobot();
  vector<Vector3d> coordinates;
  for(size_t i=0; i< env->GetRobot(_c.GetRobotIndex())->NumFreeBody(); ++i)
    coordinates.push_back(env->GetRobot(_c.GetRobotIndex())->GetFreeBody(i)
        ->WorldTransformation().translation());
  return coordinates;
}

template<class MPTraits>
double
KnotTheoryDistance<MPTraits>::
Knot(vector<Vector3d>& _c1, vector<Vector3d>& _c2) {
  if (_c1.empty() || _c1.size()<2) {
    cerr << "\n\nError in KnotTheoryDistance::Distance(), _c1 has too few links, exiting.\n";
    exit(1);
  }

  vector<Vector3d> unitVect(_c1.size()), unitVect2(_c2.size());
  double sum = 0.0, sign = 0, signsum=0;
  for(size_t i = 0; i < _c1.size()-1; ++i) {

    if((_c1[i+1]-_c1[i]).norm() != 0)
      unitVect[i] = (_c1[i+1]+_c1[i])/(_c1[i+1]-_c1[i]).norm();
    else
      unitVect[i](0, 0, 0);

    if((_c2[i+1]-_c2[i]).norm() !=0)
      unitVect2[i] = (_c2[i+1]+_c2[i])/(_c2[i+1]-_c2[i]).norm();
    else
      unitVect2[i](0, 0, 0);
    double area = (unitVect[i] % unitVect2[i]).norm();
    sign = unitVect2[i] * unitVect[i];
    sum += area;
    signsum += sign;
  }

  return signsum;
}

#endif
