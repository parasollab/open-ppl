#ifndef KNOT_THEORY_DISTANCE_H_
#define KNOT_THEORY_DISTANCE_H_

#include "DistanceMetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief Computes the knot theory distance between two cfgs, taking the
///        topological information into perspective.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class KnotTheoryDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    KnotTheoryDistance();
    KnotTheoryDistance(XMLNode& _node);
    virtual ~KnotTheoryDistance() = default;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);

    ///@}

  private:

    ///@name Helpers
    ///@{

    virtual vector<Vector3d> GetCoordinatesForKnot(const CfgType& _c);

    double Knot(vector<Vector3d>& _c1, vector<Vector3d>& _c2);

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
KnotTheoryDistance<MPTraits>::
KnotTheoryDistance() : DistanceMetricMethod<MPTraits>() {
  this->SetName("KnotTheory");
}


template <typename MPTraits>
KnotTheoryDistance<MPTraits>::
KnotTheoryDistance(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node) {
  this->SetName("KnotTheory");
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
KnotTheoryDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  vector<Vector3d> c1 = GetCoordinatesForKnot(_c1);
  vector<Vector3d> c2 = GetCoordinatesForKnot(_c2);
  return Knot(c1, c2);
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
vector<Vector3d>
KnotTheoryDistance<MPTraits>::
GetCoordinatesForKnot(const CfgType& _c) {
  _c.ConfigureRobot();
  vector<Vector3d> coordinates;
  for(size_t i = 0; i < _c.GetRobot()->NumFreeBody(); ++i)
    coordinates.push_back(_c.GetRobot()->GetFreeBody(i)->GetWorldTransformation().
        translation());
  return coordinates;
}


template <typename MPTraits>
double
KnotTheoryDistance<MPTraits>::
Knot(vector<Vector3d>& _c1, vector<Vector3d>& _c2) {
  if(_c1.size() < 2) {
    cerr << "\n\nError in KnotTheoryDistance::Distance(), _c1 has too few "
         << "links, exiting.\n";
    exit(1);
  }

  vector<Vector3d> unitVect(_c1.size()), unitVect2(_c2.size());
  double sum = 0, sign = 0, signsum = 0;
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

/*----------------------------------------------------------------------------*/

#endif
