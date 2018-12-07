#ifndef PMPL_KNOT_THEORY_DISTANCE_H_
#define PMPL_KNOT_THEORY_DISTANCE_H_

#include "DistanceMetricMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Computes the knot theory distance between two cfgs, taking the topological
/// information into perspective.
///
/// @todo Document with a meaningful description and paper reference. Knot
///       function also needs cleanup for clarity and avoiding recomputation.
///
/// @ingroup DistanceMetrics
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

    virtual std::vector<Vector3d> GetCoordinatesForKnot(const CfgType& _c);

    double Knot(std::vector<Vector3d>& _c1, std::vector<Vector3d>& _c2);

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
  auto c1 = GetCoordinatesForKnot(_c1),
       c2 = GetCoordinatesForKnot(_c2);
  return Knot(c1, c2);
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
std::vector<Vector3d>
KnotTheoryDistance<MPTraits>::
GetCoordinatesForKnot(const CfgType& _c) {
  _c.ConfigureRobot();
  std::vector<Vector3d> coordinates;
  for(size_t i = 0; i < _c.GetMultiBody()->GetNumBodies(); ++i)
    coordinates.push_back(_c.GetMultiBody()->GetBody(i)->
        GetWorldTransformation().translation());
  return coordinates;
}


template <typename MPTraits>
double
KnotTheoryDistance<MPTraits>::
Knot(std::vector<Vector3d>& _c1, std::vector<Vector3d>& _c2) {
  if(_c1.size() < 2)
    throw RunTimeException(WHERE) << "_c1 has too few links (" << _c1.size()
                                  << ")";

  std::vector<Vector3d> unitVect(_c1.size()), unitVect2(_c2.size());
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
