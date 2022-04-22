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
class KnotTheoryDistance : virtual public DistanceMetricMethod<MPTraits> {

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
  // ensure cfg contains more than one body
  if(_c1.size() < 2)
    throw RunTimeException(WHERE) << "_c1 has too few links (" << _c1.size()
                                  << ")";

  std::vector<Vector3d> unitVect(_c1.size()), unitVect2(_c2.size());
  double sum = 0, sign = 0, signsum = 0;
  // for each body (except the last)
  for(size_t i = 0; i < _c1.size()-1; ++i) {
    // if this body and the next are different
    if((_c1[i+1]-_c1[i]).norm() != 0)
      // set unitVect[i] sum of this body and next, scaled by the magnitude of 
      // the difference between this body and the next
      unitVect[i] = (_c1[i+1]+_c1[i])/(_c1[i+1]-_c1[i]).norm();
    else
      // else use the 0 vector
      unitVect[i](0, 0, 0);
    // repeat for second config
    if((_c2[i+1]-_c2[i]).norm() !=0)
      unitVect2[i] = (_c2[i+1]+_c2[i])/(_c2[i+1]-_c2[i]).norm();
    else
      unitVect2[i](0, 0, 0);
    // take cross product of two 'unit' vectors and norm
    // TODO: figure out why this is here
    double area = (unitVect[i] % unitVect2[i]).norm();
    sum += area;
    // add dot product between two vectors to signsum
    sign = unitVect2[i] * unitVect[i];
    signsum += sign;
  }

  return signsum;
}

/*----------------------------------------------------------------------------*/

#endif
