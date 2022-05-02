#ifndef PMPL_SCALED_EUCLIDEAN_DISTANCE_H_
#define PMPL_SCALED_EUCLIDEAN_DISTANCE_H_

#include "EuclideanDistance.h"


////////////////////////////////////////////////////////////////////////////////
/// A Euclidean distance that supports non-uniform weighting of the positional
/// and rotational components. Joint components are lumped in with orientation.
///
/// @todo Separate the computation of orientation and joint distances.
///
/// @todo Replace with WeightedEuclidean, which full encompases this class and
///       offers more functionality.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ScaledEuclideanDistance : virtual public EuclideanDistance<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    ScaledEuclideanDistance();
    ScaledEuclideanDistance(XMLNode& _node);
    virtual ~ScaledEuclideanDistance() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    double m_scale{.5}; ///< The ratio of positional to rotational weighting.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ScaledEuclideanDistance<MPTraits>::
ScaledEuclideanDistance() : EuclideanDistance<MPTraits>() {
  this->SetName("ScaledEuclidean");
}


template <typename MPTraits>
ScaledEuclideanDistance<MPTraits>::
ScaledEuclideanDistance(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node) {
  this->SetName("ScaledEuclidean");
  m_scale = _node.Read("scale", false, m_scale, 0.0, 1.0, "scale factor");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
ScaledEuclideanDistance<MPTraits>::
Print(ostream& _os) const {
  EuclideanDistance<MPTraits>::Print(_os);
  _os << "\tscale = " << m_scale << endl;
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
ScaledEuclideanDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  const CfgType c = _c2 - _c1;
  
  return pow(m_scale * this->PositionDistance(c)
      + (1 - m_scale) * this->OrientationDistance(c), this->m_r3);
}

/*----------------------------------------------------------------------------*/

#endif
