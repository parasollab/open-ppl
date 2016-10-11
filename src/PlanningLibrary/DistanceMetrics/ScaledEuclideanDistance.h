#ifndef SCALED_EUCLIDEAN_DISTANCE_H_
#define SCALED_EUCLIDEAN_DISTANCE_H_

#include "EuclideanDistance.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief A Euclidean distance that supports non-uniform weighting of the
///        positional and rotational components.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ScaledEuclideanDistance : public EuclideanDistance<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType       CfgType;

    ///@}
    ///@name Construction
    ///@{

    ScaledEuclideanDistance();
    ScaledEuclideanDistance(MPProblemType* _problem, XMLNode& _node);
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

  private:

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
ScaledEuclideanDistance(MPProblemType* _problem, XMLNode& _node) :
    EuclideanDistance<MPTraits>(_problem, _node) {
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
  CfgType c = this->DifferenceCfg(_c1, _c2);
  return pow(m_scale * this->PositionDistance(c)
      + (1 - m_scale) * this->OrientationDistance(c), this->m_r3);
}

/*----------------------------------------------------------------------------*/

#endif
