#ifndef SCALED_EUCLIDEAN_DISTANCE_H_
#define SCALED_EUCLIDEAN_DISTANCE_H_

#include "EuclideanDistance.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ScaledEuclideanDistance : public EuclideanDistance<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ScaledEuclideanDistance(double _scale = 0.5, bool _normalize = false);
    ScaledEuclideanDistance(MPProblemType* _problem, XMLNode& _node);
    virtual ~ScaledEuclideanDistance();

    virtual void Print(ostream& _os) const;

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);

  private:
    double m_scale;
};

template<class MPTraits>
ScaledEuclideanDistance<MPTraits>::
ScaledEuclideanDistance(double _scale, bool _normalize) :
  EuclideanDistance<MPTraits>(_normalize), m_scale(0.5) {
    this->SetName("ScaledEuclidean");
  }

template<class MPTraits>
ScaledEuclideanDistance<MPTraits>::
ScaledEuclideanDistance(MPProblemType* _problem, XMLNode& _node) :
  EuclideanDistance<MPTraits>(_problem, _node) {
    this->SetName("ScaledEuclidean");
    m_scale = _node.Read("scale", false, 0.5, 0.0, 1.0, "scale factor");
  }

template<class MPTraits>
ScaledEuclideanDistance<MPTraits>::
~ScaledEuclideanDistance() {
}

template<class MPTraits>
void
ScaledEuclideanDistance<MPTraits>::
Print(ostream& _os) const {
  EuclideanDistance<MPTraits>::Print(_os);
  _os << "\tscale = " << m_scale << endl;
}

template<class MPTraits>
double
ScaledEuclideanDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  CfgType c = this->DifferenceCfg(_c1, _c2);
  return pow(m_scale*this->PositionDistance(c)
      + (1-m_scale)*this->OrientationDistance(c), this->m_r3);
}

#endif
