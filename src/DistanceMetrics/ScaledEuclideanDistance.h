#ifndef SCALEDEUCLIDEANDISTANCE_H_
#define SCALEDEUCLIDEANDISTANCE_H_

#include "EuclideanDistance.h"

template<class MPTraits>
class ScaledEuclideanDistance : public EuclideanDistance<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    ScaledEuclideanDistance(double _scale = 0.5, bool _normalize = false);
    ScaledEuclideanDistance(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~ScaledEuclideanDistance();

    void PrintOptions(ostream& _os) const;

    virtual double Distance(Environment* _env, const CfgType& _c1, const CfgType& _c2);

  protected:
    double m_scale;
};

template<class MPTraits>
ScaledEuclideanDistance<MPTraits>::ScaledEuclideanDistance(double _scale, bool _normalize) :
  EuclideanDistance<MPTraits>(_normalize), m_scale(0.5) {
  this->m_name = "ScaledEuclidean";
}

template<class MPTraits>
ScaledEuclideanDistance<MPTraits>::ScaledEuclideanDistance(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) :
  EuclideanDistance<MPTraits>(_problem, _node, false) {
    this->m_name = "ScaledEuclidean";
    m_scale = _node.numberXMLParameter("scale", false, 0.5, 0.0, 1.0, "scale factor");

    _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
ScaledEuclideanDistance<MPTraits>::~ScaledEuclideanDistance() {}

//Print scaled distance
template<class MPTraits>
void
ScaledEuclideanDistance<MPTraits>::PrintOptions(ostream& _os) const {
  EuclideanDistance<MPTraits>::PrintOptions(_os);
  _os << " scale=" << m_scale;
}

//Calculating the scaled distance (result)
template<class MPTraits>
double
ScaledEuclideanDistance<MPTraits>::Distance(Environment* _env, const CfgType& _c1, const CfgType& _c2) {
  CfgType c = this->DifferenceCfg(_c1, _c2);
  double result = pow(m_scale*this->PositionDistance(_env, c) + (1-m_scale)*this->OrientationDistance(c), this->m_r3);
  return result;
}

#endif

