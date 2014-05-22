#ifndef SCALEDEUCLIDEANDISTANCE_H_
#define SCALEDEUCLIDEANDISTANCE_H_

#include "EuclideanDistance.h"

template<class MPTraits>
class ScaledEuclideanDistance : public EuclideanDistance<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ScaledEuclideanDistance(double _scale = 0.5, bool _normalize = false);
    ScaledEuclideanDistance(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~ScaledEuclideanDistance();

    virtual void Print(ostream& _os) const;

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);

  private:
    double m_scale;
};

template<class MPTraits>
ScaledEuclideanDistance<MPTraits>::ScaledEuclideanDistance(double _scale,
    bool _normalize) : EuclideanDistance<MPTraits>(_normalize), m_scale(0.5) {
  this->SetName("ScaledEuclidean");
}

template<class MPTraits>
ScaledEuclideanDistance<MPTraits>::ScaledEuclideanDistance(MPProblemType* _problem,
    XMLNodeReader& _node) : EuclideanDistance<MPTraits>(_problem, _node, false) {
  this->SetName("ScaledEuclidean");
  m_scale = _node.numberXMLParameter("scale", false, 0.5, 0.0, 1.0, "scale factor");

  _node.warnUnrequestedAttributes();
}

template<class MPTraits>
ScaledEuclideanDistance<MPTraits>::~ScaledEuclideanDistance() {
}

template<class MPTraits>
void
ScaledEuclideanDistance<MPTraits>::Print(ostream& _os) const {
  EuclideanDistance<MPTraits>::Print(_os);
  _os << "\tscale = " << m_scale << endl;
}

template<class MPTraits>
double
ScaledEuclideanDistance<MPTraits>::Distance(const CfgType& _c1, const CfgType& _c2) {
  CfgType c = this->DifferenceCfg(_c1, _c2);
  return pow(m_scale*this->PositionDistance(c)
      + (1-m_scale)*this->OrientationDistance(c), this->m_r3);
}

#endif
