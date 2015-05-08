#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))

#ifndef REACHABLE_DISTANCE_H_
#define REACHABLE_DISTANCE_H_

#include "DistanceMetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ReachableDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ReachableDistance(double _s1 = 0.33, double _s2 = 0.33);
    ReachableDistance(MPProblemType* _problem, XMLNode& _node, bool _warn = true);
    virtual ~ReachableDistance();

    virtual void Print(ostream& _os) const;

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);

  private:
    double m_scale1, m_scale2;
};

template<class MPTraits>
ReachableDistance<MPTraits>::
ReachableDistance(double _s1, double _s2) :
  DistanceMetricMethod<MPTraits>(), m_scale1(_s1), m_scale2(_s2)  {
    this->SetName("Reachable");
  }

template<class MPTraits>
ReachableDistance<MPTraits>::
ReachableDistance(MPProblemType* _problem, XMLNode& _node, bool _warn) :
  DistanceMetricMethod<MPTraits>(_problem, _node, false) {
    this->SetName("Reachable");

    m_scale1 = _node.Read("s1", false, 0.33, 0.0, 1.0, "position scale factor");
    m_scale2 = _node.Read("s2", false, 0.33, 0.0, 1.0, "link lengths scale factor");

    if(m_scale1 + m_scale2 > 1.0) {
      cerr << "\n\nERROR in ReachableDistance(): scale factor parameters must add up to less than or equal to 1: " << m_scale1 << " + " << m_scale2 << " = " << m_scale1 + m_scale2 << "\nexiting.\n";
      exit(-1);
    }

    if(_warn)
      _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
ReachableDistance<MPTraits>::
~ReachableDistance() {
}

template<class MPTraits>
void
ReachableDistance<MPTraits>::
Print(ostream& _os) const {
  DistanceMetricMethod<MPTraits>::Print(_os);
  _os << "\ts1 = " << m_scale1 << endl;
  _os << "\ts2 = " << m_scale2 << endl;
}

template<class MPTraits>
double
ReachableDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  //get the position difference
  double dPosition = 0.0;
  vector<double> v1 = _c1.GetData();
  vector<double> v2 = _c2.GetData();
  for(size_t i=0; i<_c1.PosDOF(); ++i) {
    pair<double,double> range = env->GetBoundary()->GetRange(i);
    dPosition += sqr(fabs(v1[i] - v2[i])/(range.second-range.first));
  }
  dPosition = sqrt(dPosition);

  //get the length difference
  double dLength = ((CfgType)_c1).LengthDistance((CfgType)_c2);

  //get the orientation difference
  double dOri = ((CfgType)_c1).OrientationDistance((CfgType)_c2);

  return (m_scale1*dPosition + m_scale2*dLength + (1-m_scale1-m_scale2)*dOri);
}

#endif

#endif
