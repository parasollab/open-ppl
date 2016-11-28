#ifndef MINKOWSKI_DISTANCE_H_
#define MINKOWSKI_DISTANCE_H_

#include "DistanceMetricMethod.h"

#include "MPProblem/Environment/Environment.h"
#include "MPProblem/IsClosedChain.h"

#include <boost/utility/enable_if.hpp>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MinkowskiDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    MinkowskiDistance(double _r1 = 3, double _r2 = 3, double _r3 = 1. / 3,
        bool _normalize = false);
    MinkowskiDistance(XMLNode& _node);
    virtual ~MinkowskiDistance() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    virtual void ScaleCfg(double _length, CfgType& _c,
        const CfgType& _o = CfgType()) override;

    static double PositionDistance(Environment *_env, const CfgType& _c,
        double _r1, double _r3, bool _normalize);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    //default implementation
    template<typename EnableCfg>
    EnableCfg DifferenceCfg(const EnableCfg& _c1, const EnableCfg& _c2,
        typename boost::disable_if<IsClosedChain<EnableCfg> >::type* _dummy = 0) {
      return _c1 - _c2;
    }

    //reachable distance implementation
    template<typename EnableCfg>
    EnableCfg DifferenceCfg(const EnableCfg& _c1, const EnableCfg& _c2,
        typename boost::enable_if<IsClosedChain<EnableCfg> >::type* _dummy = 0) {
      cerr << "Error::DistanceMetrics for ReachableDistance disabled." << endl;
      exit(1);
    }

    double PositionDistance(const CfgType& _c);
    double OrientationDistance(const CfgType& _c);

    ///@}
    ///@name Internal State
    ///@{

    double m_r1{3};          ///< For position part.
    double m_r2{3};          ///< For rotation part.
    double m_r3{1. / 3};     ///< For calculating root.
    bool m_normalize{false}; ///< Normalize distance w.r.t. environment?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
MinkowskiDistance<MPTraits>::
MinkowskiDistance(double _r1, double _r2, double _r3, bool _normalize) :
    DistanceMetricMethod<MPTraits>(), m_r1(_r1), m_r2(_r2), m_r3(_r3),
    m_normalize(_normalize) {
  this->SetName("Minkowski");
}


template <typename MPTraits>
MinkowskiDistance<MPTraits>::
MinkowskiDistance(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
    m_r1(3), m_r2(3), m_r3(1./3.), m_normalize(false) {
  this->SetName("Minkowski");

  m_r1 = _node.Read<double>("r1", false, 3.0, 0.0, MAX_DBL, "r1");
  m_r2 = _node.Read("r2", false, 3.0, 0.0, MAX_DBL, "r2");
  m_r3 = _node.Read("r3", false, 1.0/3.0, 0.0, MAX_DBL, "r3");
  m_normalize = _node.Read("normalize", false, false, "flag if position dof "
      "should be normalized by environment diagonal");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
MinkowskiDistance<MPTraits>::
Print(ostream& _os) const {
  DistanceMetricMethod<MPTraits>::Print(_os);
  _os << "\tr1 = " << m_r1 << endl;
  _os << "\tr2 = " << m_r2 << endl;
  _os << "\tr3 = " << m_r3 << endl;
  _os << "\tnormalize = " << m_normalize << endl;
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  CfgType diff = DifferenceCfg(_c1, _c2);
  double pos = PositionDistance(diff);
  double orient = OrientationDistance(diff);
  return pow(pos+orient, m_r3);
}


template <typename MPTraits>
void
MinkowskiDistance<MPTraits>::
ScaleCfg(double _length, CfgType& _c, const CfgType& _o) {
  double originalLength = this->Distance(_o, _c);
  double diff = _length - originalLength;
  do {
    _c = (_c - _o) * (_length / originalLength) + _o;
    originalLength = this->Distance(_o, _c);
    diff = _length - originalLength;
  } while((diff > 0.1) || (diff < -0.1));
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
PositionDistance(const CfgType& _c) {
  return PositionDistance(this->GetEnvironment(),_c,m_r1,m_r3,m_normalize);
}


template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
PositionDistance(Environment *_env,
    const CfgType& _c, double _r1, double _r3, bool _normalize) {
  double diagonal = _env->GetBoundary()->GetMaxDist(_r1, _r3);
  vector<double> p = _c.GetPosition();
  double pos = 0;
  for(size_t i=0; i<p.size(); ++i)
    if(_normalize)
      pos += pow(fabs(p[i])/diagonal, _r1);
    else
      pos += pow(fabs(p[i]), _r1);
  return pos;
}


template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
OrientationDistance(const CfgType& _c) {
  vector<double> o = _c.GetOrientation();
  double orient = 0;
  for(size_t i=0; i<o.size(); ++i)
    orient += pow(fabs(o[i]), m_r2);
  return orient;
}

/*----------------------------------------------------------------------------*/

#endif
