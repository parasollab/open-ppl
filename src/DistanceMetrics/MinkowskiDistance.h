#ifndef MINKOWSKI_DISTANCE_H_
#define MINKOWSKI_DISTANCE_H_

#include "DistanceMetricMethod.h"

#include "Environment/Environment.h"
#include "MPProblem/IsClosedChain.h"

#include <boost/utility/enable_if.hpp>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MinkowskiDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    MinkowskiDistance(double _r1 = 3, double _r2 = 3, double _r3 = 1.0/3,
        bool _normalize = false);
    MinkowskiDistance(MPProblemType* _problem, XMLNode& _node,
        bool _parse = true);
    virtual ~MinkowskiDistance();

    virtual void Print(ostream& _os) const;

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);
    virtual void ScaleCfg(double _length, CfgType& _c, const CfgType& _o = CfgType());
    static double PositionDistance(Environment *_env, const CfgType& _c, double _r1, double _r3, bool _normalize);

  protected:
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
        /*vector<double> _v1 = _c1.GetData();
          vector<double> _v2 = _c2.GetData();
          if(_v1.size() != _v2.size()) {
          cout << "ERROR in MinkowskiDistance::DifferenceCfgType, _c1 dofs (" << _v1.size() << ") != _c2 dofs (" << _v2.size() << ")\n";
          exit(-1);
          }
          if(_v1.size() == MPTraits::CfgTypeType::GetNumOfJoints()) {
          CfgType_fixed_tree c1Linkage;
          c1Linkage.SetData(_v1);
          CfgType_fixed_tree c2Linkage;
          c2Linkage.SetData(_v2);
          CfgType* c = c1Linkage.CreateNewCfg();
          c->subtract(c1Linkage, c2Linkage);
          return c;
          } else {
          CfgType_free_tree c1Linkage;
          c1Linkage.SetData(_v1);
          CfgType_free_tree c2Linkage;
          c2Linkage.SetData(_v2);
          CfgType* c = c1Linkage.CreateNewCfg();
          c->subtract(c1Linkage, c2Linkage);
          return c;
          }*/
      }

    double PositionDistance(const CfgType& _c);
    double OrientationDistance(const CfgType& _c);

    //Power factors for Minkowski Distance
    double m_r1; ///< For position part.
    double m_r2; ///< For rotation part.
    double m_r3; ///< For calculating root.
    bool m_normalize;
};

template<class MPTraits>
MinkowskiDistance<MPTraits>::
MinkowskiDistance(double _r1, double _r2, double _r3, bool _normalize)
  : DistanceMetricMethod<MPTraits>(), m_r1(_r1), m_r2(_r2), m_r3(_r3), m_normalize(_normalize) {
    this->SetName("Minkowski");
  }

template<class MPTraits>
MinkowskiDistance<MPTraits>::
MinkowskiDistance(MPProblemType* _problem, XMLNode& _node, bool _parse) :
  DistanceMetricMethod<MPTraits>(_problem, _node),
  m_r1(3), m_r2(3), m_r3(1./3.), m_normalize(false) {
    this->SetName("Minkowski");

    if(_parse) {
      m_r1 = _node.Read<double>("r1", false, 3.0, 0.0, MAX_DBL, "r1");
      m_r2 = _node.Read("r2", false, 3.0, 0.0, MAX_DBL, "r2");
      m_r3 = _node.Read("r3", false, 1.0/3.0, 0.0, MAX_DBL, "r3");
      m_normalize = _node.Read("normalize", false, false, "flag if position dof should be normalized by environment diagonal");
    }
  }

template<class MPTraits>
MinkowskiDistance<MPTraits>::
~MinkowskiDistance() {
}

template<class MPTraits>
void
MinkowskiDistance<MPTraits>::
Print(ostream& _os) const {
  DistanceMetricMethod<MPTraits>::Print(_os);
  _os << "\tr1 = " << m_r1 << endl;
  _os << "\tr2 = " << m_r2 << endl;
  _os << "\tr3 = " << m_r3 << endl;
  _os << "\tnormalize = " << m_normalize << endl;
}

template<class MPTraits>
double
MinkowskiDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  CfgType diff = DifferenceCfg(_c1, _c2);
  double pos = PositionDistance(diff);
  double orient = OrientationDistance(diff);
  return pow(pos+orient, m_r3);
}

template<class MPTraits>
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

template<class MPTraits>
double
MinkowskiDistance<MPTraits>::PositionDistance(const CfgType& _c) {
  return PositionDistance(this->GetEnvironment(),_c,m_r1,m_r3,m_normalize);
}

template<class MPTraits>
double
MinkowskiDistance<MPTraits>::PositionDistance(Environment *_env, const CfgType& _c, double _r1, double _r3, bool _normalize) {
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

template<class MPTraits>
double
MinkowskiDistance<MPTraits>::
OrientationDistance(const CfgType& _c) {
  vector<double> o = _c.GetOrientation();
  double orient = 0;
  for(size_t i=0; i<o.size(); ++i)
    orient += pow(fabs(o[i]), m_r2);
  return orient;
}

#endif
