#ifndef MINKOWSKIDISTANCE_H_
#define MINKOWSKIDISTANCE_H_

#include "DistanceMetricMethod.h"
#include "MPProblem/IsClosedChain.h"
#include "MPProblem/Environment.h"

template<class MPTraits>
class MinkowskiDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    
    MinkowskiDistance(double _r1 = 3, double _r2 = 3, double _r3 = 1.0/3, bool _normalize = false);
    MinkowskiDistance(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node, bool _warn = true, bool _parse = true);
    virtual ~MinkowskiDistance();
    
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const CfgType& _c1, const CfgType& _c2);
    virtual void ScaleCfg(Environment* _env, double _length, CfgType& _o, CfgType& _c, bool _normalizeOrientation = true);

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

    double PositionDistance(Environment* _env, const CfgType& _c);
    double OrientationDistance(const CfgType& _c);

    /**Power factors for Minkowski Distance **/
    double m_r1; ///<For position part.
    double m_r2; ///<For rotation part.
    double m_r3; ///<For calculating root.
    bool m_normalize;
};

template<class MPTraits>
MinkowskiDistance<MPTraits>::MinkowskiDistance(double _r1, double _r2, double _r3, bool _normalize) 
  : DistanceMetricMethod<MPTraits>(), m_r1(_r1), m_r2(_r2), m_r3(_r3), m_normalize(_normalize) {
    this->m_name = "Minkowski";
  }

template<class MPTraits>
MinkowskiDistance<MPTraits>::MinkowskiDistance(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node, bool _warn, bool _parse) 
  : DistanceMetricMethod<MPTraits>(_problem, _node, false) {
    this->m_name = "Minkowski";

    if(_parse){
      m_r1 = _node.numberXMLParameter("r1", false, 3.0, 0.0, 1000.0, "r1");
      m_r2 = _node.numberXMLParameter("r2", false, 3.0, 0.0, 1000.0, "r2");
      m_r3 = _node.numberXMLParameter("r3", false, 1.0/3.0, 0.0, 1000.0, "r3");
      m_normalize = _node.boolXMLParameter("normalize", false, false, "flag if position dof should be normalized by environment diagonal");
    }

    if(_warn)
      _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
MinkowskiDistance<MPTraits>::~MinkowskiDistance() {}

template<class MPTraits>
void
MinkowskiDistance<MPTraits>::PrintOptions(ostream& _os) const {
  DistanceMetricMethod<MPTraits>::PrintOptions(_os);
  _os << "r1=" << m_r1 << " r2=" << m_r2 << " r3=" << m_r3 << " normalize=" << m_normalize << endl;
}

template<class MPTraits>
double
MinkowskiDistance<MPTraits>::Distance(Environment* _env, const CfgType& _c1, const CfgType& _c2) {
  CfgType diff = DifferenceCfg(_c1, _c2);
  double pos = PositionDistance(_env, diff);
  double orient = OrientationDistance(diff);
  return pow(pos+orient, m_r3);
}

template<class MPTraits>
void
MinkowskiDistance<MPTraits>::ScaleCfg(Environment* _env, double _length, CfgType& _o, CfgType& _c, bool _normalizeOrientation) {
  double originalLength = this->Distance(_env, _o, _c);
  double diff = _length - originalLength;
  do {
    _c *= _length/originalLength;
    originalLength = this->Distance(_env, _o, _c);
    diff = _length - originalLength;
  } while((diff > 0.1) || (diff < -0.1));
}

template<class MPTraits>
double
MinkowskiDistance<MPTraits>::PositionDistance(Environment* _env, const CfgType& _c) {
  double diagonal = 0;
  for(size_t i=0; i<_c.PosDOF(); ++i) {
    std::pair<double,double> range = _env->GetBoundary()->GetRange(i);
    diagonal += pow(fabs(range.second-range.first), m_r1);
  }
  diagonal = pow(diagonal, m_r3);

  vector<double> p = _c.GetPosition();
  double pos = 0;
  for(size_t i=0; i<p.size(); ++i) 
    if(m_normalize)
      pos += pow(fabs(p[i])/diagonal, m_r1);
    else
      pos += pow(fabs(p[i]), m_r1);
  return pos; 
}

template<class MPTraits>
double
MinkowskiDistance<MPTraits>::OrientationDistance(const CfgType& _c) {
  vector<double> o = _c.GetOrientation();
  double orient = 0;
  for(size_t i=0; i<o.size(); ++i) 
    orient += pow(fabs(o[i]), m_r2);
  return orient;
}

#endif
