#ifndef DISTANCEMETRICMETHOD_H
#define DISTANCEMETRICMETHOD_H

const double MAX_DIST =  1e10;

/**This is the interface for all distance metric methods(euclidean,
 *scaledEuclidean, minkowski, manhattan, com, etc.).
 */
template<class MPTraits>
class DistanceMetricMethod  : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    DistanceMetricMethod();
    DistanceMetricMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node, bool _warn = true);
    virtual ~DistanceMetricMethod();

    virtual void PrintOptions(ostream& _os) const;

    virtual double Distance(Environment* _env, const CfgType& _c1, const CfgType& _c2) = 0;
    virtual void ScaleCfg(Environment* _env, double _length, CfgType& _o, CfgType& _c, bool _norm=true);
};

template<class MPTraits>
DistanceMetricMethod<MPTraits>::DistanceMetricMethod() {}

template<class MPTraits>
DistanceMetricMethod<MPTraits>::DistanceMetricMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node, bool _warn) : MPBaseObject<MPTraits>(_problem, _node) {
  if(_warn)
    _node.warnUnrequestedAttributes();
}

template<class MPTraits>
DistanceMetricMethod<MPTraits>::~DistanceMetricMethod() {
}

template<class MPTraits>
void
DistanceMetricMethod<MPTraits>::PrintOptions(ostream& _os) const {
  _os << "    " << this->GetName() << "::  ";
}

template<class MPTraits>
void
DistanceMetricMethod<MPTraits>::ScaleCfg(Environment* _env, double _length, CfgType& _o, CfgType& _c, bool _norm) {
  _length = fabs(_length); //a distance must be positive
  CfgType origin = _o;
  CfgType outsideCfg = _c;
  // first find an outsite configuration with sufficient size
  while(Distance(_env, origin, outsideCfg) < 2*_length)
    for(size_t i=0; i<outsideCfg.DOF(); ++i)
      outsideCfg[i] *= 2.0;
  // now, using binary search  find a configuration with the approximate length
  CfgType aboveCfg = outsideCfg;
  CfgType belowCfg = origin;
  CfgType currentCfg = _c;
  while (1) {
    for(size_t i=0; i<currentCfg.DOF(); ++i)
      currentCfg[i] = (aboveCfg[i] + belowCfg[i]) / 2.0;
    double magnitude = Distance(_env, origin, currentCfg);
    if( (magnitude >= _length*0.9) && (magnitude <= _length*1.1))
      break;
    if(magnitude>_length)
      aboveCfg = currentCfg;
    else
      belowCfg = currentCfg;
  }

  _c = currentCfg;
}

#endif
