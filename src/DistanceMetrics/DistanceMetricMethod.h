#ifndef DISTANCE_METRIC_METHOD_H
#define DISTANCE_METRIC_METHOD_H

#include "MPProblem/MPBaseObject.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief Base algorithm abstraction for \ref DistanceMetrics.
/// @tparam MPTraits Motion planning universe
///
/// DistanceMetricMethod has two important methods: @c Distance and @c ScaleCfg.
///
/// @c Distance takes as input two configurations \f$c_1\f$ and \f$c_2\f$ and
/// returns the computed transition distance between them.
///
/// @c ScaleCfg is purposed to scale a \f$d\f$-dimensional ray in @cspace to a
/// certain magnitude based upon a general @dm.
///
/// @note we assume that for asymmetric distance metrics, we are checking the
/// distance from the first cfg to the second
///
/// @note in all cases we assume that using distance = infinity means that the
/// second cfg is not reachable from the first
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class DistanceMetricMethod  : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    DistanceMetricMethod() {}
    DistanceMetricMethod(MPProblemType* _problem, XMLNode& _node);
    virtual ~DistanceMetricMethod() {}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute a distance between two configurations
    /// @param _c1 Configuration 1
    /// @param _c2 Configuration 2
    /// @return Distance value
    ///
    /// @usage
    /// @code
    /// DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
    /// CfgType c1, c2;
    /// double dist = dm->Distance(c1, c2);
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    virtual double Distance(const CfgType& _c1, const CfgType& _c2) = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Scale a directional configuration to a certain magnitude
    /// @param _length Desired magnitude
    /// @param _c Configuration to be scaled
    /// @param _o Configuration to scale upon (origin of scaling)
    /// @return Distance value
    ///
    /// @usage
    /// @code
    /// DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
    /// CfgType ray, origin;
    /// double length;
    /// dm->ScaleCfg(length, ray, origin);
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    virtual void ScaleCfg(double _length, CfgType& _c, const CfgType& _o = CfgType());
};

template<class MPTraits>
DistanceMetricMethod<MPTraits>::
DistanceMetricMethod(MPProblemType* _problem, XMLNode& _node)
  : MPBaseObject<MPTraits>(_problem, _node) {
}

template<class MPTraits>
void
DistanceMetricMethod<MPTraits>::
ScaleCfg(double _length, CfgType& _c, const CfgType& _o) {
  _length = fabs(_length); //a distance must be positive
  CfgType origin = _o;
  CfgType outsideCfg = _c;
  // first find an outsite configuration with sufficient size
  while(Distance(origin, outsideCfg) < 2*_length)
    for(size_t i=0; i<outsideCfg.DOF(); ++i)
      outsideCfg[i] *= 2.0;
  // now, using binary search find a configuration with the approximate length
  CfgType aboveCfg = outsideCfg;
  CfgType belowCfg = origin;
  CfgType currentCfg = _c;
  while (1) {
    for(size_t i=0; i<currentCfg.DOF(); ++i)
      currentCfg[i] = (aboveCfg[i] + belowCfg[i]) / 2.0;
    double magnitude = Distance(origin, currentCfg);
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
