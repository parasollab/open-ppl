#ifndef PMPL_DISTANCE_METRIC_METHOD_H
#define PMPL_DISTANCE_METRIC_METHOD_H

#include "MPLibrary/MPBaseObject.h"


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref DistanceMetrics.
///
/// DistanceMetricMethod has two important methods: @c Distance and @c ScaleCfg.
///
/// @c Distance takes as input two configurations \f$c_1\f$ and \f$c_2\f$ and
/// returns the computed transition distance between them.
/// @usage
/// @code
/// auto dm = this->GetDistanceMetric(m_dmLabel);
/// CfgType c1, c2;
/// double dist = dm->Distance(c1, c2);
/// @endcode
///
/// @c ScaleCfg is purposed to scale a \f$d\f$-dimensional ray in @cspace to a
/// certain magnitude based upon a general @dm.
/// @usage
/// @code
/// auto dm = this->GetDistanceMetric(m_dmLabel);
/// CfgType ray, origin;
/// double length;
/// dm->ScaleCfg(length, ray, origin);
/// @endcode
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DistanceMetricMethod  : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::RoadmapType   RoadmapType;
    typedef typename RoadmapType::VID        VID;
    typedef typename MPTraits::GroupCfgType  GroupCfgType;
    typedef typename GroupCfgType::Formation Formation;

    ///@}
    ///@name Construction
    ///@{

    DistanceMetricMethod() = default;
    DistanceMetricMethod(XMLNode& _node);
    virtual ~DistanceMetricMethod() = default;

    ///@}
    ///@name Distance Interface
    ///@{

    /// Compute a distance between two configurations.
    /// @param _c1 The first configuration.
    /// @param _c2 The second configuration.
    /// @return The computed distance between _c1 and _c2.
    virtual double Distance(const CfgType& _c1, const CfgType& _c2) = 0;

    /// This version is for group configurations. The default implementation
    /// returns the summed individual distances.
    /// @overload
    virtual double Distance(const GroupCfgType& _c1, const GroupCfgType& _c2);

    /// Compute the weight of an existing roadmap edge according to this metric.
    /// @param _source The source vertex descriptor.
    /// @param _target The target vertex descriptor.
    /// @return The total edge weight going through each intermediate of the
    ///         edge (_source, _target), or infinity if the edge does not exist.
    /// @todo We need to make this work for group roadmaps. Probably it needs to
    ///       take the roadmap as a templated parameter and not be virtual
    ///       (there is really only one way to do this).
    virtual double EdgeWeight(const VID _source, const VID _target);

    ///@}
    ///@name Configuration Scaling
    ///@{
    /// These functions rescale a configuration vector based on this distance
    /// metric.

    /// Scale a directional configuration to a certain magnitude.
    /// @param _length Desired magnitude.
    /// @param _c Configuration to be scaled.
    /// @param _o Origin of scaling.
    virtual void ScaleCfg(double _length, CfgType& _c, const CfgType& _o);

    /// This vesion uses the default origin.
    /// @overload
    void ScaleCfg(double _length, CfgType& _c);

    /// This version is for group configurations.
    /// @overload
    virtual void ScaleCfg(double _length, GroupCfgType& _c,
        const GroupCfgType& _o);

    /// This version is for group configurations with default origin.
    /// @overload
    void ScaleCfg(double _length, GroupCfgType& _c);

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
DistanceMetricMethod<MPTraits>::
DistanceMetricMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
DistanceMetricMethod<MPTraits>::
Distance(const GroupCfgType& _c1, const GroupCfgType& _c2) {
  double sum = 0;
  for(size_t i = 0; i < _c1.GetNumRobots(); ++i)
    sum += Distance(_c1.GetRobotCfg(i), _c2.GetRobotCfg(i));

  return sum;
}


template <typename MPTraits>
double
DistanceMetricMethod<MPTraits>::
EdgeWeight(const VID _source, const VID _target) {
  auto g = this->GetRoadmap();

  // If the edge does not exist, report infinite distance.
  if(!g->IsEdge(_source, _target))
    return std::numeric_limits<double>::infinity();

  // Get the intermediates. If there aren't any, the weight is just from source
  // to target.
  auto& intermediates = g->GetEdge(_source, _target).GetIntermediates();
  if(intermediates.empty())
    return Distance(g->GetVertex(_source), g->GetVertex(_target));

  // If we're still here, there are intermediates. Add up the distance through
  // the intermediates.
  double totalDistance = Distance(g->GetVertex(_source), intermediates.front())
                       + Distance(intermediates.back(), g->GetVertex(_target));

  for(size_t i = 0; i < intermediates.size() - 1; ++i)
    totalDistance += Distance(intermediates[i], intermediates[i + 1]);

  return totalDistance;
}

/*---------------------------- Configuration Scaling -------------------------*/

template <typename MPTraits>
void
DistanceMetricMethod<MPTraits>::
ScaleCfg(double _length, CfgType& _c, const CfgType& _o) {
  /// @todo This is a very expensive way to scale a configuration. We should
  ///       probably remove it and require derived classes to implement a more
  ///       efficient function (this is the best we can do for a base class that
  ///       does not know anything about the properties of the metric space).

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


template <typename MPTraits>
void
DistanceMetricMethod<MPTraits>::
ScaleCfg(double _length, CfgType& _c) {
  ScaleCfg(_length, _c, CfgType(_c.GetRobot()));
}


template <typename MPTraits>
void
DistanceMetricMethod<MPTraits>::
ScaleCfg(double _length, GroupCfgType& _c, const GroupCfgType& _o) {
  throw NotImplementedException(WHERE) << "Not yet implemented.";
}


template <typename MPTraits>
void
DistanceMetricMethod<MPTraits>::
ScaleCfg(double _length, GroupCfgType& _c) {
  const GroupCfgType origin(_c.GetGroupRoadmap(), true);
  ScaleCfg(_length, _c, origin);
}

/*----------------------------------------------------------------------------*/

#endif
