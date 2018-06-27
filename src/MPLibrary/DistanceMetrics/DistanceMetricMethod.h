#ifndef DISTANCE_METRIC_METHOD_H
#define DISTANCE_METRIC_METHOD_H

#include "MPLibrary/MPBaseObject.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief Base algorithm abstraction for \ref DistanceMetrics.
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
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DistanceMetricMethod  : public MPBaseObject<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::RoadmapType   RoadmapType;
    typedef typename RoadmapType::GraphType  GraphType;
    typedef typename GraphType::VID          VID;
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

    /// Compute a distance between two configurations
    /// @param _c1 Configuration 1
    /// @param _c2 Configuration 2
    /// @return Distance value
    virtual double Distance(const CfgType& _c1, const CfgType& _c2) = 0;

    /// GroupCfg Overload
    virtual double Distance(const GroupCfgType& _c1, const GroupCfgType& _c2)
        { throw RunTimeException(WHERE, "Not Implemented!"); }


    /// @brief Scale a directional configuration to a certain magnitude
    /// @param _length Desired magnitude
    /// @param _c Configuration to be scaled
    /// @param _o Configuration to scale upon (origin of scaling)
    /// @return Distance value
    virtual void ScaleCfg(double _length, CfgType& _c, const CfgType& _o);


    /// Compute the weight of an existing roadmap edge
    /// @param _source VID source/start
    /// @param _target VID target/end
    /// @return The total edge weight going through each intermediate.
    virtual double EdgeWeight(const VID _source, const VID _target);

    void ScaleCfg(double _length, CfgType& _c);


    /// Group Cfg Overloads:
    virtual void ScaleCfg(double _length, GroupCfgType& _c,
              const GroupCfgType& _o)
        { throw RunTimeException(WHERE, "Not Implemented!"); }

    void ScaleCfg(double _length, GroupCfgType& _c)
        { throw RunTimeException(WHERE, "Not Implemented!"); }

    ///@}
    ///@name Modifiers
    ///@{

    /// Sets and sorts the active robots, which is used for group cfg comparing.
    void SetActiveRobots(const Formation& _robots);
    Formation GetActiveRobots() const noexcept { return m_activeRobots; }

    ///@}

  protected:

    Formation m_activeRobots; ///< Used for group cfgs to ensure valid neighbors
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
DistanceMetricMethod<MPTraits>::
DistanceMetricMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
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


template <typename MPTraits>
void
DistanceMetricMethod<MPTraits>::
ScaleCfg(double _length, CfgType& _c) {
  ScaleCfg(_length, _c, CfgType(_c.GetRobot()));
}


template <typename MPTraits>
double
DistanceMetricMethod<MPTraits>::
EdgeWeight(const VID _source, const VID _target) {
  auto g = this->GetRoadmap()->GetGraph();

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

/*----------------------------------------------------------------------------*/


template <typename MPTraits>
void
DistanceMetricMethod<MPTraits>::
SetActiveRobots(const Formation& _robots) {
  m_activeRobots = _robots;

  // Sort it for easy equality checking (order ultimately doesn't matter though)
  if(m_activeRobots.size() > 1)
    std::sort(m_activeRobots.begin(), m_activeRobots.end());
}

#endif
