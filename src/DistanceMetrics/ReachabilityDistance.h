#ifndef REACHABILITY_DISTANCE_H_
#define REACHABILITY_DISTANCE_H_

#include <vector>
#include <limits>
#include "Utilities/ReachabilityUtil.h"
#include "Environment/ReachableBoundary.h"
#include "DistanceMetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Distance Metrics
/// @brief A distance metric that checks if the new configuration is within the
///       reachable set of the exteneding configuration
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ReachabilityDistance : public DistanceMetricMethod<MPTraits> {
  public:
    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;

    ///@{
    ///@name Local Types
    ///@{

    typedef typename ReachabilityUtil<MPTraits>::ReachableSet ReachableSet;

    ///@{
    ///@name Contruction
    ///@{

    ReachabilityDistance();

    ReachabilityDistance(MPProblemType* _problem, XMLNode& _node);

    virtual ~ReachabilityDistance() = default;

    ///@}
    ///@name Overloaded Distance Methods
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    ///@}
  
  private:
    ReachabilityUtil<MPTraits> m_reachabilityUtil; ///< computes reachable set
    string m_dmLabel;                              ///< distance metric label
};

/*------------------------------- Construction -------------------------------*/

template<class MPTraits>
ReachabilityDistance<MPTraits>::
ReachabilityDistance() {
  this->SetName("ReachabilityDistance");
}

template<class MPTraits>
ReachabilityDistance<MPTraits>::
ReachabilityDistance(MPProblemType* _problem, XMLNode& _node)
  : DistanceMetricMethod<MPTraits>(_problem, _node),
  m_reachabilityUtil(_problem, _node) {
  m_dmLabel = _node.Read("dmLabel", false, "euclidean", 
                         "Underlying distance metric");
  this->SetName("ReachabilityDistance");
}

/*----------------------------- Distance Methods -----------------------------*/

template<class MPTraits>
double
ReachabilityDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {

  // Get distance metric and reachable set
  const ReachableSet& rSet = m_reachabilityUtil(_c1, "FixedBest");
  auto dm = this->GetDistanceMetric(m_dmLabel);

  std::pair<double, CfgType> smallest = std::make_pair(numeric_limits<double>::infinity(),
                                                       CfgType(_c1.GetRobotIndex()));

  // finds the closest cfg in the reachable set to the new cfg
  for(const auto& cfg : rSet) {
    double d = dm->Distance(cfg, _c2);
    if(d < smallest.first) {
      smallest.first = d;
      smallest.second = cfg;
    }
  }

  double d = smallest.first;
  // if the smallest distance is greater than the distance between the inputted 
  // cfg's then return infinity otherwise return the distance.
  if(d > dm->Distance(_c1, _c2)) 
    return numeric_limits<double>::infinity();
  else
    return d;
}

#endif
