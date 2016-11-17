#ifndef REACHABILITY_DISTANCE_H_
#define REACHABILITY_DISTANCE_H_

#include <vector>
#include <limits>
#include "Utilities/ReachabilityUtil.h"
#include "Environment/ReachableBoundary.h"
#include "EuclideanDistance.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Distance Metrics
/// @brief A distance metric that checks if the new configuration is within the
///       reachable set of the exteneding configuration
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ReachabilityDistance : public EuclideanDistance<MPTraits> {
  public:
    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;

    ///@name Contruction
    ///@{

    ReachabilityDistance(MPProblemType* _problem, XMLNode& _node);

    virtual ~ReachabilityDistance() = default;

    ///@}
    ///@name Overloaded Distance Methods
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    ///@}
  private:
    ReachabilityUtil<MPTraits> m_reachabilityUtil;
};

/*------------------------------- Construction -------------------------------*/

template<class MPTraits>
ReachabilityDistance<MPTraits>::
ReachabilityDistance(MPProblemType* _problem, XMLNode& _node)
  : EuclideanDistance<MPTraits>(_problem, _node), m_reachabilityUtil(_problem, _node) {}

/*----------------------------- Distance Methods -----------------------------*/

template<class MPTraits>
double
ReachabilityDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  // Get distance metric and reachable set
  const vector<CfgType>& rSet = m_reachabilityUtil(_c1, "FixedBest");

  ReachableBoundary<MPTraits> bb(rSet);

  for(const auto& c : rSet)
    if(bb.InBounds(_c2.GetPoint()))
      return EuclideanDistance<MPTraits>::Distance(_c1, _c2);

  return numeric_limits<double>::infinity();
}

#endif
