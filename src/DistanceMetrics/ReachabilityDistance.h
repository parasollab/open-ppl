#ifndef REACHABILITYDISTANCE_H_
#define REACHABILITYDISTANCE_H_

#include <vector>
#include <limits>
#include "DistanceMetricMethod.h"
#include "MPStrategies/DynamicRegionRRT.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Distance Metrics 
/// @brief A distance metric that checks if the new configuration is within the 
///       reachable set of the exteneding configuration
/// @tparam MPProblemType   The Stategy that is using the utility (template
///                         templated type)  
/// @tparam MPTraits        Motion planning universe
template <template <typename> class MPProblemType, typename MPTraits>
class ReachabilityDistance : public DistanceMetricMethod<MPTraits> {
  public:
    /// @name Motion Planning Types
    /// @{
  
    typedef typename MPTraits::CfgType CfgType;

    /// @}
    /// @name Other Types
    /// @{
    
    /// @}
    /// @name Contructors
    /// @{

    ReachabilityDistance(MPProblemType<MPTraits>* _problem);
    
    /// @}
    /// @name Overloaded Distance Methods
    /// @{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);

    /// @}

  private:
    MPProblemType<MPTraits>* m_problem;
};

/*------------------------------- Construction -------------------------------*/

template <template <typename> class MPProblemType, typename MPTraits>
ReachabilityDistance<MPProblemType, MPTraits>::
ReachabilityDistance(MPProblemType<MPTraits>* _problem) : DistanceMetricMethod<MPTraits>(), 
  m_problem(_problem) {}

/*----------------------------- Distance Methods -----------------------------*/

template <template <typename> class MPProblemType, typename MPTraits>
double
ReachabilityDistance<MPProblemType, MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  auto dm = m_problem->GetDistanceMetric("euclidean");
  const vector<CfgType>& rset = m_problem->GetReachableSet();

  for(const auto& c : rset)
    if(c == _c2)
      return dm->Distance(_c1, _c2); 
  
  return numeric_limits<double>::infinity();
}
#endif
