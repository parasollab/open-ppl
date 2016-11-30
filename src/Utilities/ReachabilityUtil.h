#ifndef REACHABILITY_UTIL_H_
#define REACHABILITY_UTIL_H_

#include <vector>
#include <map>

#include <utility>
#include "MPProblem/MPBaseObject.h"
#include "Environment/Control.h"
#include "Environment/NonHolonomicMultiBody.h"
#include "Extenders/KinodynamicExtender.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief A utility that will generate the reachable set of a
///        given configuration.
///
/// @note A Reachable Set is a set of states that can reached with respect to 
///       its current degrees of freedom
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ReachabilityUtil : public MPBaseObject<MPTraits> {
  public:
    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Internal Types
    ///@{

    typedef vector<CfgType> ReachableSet;

    ///@}
    ///@name Construction
    ///@{

    ReachabilityUtil() = default;

    ReachabilityUtil(MPProblemType* _problem, XMLNode& _node);

    ///@}
    ///@name Utility Operator
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Generate a reachable set of a given configuration
    /// \param[in] _cfg The extended configuration
    /// \param[in] _extednerLabel The extender used to extend cfg 
    ReachableSet operator() (const CfgType& _cfg,
        const std::string& _extenderLabel);

    ///@}
  private:

    // TODO: optimize with unordered map
    map<CfgType, ReachableSet> m_reachableSets; ///< computed reachable sets
};

/*------------------------------- Construction -------------------------------*/

template<class MPTraits>
ReachabilityUtil<MPTraits>::
ReachabilityUtil(MPProblemType* _problem, XMLNode& _node)
  : MPBaseObject<MPTraits>(_problem, _node) {}


/*----------------------------- Utility Operator ----------------------------*/

template<class MPTraits>
typename ReachabilityUtil<MPTraits>::ReachableSet
ReachabilityUtil<MPTraits>::
operator() (const CfgType& _cfg, const string& _extenderLabel) {
  auto stat = this->GetStatClass();
  stat->StartClock("ReachabilityUtil");
  
  // test if the cfg is already in the cache
  // if true, then return the set; otherwise compute reachable set with the 
  // given extender
  auto iter = m_reachableSets.find(_cfg);
  if(iter != m_reachableSets.end()) {
    stat->StopClock("ReachabilityUtil");
    return iter->second;
  }
  
  // Get the extender, environment, robot, and controls
  auto extender = static_pointer_cast<KinodynamicExtender<MPTraits>>(
      this->GetExtender(_extenderLabel));

  shared_ptr<NonHolonomicMultiBody> robot =
      dynamic_pointer_cast<NonHolonomicMultiBody>(_cfg.GetRobot());

  const vector<shared_ptr<Control>>& control = robot->AvailableControls();

  ReachableSet set;
  CfgType cfg;
  bool collision;

  // Apply each control, if the result is not in collision add it to the
  // reachability set
  for(auto& c : control) {
    cfg = extender->ApplyControl(_cfg, c->GetControl(), collision);
    if(!collision)
      set.push_back(cfg);
  }
  m_reachableSets.insert(make_pair(_cfg, set));
  stat->StopClock("ReachabilityUtil");
  return set;
}

#endif
