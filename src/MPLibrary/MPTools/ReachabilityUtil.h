#ifndef REACHABILITY_UTIL_H_
#define REACHABILITY_UTIL_H_

#include <vector>
#include <map>

#include <utility>
#include "MPLibrary/MPBaseObject.h"
#include "MPLibrary/Extenders/KinodynamicExtender.h"

////////////////////////////////////////////////////////////////////////////////
/// A utility that will generate the reachable set of a
///        given configuration.
///
/// @note A Reachable Set is a set of states that can reached with respect to 
///       its current degrees of freedom
///
/// Reachability-Guided Sampling for Planning Under Differential Constraints
///     Alexander Shkolnik, Matthew Walter, and Russ Tedrake
///
/// @ingroup Utilities
////////////////////////////////////////////////////////////////////////////////

template<class MPTraits>
class ReachabilityUtil : public MPBaseObject<MPTraits> {
  public:
    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Internal Types
    ///@{

    typedef vector<CfgType> ReachableSet;

    ///@}
    ///@name Construction
    ///@{

    ReachabilityUtil();

    ReachabilityUtil(XMLNode& _node);

    ///@}
    ///@name Overriden Base Methods
    ///@{
    
    virtual void Initialize() override;

    ///@}
    ///@name Utility Operator
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// Generate a reachable set of a given configuration
    /// @param _cfg The extended configuration
    /// @return The set of cfgs which are reachable from _cfg.
    ReachableSet operator() (const CfgType& _cfg);

    ///@}
  private:
    std::string m_extenderLabel;

    // TODO: optimize with unordered map
    std::map<CfgType, ReachableSet> m_reachableSets; ///< computed reachable sets
};

/*------------------------------- Construction -------------------------------*/

template<class MPTraits>
ReachabilityUtil<MPTraits>::
ReachabilityUtil(){
  this->SetName("ReachabilityUtil");
}

template<class MPTraits>
ReachabilityUtil<MPTraits>::
ReachabilityUtil(XMLNode& _node)
  : MPBaseObject<MPTraits>(_node) {
    this->SetName("ReachabilityUtil");
    m_extenderLabel = _node.Read("extenderLabel", true, "", "Kinodynamic Extender used to compute"
                                                            " reachable set");
}

/*----------------------------- Overriden Methods ---------------------------*/

template <class MPTraits>
void
ReachabilityUtil<MPTraits>::
Initialize() {
  auto extender = static_pointer_cast<KinodynamicExtender<MPTraits>>(
      this->GetExtender(m_extenderLabel));
  if(!extender) {
    throw RunTimeException(WHERE, "extender for ReachabilityUtil must be a KinodynamicExtender");
  } 

  auto robot = this->GetMPProblem()->GetRobot(0);
  const auto& controls = robot->GetController()->GetControlSet();


  if(controls->empty()) {
    throw RunTimeException(WHERE, "only descrete control are supported by ReachabilityUtil, "
                                  "for now.");
  }
}

/*----------------------------- Utility Operator ----------------------------*/

template<class MPTraits>
typename ReachabilityUtil<MPTraits>::ReachableSet
ReachabilityUtil<MPTraits>::
operator() (const CfgType& _cfg) {
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
  auto extender = static_cast<KinodynamicExtender<MPTraits>*>(
      this->GetExtender(m_extenderLabel).get());
  auto robot = _cfg.GetRobot();
  const auto& controls = robot->GetController()->GetControlSet();

  ReachableSet set;
  CfgType cfg;

  // Apply each control, if the result is not in collision add it to the
  // reachability set
  for(auto& c : *controls) {
    
    cfg = extender->ApplyControl(_cfg, c);

    if(this->m_debug) {
      std::cout << cfg << " : from control: " << c << std::endl;
    }
  
    // the applied control used returns the given cfg if 
    // the min distance was not reached.
    if(cfg == _cfg)
      continue;

    set.push_back(cfg);
  }
  m_reachableSets.insert(make_pair(_cfg, set));
  stat->StopClock("ReachabilityUtil");
  return set;
}

#endif
