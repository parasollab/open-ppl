#ifndef REACHABILITYSET_H_
#define REACHABLITLYSET_H_


#include "MPProblem/MPBaseObject.h"
#include "Environment/Control.h"
#include "Environment/NonHolonomicMultiBody.h"
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities 
/// @brief A utility that will generate the reachable set of a
///        given configuration.
/// @tparam MPProblemType   The Stategy that is using the utility (template
///                         templated type)  
/// @tparam MPTraits        Motion planning universe
template <template <typename> class MPProblemType, typename MPTraits>
class ReachabilityUtil : public MPBaseObject<MPTraits> {
  public:
    /// @name Motion Planning Types
    /// @{
    
    typedef typename MPTraits::CfgType CfgType;

    /// @}
    /// @name Internal Types
    /// @{
    
    typedef vector<CfgType> ReachableSet;

    /// @}
    /// @name Constructor
    /// @{
    
    ReachabilityUtil(MPProblemType<MPTraits>* _problem);

    /// @}
    /// @name Utility Operator
    /// @{
    
    ReachableSet operator() (const CfgType& _cfg); 

    /// @}
    
  private:
    MPProblemType<MPTraits>* m_problem; ///< problem pointer
};

template <template <typename> class MPProblemType, typename MPTraits>
ReachabilityUtil<MPProblemType, MPTraits>::
ReachabilityUtil(MPProblemType<MPTraits>* _problem): MPBaseObject<MPTraits>(), m_problem(_problem) {}

template <template <typename> class MPProblemType, typename MPTraits>
typename ReachabilityUtil<MPProblemType, MPTraits>::ReachableSet
ReachabilityUtil<MPProblemType, MPTraits>::
operator() (const CfgType& _cfg) {
  // hard coded for now
  //
  auto extender = static_pointer_cast<KinodynamicExtender<MPTraits>>(m_problem->GetExtender("FixedBest"));

  Environment* env = m_problem->GetEnvironment();
  shared_ptr<NonHolonomicMultiBody> robot =
      dynamic_pointer_cast<NonHolonomicMultiBody>(env->GetRobot(0));

  const vector<shared_ptr<Control>>& control = robot->AvailableControls();
  

  ReachableSet set;
  CfgType cfg;
  bool collision; 
  
  for(auto& c : control) {
    cfg = extender->ApplyControl(_cfg, c->GetControl(), collision);
    if(!collision)
      set.push_back(cfg);
  }

  return set;
}

#endif
