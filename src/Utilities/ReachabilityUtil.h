#ifndef REACHABILITY_UTIL_H_
#define REACHABILITY_UTIL_H_

#include <vector>

#include "MPProblem/MPBaseObject.h"
#include "Environment/Control.h"
#include "Environment/NonHolonomicMultiBody.h"
#include "Extenders/KinodynamicExtender.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief A utility that will generate the reachable set of a
///        given configuration.
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

    ReachabilityUtil(MPProblemType* _problem, XMLNode& _node);

    ///@}
    ///@name Utility Operator
    ///@{

    ReachableSet operator() (const CfgType& _cfg,
        const std::string& _extenderLabel);

    ///@}
};

template<class MPTraits>
ReachabilityUtil<MPTraits>::
ReachabilityUtil(MPProblemType* _problem, XMLNode& _node)
  : MPBaseObject<MPTraits>(_problem, _node) {}

template<class MPTraits>
typename ReachabilityUtil<MPTraits>::ReachableSet
ReachabilityUtil<MPTraits>::
operator() (const CfgType& _cfg, const string& _extenderLabel) {
  // Get the extender, environment, robot, and controls
  auto extender = static_pointer_cast<KinodynamicExtender<MPTraits>>(
      this->GetExtender(_extenderLabel));

  Environment* env = this->GetEnvironment();

  shared_ptr<NonHolonomicMultiBody> robot =
      dynamic_pointer_cast<NonHolonomicMultiBody>(_cfg->GetRobot());

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

  return set;
}

#endif
