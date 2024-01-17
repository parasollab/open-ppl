#include "ReachabilityUtil.h"

#include <vector>
#include <map>
#include <utility>

/*------------------------------- Construction -------------------------------*/

ReachabilityUtil::
ReachabilityUtil() {
  this->SetName("ReachabilityUtil");
}


ReachabilityUtil::
ReachabilityUtil(XMLNode& _node) : MPBaseObject(_node) {
  this->SetName("ReachabilityUtil");

  m_extenderLabel = _node.Read("extenderLabel", true, "",
      "Kinodynamic Extender used to compute reachable set");
}

/*----------------------------- Overriden Methods ---------------------------*/

void
ReachabilityUtil::
Initialize() {
  // Ensure we got a valid extender.
  /*auto extender = dynamic_cast<KinodynamicExtender*>(
      this->GetExtender(m_extenderLabel));
  if(!extender)
    throw RunTimeException(WHERE) << "Extender for ReachabilityUtil must be a "
                                  << "KinodynamicExtender";

  // Ensure the robot has a discrete control set.
  auto robot = this->GetTask()->GetRobot();
  const auto& controls = robot->GetController()->GetControlSet();
  if(controls->empty())
    throw RunTimeException(WHERE) << "Only descrete controls are supported by "
                                  << "ReachabilityUtil for now.";
  */
}

/*----------------------------- Utility Operator ----------------------------*/

typename ReachabilityUtil::ReachableSet
ReachabilityUtil::
operator() (const Cfg& _cfg) {
  MethodTimer mt(this->GetStatClass(), "ReachabilityUtil");
/*
  // test if the cfg is already in the cache
  // if true, then return the set; otherwise compute reachable set with the
  // given extender
  auto iter = m_reachableSets.find(_cfg);
  if(iter != m_reachableSets.end())
    return iter->second;

  // Get the extender, environment, robot, and controls
  auto extender = static_cast<KinodynamicExtender*>(
      this->GetExtender(m_extenderLabel));
  auto robot = _cfg.GetRobot();
  const auto& controls = robot->GetController()->GetControlSet();
*/
  ReachableSet set;
/*
  Cfg cfg;

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
*/  
  return set;
}
