#include "BasicHCR.h"

#include "MPLibrary/MPSolution.h"

#include "MPProblem/GroupTask.h"

#include "TMPLibrary/ActionSpace/Action.h"
#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/ActionSpace/ProximityCondition.h"
#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"
#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

/*----------------------- Construction -----------------------*/

BasicHCR::
BasicHCR() {
  this->SetName("BasicHCR");
}

BasicHCR::
BasicHCR(XMLNode& _node) : TMPStrategyMethod(_node) {
  this->SetName("BasicHCR");

  m_expansionProbability = _node.Read("expansionProb", false,
           0.5, 0., 1.0, "The initial probability of expanding "
           "the roadmap vs sampling an interaction.");
 
  // TODO:: Probably want more than one strategy 
  m_mpStrategy = _node.Read("mpStrat", true, "", 
           "The MPStrategy to use when expanding roadmaps. "
           "This strategy should be premised around a number of "
           "nodes to add.");

  m_sgLabel = _node.Read("sgLabel", true, "", 
           "Temp till stategraph is embedded in plan.");

  m_successUpdate = _node.Read("successUpdate", false, 1.0, 0.0, 10.0,
            "Factor to multiply utility by in case of success.");

  m_failUpdate = _node.Read("failUpdate", false, 1.0, 0.0, 10.0,
            "Factor to multiply utility by in case of failure.");
}

BasicHCR::
~BasicHCR() {}

/*------------------------ Interface -------------------------*/

void
BasicHCR::
Initialize() {
  auto as = this->GetTMPLibrary()->GetActionSpace();

  for(const auto& action : as->GetActions()) {
    m_interactionUtilityScore[action.second] = 1;
  }

}

void
BasicHCR::
PlanTasks() {
  std::cout << "Planning tasks" << std::endl;

  auto te = this->GetTaskEvaluator(m_teLabel);

  do {
    auto sr = SampleSemanticRoadmap();

    if(DRand() <= m_expansionProbability) {
      ExpandRoadmap(sr);
    }
    else {
      bool success = SampleInteraction(sr);
      UpdateSemanticRoadmapUtility(sr,success);
    }

  } while(!te->operator()());
}

/*--------------------- Helper Functions ---------------------*/

BasicHCR::SemanticRoadmap*
BasicHCR::
SampleSemanticRoadmap() {

  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());

  // Normalize SemanticRoadmap Probabilities

  double totalUtil = 0;
  std::vector<std::pair<SemanticRoadmap*,double>> dist;

  for(auto& sr : hcr->GetSemanticRoadmaps()) {

    double util = 1;

    // Check if roadmap has an active robot.
    bool active = false;
    auto group = sr->first->GetGroup();
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsActive()) {
        active = true;
        break;
      }
    }

    if(!active)
      continue;

    // Get utility score for existing sr
    auto iter = m_srUtilityScore.find(sr);
    if(iter != m_srUtilityScore.end()) {
      util = iter->second;
    }
    // Initialize utility score for new sr
    else {
      m_srUtilityScore[sr] = util;
    }

    totalUtil += util;    
    
    dist.push_back(std::make_pair(sr,totalUtil));
  }

  for(auto& util : dist) {
    util.second = util.second/totalUtil;
  }

  if(dist.back().second != 1)
    throw RunTimeException(WHERE) << "Utility not properly normalized.";
 
  // Sample distribution
 
  double d = DRand();

  for(auto sr : dist) {
    if(d <= sr.second) {
      //Temp simple update:If successful, reduce probaility of sampling again
      //m_srUtilityScore[sr.first] = m_srUtilityScore[sr.first] * 0.5;
      return sr.first;
    }
  }
  
  throw RunTimeException(WHERE) << "SemanticRoadmapo not sampled from distribution.";

  return nullptr;
}

bool
BasicHCR::
SampleInteraction(SemanticRoadmap* _sr) {

  // Set hcr to interaction expansion
  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());
  hcr->SetExpansionStatus(false);

  // Normalize Interaction Probabilities

  double totalUtil = 0;
  std::vector<std::pair<Action*,double>> dist;

  for(auto kv : m_interactionUtilityScore) {
    totalUtil += kv.second;
    dist.push_back(std::make_pair(kv.first,totalUtil));
  }

  for(auto& util : dist) {
    util.second = util.second/totalUtil;
  }

  if(dist.back().second != 1)
    throw RunTimeException(WHERE) << "Utility not properly normalized.";
 
  // Sample distribution
 
  double d = DRand();

  Interaction* inter = nullptr;

  for(auto interProb : dist) {
    if(d <= interProb.second) {
      inter = dynamic_cast<Interaction*>(interProb.first);
      break;
    }
  }
  
  if(!inter) 
    throw RunTimeException(WHERE) << "Interaction not sampled from distribution.";

  auto start = FindStartState2(inter,_sr);

  if(start.first.empty())
    return false;

  // Make copy to get modified to output state by interaction strategy
  auto startCopy = start.second;
  
  auto is = this->GetInteractionStrategyMethod(inter->GetInteractionStrategyLabel());
  if(!is->operator()(inter,startCopy)) {
    if(m_debug) {
      std::cout << "Failed to plan interaction for "
                << inter->GetLabel()
                << "and "
                << _sr->first->GetGroup()->GetLabel()
                << ".";
    }
    m_interactionUtilityScore[inter] = m_interactionUtilityScore[inter] * m_failUpdate;
    return false;
  }

  if(m_debug) {
    std::cout << "Successful interaction plan for "
              << inter->GetLabel()
              << "and "
              << _sr->first->GetGroup()->GetLabel()
              << ".";
  }

  //Temp simple update:If successful, reduce probaility of sampling again
  m_interactionUtilityScore[inter] = m_interactionUtilityScore[inter] * m_successUpdate;

  hcr->AddInteraction(start.first,start.second,startCopy,inter);

  return true;
}

void
BasicHCR::
ExpandRoadmap(SemanticRoadmap* _sr) {

  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());
  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
 
  // Set hcr to roadmap expansion
  hcr->SetExpansionStatus(true);

  // Initialize dummy task
  auto task = new GroupTask(_sr->first->GetGroup());

  for(auto r : _sr->first->GetGroup()->GetRobots()) {
    auto t = MPTask(r);
    task->AddTask(t);
  } 
 
  // Grab MPSolution from HCR
  auto solution = hcr->GetMPSolution();

  // Call the MPLibrary solve function to expand the roadmap
  lib->SetPreserveHooks(true);
  lib->Solve(prob,task,solution,m_mpStrategy, LRand(), 
             "ExpandSemanticRoadmap");
  lib->SetPreserveHooks(false);

  delete task;
}

std::pair<BasicHCR::CompositeSemanticRoadmap,BasicHCR::State>
BasicHCR::
FindStartState2(Interaction* _interaction, SemanticRoadmap* _sr) {

  if(m_debug) {
    std::cout << "Sampling interaction " 
              << _interaction->GetLabel()
              << " for "
              << _sr->first->GetGroup()->GetLabel()
              << "."
              << std::endl;
  }

  auto as = this->GetTMPLibrary()->GetActionSpace();

  auto group = _sr->first->GetGroup();
  bool matchedInputGroup = false;

  // Collect all role combinations
  std::vector<std::vector<std::string>> totalTypes;
  const auto& initialStage = _interaction->GetStages()[0];
  for(auto label : _interaction->GetStageConditions(initialStage)) {

    auto f = dynamic_cast<FormationCondition*>(as->GetCondition(label));

    if(!f) continue; // Check that it is a formation condition

    auto types = f->GetTypes();

    std::set<Robot*> used;

    // If the input group hasn't been matched to a condition, 
    // check if this is a match.
    if(!matchedInputGroup) { 
      for(auto type : types) {
        for(auto robot : group->GetRobots()) {
          if(used.count(robot))
            continue;

          if(type == robot->GetCapability()) {
            used.insert(robot);
            break;
          }
        }
      }
    }

    if(types.size() == used.size() and used.size() == group->Size()) {
      matchedInputGroup = true;
    }
    else {
      totalTypes.push_back(types);
    }
  }

  // If semantic roadmap does not match any formation conditions, quit
  if(!matchedInputGroup)
    return std::make_pair(CompositeSemanticRoadmap(),State());
   
  auto csrs = BuildCompositeSemanticRoadmaps({_sr},totalTypes,0);


  // Naive, check all state combinations
  for(auto csr : csrs) {

    std::vector<SemanticRoadmap*> vCsr;
    for(auto sr : csr) {
      vCsr.push_back(sr);
    }

    std::vector<size_t> indices(csr.size(),0);

    auto value = CheckCompositeStatesForProximity(vCsr,indices,0,_interaction);

    if(!value.second.empty())
      return value;
  }
    
  return std::make_pair(CompositeSemanticRoadmap(),State());
}

//Add recusrive function to find composite semantic roadmaps

std::vector<BasicHCR::CompositeSemanticRoadmap>
BasicHCR::
BuildCompositeSemanticRoadmaps(CompositeSemanticRoadmap _csr, 
      std::vector<std::vector<std::string>>& _types, size_t _offset) {

  if(_offset >= _types.size())
    return {_csr}; 
 
  std::vector<CompositeSemanticRoadmap> csrs;

  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());

  auto types = _types[_offset];
  for(auto sr : hcr->GetSemanticRoadmaps()) {
    if(_csr.count(sr))
      continue;

    auto group = sr->first->GetGroup();

    // Make sure this group is disjoint from existing sr in the csr
    bool disjoint = true;
    for(auto sr2 : _csr) {
      auto group2 = sr2->first->GetGroup();
      for(auto robot : group->GetRobots()) {
        for(auto robot2 : group2->GetRobots()) {
          if(robot != robot2)
            continue;
          disjoint = false;
          break;
        }
        if(disjoint == false)
          break;
      }
      if(disjoint == false)
        break;
    }
    if(disjoint == false)
      continue;

    std::set<Robot*> used;
  
    for(auto type : types) {
      for(auto robot : group->GetRobots()) {
        if(used.count(robot))
          continue;

        if(type == robot->GetCapability()) {
          used.insert(robot);
          break;
        }
      }
    }
    
    if(types.size() == used.size()) {
      CompositeSemanticRoadmap newCsr = _csr;
      newCsr.insert(sr);
      auto newCsrs = BuildCompositeSemanticRoadmaps(newCsr,_types,_offset+1);

      for(auto c : newCsrs) {
        csrs.push_back(c);
      }
    }
  }

  return csrs;
}

std::pair<BasicHCR::CompositeSemanticRoadmap,BasicHCR::State>
BasicHCR::
CheckCompositeStatesForProximity(std::vector<SemanticRoadmap*> _csr, 
                                 std::vector<size_t>& _indices,size_t _csrIndex,
                                 Interaction* _interaction) {

  if(_csrIndex == _csr.size()) {
    // Construct state
    State state;
    for(size_t i = 0; i < _csr.size(); i++) {
      auto sr = _csr[i];
      auto grm = sr->first;
      state[grm->GetGroup()] = std::make_pair(grm,_indices[i]);
    }

    CompositeSemanticRoadmap csr;
    for(auto sr : _csr) {
      csr.insert(sr);
    }

    // Check proximity
    auto as = this->GetTMPLibrary()->GetActionSpace();

    const auto& initialStage = _interaction->GetStages()[0];
    for(auto label : _interaction->GetStageConditions(initialStage)) {

      auto p = dynamic_cast<ProximityCondition*>(as->GetCondition(label));

      if(!p) continue; // Check that it is a proximty condition

      if(!p->Satisfied(state))
        return std::make_pair(CompositeSemanticRoadmap(),State());
    } 

    // Return value if it satisfies the Motion Constraints
    if(CheckCompositeStatesForMotionConstraints(_interaction,state))
      return std::make_pair(csr,state);

    return std::make_pair(CompositeSemanticRoadmap(),State());
  }
    
  auto sr = _csr[_csrIndex];
  auto grm = sr->first;

  for(auto iter = grm->begin(); iter != grm->end(); iter++) {
    auto vid = iter->descriptor();
    _indices[_csrIndex] = vid;

    auto value = CheckCompositeStatesForProximity(_csr,_indices,_csrIndex+1,_interaction);

    if(!value.second.empty())
      return value;
  }

  return std::make_pair(CompositeSemanticRoadmap(),State());
}

bool 
BasicHCR::
CheckCompositeStatesForMotionConstraints(Interaction* _interaction, State& _state) {
  
  auto as = this->GetTMPLibrary()->GetActionSpace();

  ProximityCondition* p;

  // Find the proximity condition - assume only one
  const auto& initialStage = _interaction->GetStages()[0];
  for(auto label : _interaction->GetStageConditions(initialStage)) {
    p = dynamic_cast<ProximityCondition*>(as->GetCondition(label));
 
    // Check that it is a proximty condition 
    if(p)
      break;
  }

  // Check Motion Constraints
  for(auto label : _interaction->GetStageConditions(initialStage)) {

    auto m = dynamic_cast<MotionCondition*>(as->GetCondition(label));

    // Check that it is a motion condition
    if(!m) 
      continue; 

    // ReCenter m on p
    if(p)
      m->ReCenter(p->GetBoundary()->GetCenter());

    bool sat = m->Satisfied(_state);

    std::vector<double> origin = {0,0,0};
    m->ReCenter(origin);

    if(!sat)
      return false;
  } 

  return true;
}

std::pair<BasicHCR::CompositeSemanticRoadmap,BasicHCR::State>
BasicHCR::
FindStartState(Interaction* _interaction, SemanticRoadmap* _sr) {

  if(m_debug) {
    std::cout << "Sampling interaction " 
              << _interaction->GetLabel()
              << " for "
              << _sr->first->GetGroup()->GetLabel()
              << "."
              << std::endl;
  }

  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());

  // Check for valid composite sr

  //TODO::Move all of this into hook functions and caching
  // Collect possible role assignments
  // Map of a semantic roadmap to each set of roles that it satisfies and the vids it satisfies them with
  std::unordered_map<SemanticRoadmap*,std::vector<RoleSet>> roleMap;
  std::unordered_map<SemanticRoadmap*,std::vector<std::vector<size_t>>> vidMap;

  roleMap[_sr] = {};

  RoleSet totalRoles;

  const auto& initialStage = _interaction->GetStages()[0];
  for(auto label : _interaction->GetStageConditions(initialStage)) {

    auto m = dynamic_cast<MotionCondition*>(as->GetCondition(label));

    if(!m) continue; // Check that it is a motion condition

    auto roles = m->GetRoles();
    for(auto role : roles) {
      totalRoles.insert(role);
    }

    // Find groupCfgs that satisfy the motion condition
    for(auto sr : hcr->GetSemanticRoadmaps()) {

      // Check that the number of robots in the sr matches the motion condition
      if(sr->first->GetGroup()->Size() != roles.size())
        continue;

      std::vector<size_t> satisfyingVIDs;

      auto rm = sr->first;
      State state;
      for(auto vit = rm->begin(); vit != rm->end(); vit++) {
        state.clear();

        auto vid = vit->descriptor();
        state[rm->GetGroup()] = std::make_pair(rm,vid);

        if(m->Satisfied(state)) {
          satisfyingVIDs.push_back(vid);
        }
      }

      if(satisfyingVIDs.size() == 0)
        continue;

      roleMap[sr].push_back(roles);
      vidMap[sr].push_back(satisfyingVIDs);
    }
  }

  // Check that the input semantic roadmap is fully utilized
  if(roleMap[_sr].empty())
    return std::make_pair(CompositeSemanticRoadmap(), State());

  // Build Composite Semantic Roadmaps
  // Extract just the role labels
  /*std::unordered_map<SemanticRoadmap*,std::vector<RoleSet>> possibleRoles;
  for(const auto& kv : roleMap) {
    for(const auto& roles : kv.second) {
      possibleRoles[kv.first].push_back(roles.first);
    }
  }*/

  // Check composite semantic roadmaps with the input semantic roadmap in initial set of satisfied roles
  std::vector<std::unordered_map<SemanticRoadmap*,size_t>> csrs;
  //for(auto init : possibleRoles[_sr]) {
  for(size_t i = 0; i < roleMap[_sr].size(); i++) {
    auto init = roleMap[_sr][i];

    std::set<std::string> satisfiedRoles;
    for(auto role : init) {
      satisfiedRoles.insert(role);
    }

    std::unordered_map<SemanticRoadmap*,size_t> csr;
    csr[_sr] = i;

    //auto moreCsrs = BuildCompositeSemanticRoadmaps(possibleRoles,totalRoles,satisfiedRoles,csr);
    auto moreCsrs = BuildCompositeSemanticRoadmaps(roleMap,totalRoles,satisfiedRoles,csr);
    for(auto csr : moreCsrs) {
      csrs.push_back(csr);
    }
  }

  // Make sure at least one valid state was found.
  if(csrs.size() == 0)
    return std::make_pair(CompositeSemanticRoadmap(), State());

  //Todo::Sample a state from there

  for(auto csrInfo : csrs) {
    CompositeSemanticRoadmap csr;
    State state;

    for(auto kv : csrInfo) {
      auto sr = kv.first;
      auto vidSet = vidMap[sr][kv.second];
      //TODO::Sample vid from set or check all combos
      auto vid = vidSet[0];

      csr.insert(sr);
      state[sr->first->GetGroup()] = std::make_pair(sr->first,vid);
    }

    //TODO::Validate that composite state works
    return std::make_pair(csr,state);
  }

  /*for(auto csm : csms) {
    auto rm = _sr->first;
    State state;

    for(auto vit = rm->begin(); vit != rm->end(); vit++) {

      auto vid = vit->descriptor();

      state[rm->GetGroup()] = std::make_pair(rm,vid);

      const auto& initialStage = _interaction->GetStages()[0];
      for(auto label : _interaction->GetStageConditions(initialStage)) {

        auto m = dynamic_cast<MotionCondition*>(as->GetCondition(label));

        if(!m) continue; // Check that it is a motion condition

        if(!m->Satisfied(state)) {
          state.clear();
          break;
        }
      }

      // Check if state met all conditions
      if(!state.empty()) {
        if(m_debug) {
          auto cfg = vit->property();
          std::cout << "Valid State at VID: "
                   << vid
                   << ", representing"
                   << cfg.PrettyPrint()
                    <<std::endl;
        }
  
        return std::make_pair(csm,state);
      }
    }
  }*/

  return std::make_pair(CompositeSemanticRoadmap(), State());
}

std::vector<std::unordered_map<BasicHCR::SemanticRoadmap*,size_t>>
BasicHCR::
BuildCompositeSemanticRoadmaps(
    const std::unordered_map<SemanticRoadmap*,std::vector<std::set<std::string>>>& _possibleRoles,
    RoleSet _totalRoles, RoleSet _satisfiedRoles, std::unordered_map<SemanticRoadmap*,size_t> _csr) {

  // Check if _totalRoles matches _satisfiedRoles
  if(_totalRoles == _satisfiedRoles) 
    return {_csr};

  std::vector<std::unordered_map<SemanticRoadmap*,size_t>> csrs;

  for(auto kv : _possibleRoles) {
    auto sr = kv.first;

    // Check if semantic roadmap is already in composite semantic roadmap
    auto iter = _csr.find(sr);
    if(iter != _csr.end())
      continue;

    // Try to add each possible role assignment for semantic roadmap
    const auto& roleSets = kv.second; 
    for(size_t i = 0; i < roleSets.size(); i++) {
      auto roleSet = roleSets[i];

      auto satisfied = _satisfiedRoles;

      for(auto role : roleSet) {
        satisfied.insert(role);
      }

      // Check that the satisfied set increased by the proper size. 
      // Otherwise, some roles are already claimed, and we move on to the next role set.
      if(satisfied.size() != roleSet.size() + _satisfiedRoles.size())
        continue;
 
      auto csr = _csr;
      csr[sr] = i;

      auto newCsrs = BuildCompositeSemanticRoadmaps(_possibleRoles,_totalRoles,
                                                    satisfied,csr);

      for(auto c : newCsrs) {
        csrs.push_back(c);
      }
    }
  } 

  return csrs;
}

void
BasicHCR::
UpdateSemanticRoadmapUtility(SemanticRoadmap* _sr, bool _success) {
  double util = m_srUtilityScore.at(_sr);

  if(_success)
    util = util * m_successUpdate;
  else 
    util = util * m_failUpdate;

  m_srUtilityScore[_sr] = util;
}

/*------------------------------------------------------------*/
