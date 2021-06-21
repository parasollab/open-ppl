#include "BasicHCR.h"

#include "MPLibrary/MPSolution.h"

#include "MPProblem/GroupTask.h"

#include "TMPLibrary/ActionSpace/Action.h"
#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
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
    auto sm = SampleSemanticRoadmap();

    if(DRand() <= m_expansionProbability) {
      ExpandRoadmap(sm);
    }
    else {
      SampleInteraction(sm);
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
      m_srUtilityScore[sr.first] = m_srUtilityScore[sr.first] * 0.5;
      return sr.first;
    }
  }
  
  throw RunTimeException(WHERE) << "SemanticRoadmapo not sampled from distribution.";

  return nullptr;
}

bool
BasicHCR::
SampleInteraction(SemanticRoadmap* _sr) {

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

  auto start = FindStartState(inter,_sr);

  if(start.first.empty())
    return false;

  // Make copy to get modified to output state by interaction strategy
  auto startCopy = start.second;
  
  auto is = this->GetInteractionStrategyMethod(inter->GetInteractionStrategyLabel());
  if(!is->operator()(inter,startCopy))
    return false;

  //Temp simple update:If successful, reduce probaility of sampling again
  m_interactionUtilityScore[inter] = m_interactionUtilityScore[inter] * 0.5;

  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());
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
 
  // Initialize dummy task
  auto task = new GroupTask(_sr->first->GetGroup());

  for(auto r : _sr->first->GetGroup()->GetRobots()) {
    auto t = MPTask(r);
    task->AddTask(t);
  } 
 
  // Grab MPSolution from HCR
  auto solution = hcr->GetMPSolution();

  // Call the MPLibrary solve function to expand the roadmap
  lib->Solve(prob,task,solution,m_mpStrategy, LRand(), 
             "ExpandSemanticRoadmap");

  delete task;
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

  // Check for valid composite sr

  // Temp for implementing sampling
  CompositeSemanticRoadmap csm = {_sr};

  auto rm = _sr->first;
  State state;

  for(auto vit = rm->begin(); vit != rm->end(); vit++) {

    auto vid = vit->descriptor();

    if(m_debug) {
      auto cfg = vit->property();
      std::cout << "Evaluating VID: "
                << vid
                << ", representing"
                << cfg.PrettyPrint()
                <<std::endl;
    }
  
    state[rm->GetGroup()] = std::make_pair(rm,vid);

    for(auto label : _interaction->GetPreConditions()) {

      auto m = dynamic_cast<MotionCondition*>(as->GetCondition(label));

      if(!m) continue; // Check that it is a motion condition

      if(!m->Satisfied(state)) {
        state.clear();
        break;
      }
    }

    // Check if state met all conditions
    if(!state.empty())
      return std::make_pair(csm,state);
  }

  return std::make_pair(CompositeSemanticRoadmap(), State());
}

/*------------------------------------------------------------*/
