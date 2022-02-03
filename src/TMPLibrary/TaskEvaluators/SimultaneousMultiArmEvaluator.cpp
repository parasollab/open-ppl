#include "SimultaneousMultiArmEvaluator.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"
#include "TMPLibrary/Solution/Plan.h"

/*------------------------------ Construction --------------------------------*/

SimultaneousMultiArmEvaluator::
SimultaneousMultiArmEvaluator() {
  this->SetName("SimultaneousMultiArmEvaluator");
}

SimultaneousMultiArmEvaluator::
SimultaneousMultiArmEvaluator(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("SimultaneousMultiArmEvaluator");

  m_maxAttempts = _node.Read("maxAttempts",false,5,0,MAX_INT,
            "Number of attempts to sample a transition.");

  m_connectorLabel = _node.Read("connectorLabel",true,"",
            "Connector method to use to connect transition paths to roadmaps.");
}

SimultaneousMultiArmEvaluator::
~SimultaneousMultiArmEvaluator() { }

/*-------------------------------- Interface ---------------------------------*/

void
SimultaneousMultiArmEvaluator::
Initialize() {
  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();
  if(g->Size() == 0)
    return;

  // TODO::Temporary code to test helper functions
  std::cout << "Calling dummy initialize code for testing development." << std::endl;

  auto mode = SelectMode();
  auto neighbors = GetModeNeighbors(mode);
  for(auto n : neighbors) {
    SampleTransition(mode, n);
  }

}

/*----------------------------- Helper Functions -----------------------------*/

bool
SimultaneousMultiArmEvaluator::
Run(Plan* _plan) {

  Plan* plan = _plan ? _plan : this->GetPlan();
  plan->Print();

  return false;
}

SimultaneousMultiArmEvaluator::VID
SimultaneousMultiArmEvaluator::
SelectMode() {

  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();

  // For now, sample random mode
  auto vid = LRand() % g->Size();
  return vid;
}

std::set<SimultaneousMultiArmEvaluator::VID>
SimultaneousMultiArmEvaluator::
GetModeNeighbors(VID _vid) {

  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();
  std::set<VID> neighbors;

  // Simply return all neighbors in graph for now
  auto vit = g->find_vertex(_vid);

  for(auto eit = vit->begin(); eit != vit->end(); eit++) {
    neighbors.insert(eit->target());
  }
  
  return neighbors;
}

bool
SimultaneousMultiArmEvaluator::
SampleTransition(VID _source, VID _target) {

  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto prob = this->GetMPProblem();
  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();

  auto edge = g->GetEdge(_source, _target);

  // Plan each interaction
  for(auto kv : edge) {
    auto interaction = kv.first;
    auto is = this->GetInteractionStrategyMethod(
                    interaction->GetInteractionStrategyLabel());
    auto stages = interaction->GetStages();

    for(auto pair : kv.second) {
      auto reverse = pair.first;
      auto roleMap = pair.second;

      // Create state
      State state;

      auto stage = stages.front();
  
      for(auto condition : interaction->GetStageConditions(stage)) {
      
        // Grab condition and make sure it is of type formation
        auto c = as->GetCondition(condition);
        auto f = dynamic_cast<FormationCondition*>(c);
        if(!f)
          continue;

        // Create appropriate robot group
        std::vector<Robot*> robots;
        std::string label;
        
        for(auto role : f->GetRoles()) {
          auto robot = roleMap[role];
          robots.push_back(robot);
          label += ("::" + robot->GetLabel());
        }

        auto group = prob->AddRobotGroup(robots,label);

        // Add group to state
        state[group] = std::make_pair(nullptr,MAX_INT);
      }

      //Plan interaction
      for(size_t i = 0; i < m_maxAttempts; i++) { 
        bool success = is->operator()(interaction,state);

        if(success) {
          ConnectToExistingRoadmap(interaction,state,reverse);
          break;
        }
      }
    }
  }

  return false;
}

void
SimultaneousMultiArmEvaluator::
ConnectToExistingRoadmap(Interaction* _interaction, State& _state, bool _reverse) {

  // Initialize set of robot paths
  std::unordered_map<Robot*,std::vector<Cfg>> robotPaths;
  for(auto kv : _state) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      robotPaths[robot] = {};
    }
  }
  
  // Collect individual robot paths

  const auto& stages = _interaction->GetStages();
  double totalCost = 0;

  for(size_t i = 1; i < stages.size(); i++) {
    auto paths = _interaction->GetToStagePaths(stages[i]);

    double stageCost = 0;

    for(auto path : paths) {
      const auto& cfgs = path->Cfgs();
      auto robot = path->GetRobot();
  
      for(auto& cfg : cfgs) {
        robotPaths[robot].push_back(cfg);
      }

      stageCost = std::max(stageCost,double(path->TimeSteps()));
    }

    totalCost += stageCost;
  }

  // TODO::Add start and end robot cfgs to individual roadmaps

  for(auto& kv : robotPaths) {
    if(kv.second.empty())
      continue;

    const auto& start = kv.second.front();
    const auto& end = kv.second.back();

    AddToRoadmap(start);
    if(start != end)
      AddToRoadmap(end);
  }

  // TODO::Save as edge in tensor-product roadmap

}

void
SimultaneousMultiArmEvaluator::
AddToRoadmap(Cfg _cfg) {

  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  //auto mpSolution = sg->GetMPSolution();

  // Get appropriate roadmap
  auto robot = _cfg.GetRobot();
  auto group = prob->GetRobotGroup(robot->GetLabel());
  auto rm = sg->GetGroupRoadmap(group);

  // Convert to group cfg
  GroupCfg gcfg(rm);
  gcfg.SetRobotCfg(robot,std::move(_cfg));

  // Add vertex to roadmap
  auto vid = rm->AddVertex(gcfg);

  // Make dummy group task
  MPTask mt(robot);
  GroupTask gt(group);
  gt.AddTask(mt);

  // Configure library
  lib->SetGroupTask(&gt);
  lib->SetTask(nullptr);
  //lib->SetMPSolution(mpSolution);

  // Connect new vertex to existing roadmap
  auto connector = lib->GetConnector(m_connectorLabel);
  connector->Connect(rm,vid);

  if(m_debug) {
    std::cout << "Connected new vid (" << vid 
              << ") to " << rm->get_degree(vid)
              << " vertices." << std::endl;
  }
}

/*----------------------------------------------------------------------------*/
