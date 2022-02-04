#include "SimultaneousMultiArmEvaluator.h"

#include "Behaviors/Agents/Coordinator.h"

#include "ConfigurationSpace/Formation.h"

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

  m_maxIters = _node.Read("maxIters",false,1000,1,MAX_INT,
            "Number of iterations to look for a path.");

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

  // TODO::Temporary code to test helper functions
  if(g->Size() == 0)
    return;

  std::cout << "Calling dummy initialize code for testing development." << std::endl;

  //auto mode = SelectMode().first;
  //auto neighbors = GetModeNeighbors(mode);
  //for(auto n : neighbors) {
  //  SampleTransition(mode, n);
  //}

  Run();

}

/*----------------------------- Helper Functions -----------------------------*/

bool
SimultaneousMultiArmEvaluator::
Run(Plan* _plan) {

  Plan* plan = _plan ? _plan : this->GetPlan();
  plan->Print();

  auto prob = this->GetMPProblem();
  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  auto c = this->GetPlan()->GetCoordinator();

  // Create composite group
  std::vector<Robot*> robots;
  for(auto pair : c->GetInitialRobotGroups()) {
    auto group = pair.first;
    for(auto robot : group->GetRobots()) {
      robots.push_back(robot);
    }
  }

  auto group = prob->AddRobotGroup(robots,"TensorGroup");
  sg->GetMPSolution()->AddRobotGroup(group);

  m_tensorProductRoadmap = std::unique_ptr<TensorProductRoadmap>(new TensorProductRoadmap(group,sg->GetMPSolution()));

  m_taskGraph = std::unique_ptr<TaskGraph>(new TaskGraph(c->GetRobot()));

  m_actionExtendedGraph = std::unique_ptr<ActionExtendedGraph>(new ActionExtendedGraph(c->GetRobot()));

  auto actionStart = CreateRootNodes();
  std::cout << actionStart << std::endl;

  for(size_t i = 0; i < m_maxIters; i++) {

    // Select Mode 

    auto pair = SelectMode();
    auto mode = pair.first;
    auto history = pair.second;
  
    std::cout << mode << " " << history << std::endl;

    // TODO::Compute Heuristic



    // Sample Transitions

    auto neighbors = GetModeNeighbors(mode);
    for(auto n : neighbors) {
      SampleTransition(mode, n);
    }

    // TODO::RRT Logic
    // - Qnear = Sample vertex (from heursitic)
    // - Qnew = Extend(Qnear,hid,heuristic)
    // - Qbest = rewire(Qmew)

    // TODO::If Qbest to Qnew is valid add it to the graph and connect to stuff

    // TODO::Check if Qnew is in a neighboring mode and create new vertex over there if so

    // TODO::Check if Qnew is a goal configuration and update the path if so

  }

  return false;
}

size_t
SimultaneousMultiArmEvaluator::
CreateRootNodes() {

  // Create initial vertex
  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
 
  std::vector<GroupCfg> cfgs;

  // Get initial vertex in TensorProductRoadmap
  auto c = this->GetPlan()->GetCoordinator();
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    auto rm = sg->GetGroupRoadmap(group);
    // Note::Assuming first vertex is starting position
    cfgs.push_back(rm->GetVertex(0));
  }

  auto tprStart = CreateTensorProductVertex(cfgs);

  // Create initial Task Graph vertex
  TaskState taskStartState;
  taskStartState.vid = tprStart;
  taskStartState.mode = 0;

  auto taskStart = m_taskGraph->AddVertex(taskStartState);

  // Create initial Action Extended Graph vertex
  ActionExtendedState actionStartState;
  actionStartState.vid = taskStart;

  m_actionHistories.emplace_back(ActionHistory());
  actionStartState.ahid = m_actionHistories.size()-1;

  // Create entry for initial mode and history
  m_modeHistories[0].push_back(0);

  return m_actionExtendedGraph->AddVertex(actionStartState);
}


std::pair<size_t,size_t>
SimultaneousMultiArmEvaluator::
SelectMode() {

  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();

  while(true) {
    // For now, sample random mode
    auto mode = LRand() % g->Size();
    
    auto histories = m_modeHistories[mode];
  
    if(histories.empty())
      continue;

    auto hid = LRand() % histories.size();

    if(m_debug) {
      std::cout << "Selected Mode " << mode << " with history [ ";
      for(auto vid : m_actionHistories[hid]) {
        std::cout << vid << ", ";
      }
      std::cout << "]" << std::endl;
    }

    return std::make_pair(mode,hid);
  }

  return std::make_pair(MAX_INT,MAX_INT);
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
        State end = state;
        bool success = is->operator()(interaction,end);

        if(success) {
          ConnectToExistingRoadmap(interaction,state,end,reverse);
          break;
        }
      }
    }
  }

  return false;
}

void
SimultaneousMultiArmEvaluator::
ConnectToExistingRoadmap(Interaction* _interaction, State& _start, State& _end, bool _reverse) {

  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());

  // Initialize set of robot paths
  auto interactionPath = std::unique_ptr<InteractionPath>(new InteractionPath());
  m_interactionPaths.push_back(std::move(interactionPath));
  InteractionPath& robotPaths = *(m_interactionPaths.back().get());
  for(auto kv : _start) {
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

  // Add start and end group cfgs to individual roadmaps
  TransitionVertex startVertices;
  TransitionVertex endVertices;
  for(auto kv : _start) {
    auto group = kv.first;
    auto rm = sg->GetGroupRoadmap(group);
    GroupCfg gcfg(rm);

    for(auto robot : group->GetRobots()) {
      const auto& path = robotPaths[robot];
      auto cfg = path.front();
      gcfg.SetRobotCfg(robot,std::move(cfg));
    }

    auto vid = AddToRoadmap(gcfg);
    startVertices.emplace_back(rm,vid);
  }

  for(auto kv : _end) {
    auto group = kv.first;
    auto rm = sg->GetGroupRoadmap(group);
    GroupCfg gcfg(rm);

    for(auto robot : group->GetRobots()) {
      const auto& path = robotPaths[robot];
      auto cfg = path.back();
      gcfg.SetRobotCfg(robot,std::move(cfg));
    }

    auto vid = AddToRoadmap(gcfg);
    endVertices.emplace_back(rm,vid);
  }

  // Save key to edge in transition map
  m_transitionMap[startVertices][endVertices] = m_interactionPaths.back().get();
}

SimultaneousMultiArmEvaluator::VID
SimultaneousMultiArmEvaluator::
AddToRoadmap(GroupCfg _cfg) {

  auto lib = this->GetMPLibrary();
  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());

  // Get appropriate roadmap
  auto group = _cfg.GetGroupRoadmap()->GetGroup();
  auto rm = sg->GetGroupRoadmap(group);

  // Move cfg to appropriate roadmap
  _cfg.SetGroupRoadmap(rm);

  // Add vertex to roadmap
  auto vid = rm->AddVertex(_cfg);

  // Make dummy group task
  GroupTask gt(group);
  for(auto robot : group->GetRobots()) {
    MPTask mt(robot);
    gt.AddTask(mt);
  }

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

  return vid;
}

SimultaneousMultiArmEvaluator::VID
SimultaneousMultiArmEvaluator::
CreateTensorProductVertex(const std::vector<GroupCfg>& _cfgs) {
  
  // Collect individual cfgs and active formations
  std::vector<std::pair<VID,Cfg>> individualCfgs;
  std::vector<Formation*> formations;

  for(const auto& cfg : _cfgs) {
    for(auto f : cfg.GetFormations()) {
      formations.push_back(f);
    }

    for(size_t i = 0; i < cfg.GetNumRobots(); i++) {
      auto vid = cfg.GetVID(i);
      individualCfgs.emplace_back(vid,cfg.GetRobotCfg(i));
    }
  }

  auto tpr = m_tensorProductRoadmap.get();
  tpr->SetAllFormationsInactive();
  auto group = tpr->GetGroup();
  
  for(auto f : formations) {
    tpr->AddFormation(f,true);
  }

  GroupCfg tensorCfg(tpr);

  for(auto cfg : individualCfgs) {
    auto robot = cfg.second.GetRobot();
    if(cfg.first != MAX_INT)
      tensorCfg.SetRobotCfg(group->GetGroupIndex(robot),cfg.first);
    else
      tensorCfg.SetRobotCfg(robot,std::move(cfg.second));
  }

  return m_tensorProductRoadmap->AddVertex(tensorCfg);
}

/*----------------------------------------------------------------------------*/

istream&
operator>>(std::istream& _is, const SimultaneousMultiArmEvaluator::TaskState) {
  return _is;
}

ostream&
operator<<(std::ostream& _os, const SimultaneousMultiArmEvaluator::TaskState) {
  return _os;
}

istream&
operator>>(std::istream& _is, const SimultaneousMultiArmEvaluator::TaskEdge) {
  return _is;
}

ostream&
operator<<(std::ostream& _os, const SimultaneousMultiArmEvaluator::TaskEdge) {
  return _os;
}

istream&
operator>>(std::istream& _is, const SimultaneousMultiArmEvaluator::ActionExtendedState) {
  return _is;
}

ostream&
operator<<(std::ostream& _os, const SimultaneousMultiArmEvaluator::ActionExtendedState) {
  return _os;
}

istream&
operator>>(std::istream& _is, const SimultaneousMultiArmEvaluator::ActionExtendedEdge) {
  return _is;
}

ostream&
operator<<(std::ostream& _os, const SimultaneousMultiArmEvaluator::ActionExtendedEdge) {
  return _os;
}
