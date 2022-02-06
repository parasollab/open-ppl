#include "SimultaneousMultiArmEvaluator.h"

#include "Behaviors/Agents/Coordinator.h"

#include "ConfigurationSpace/Formation.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"
#include "TMPLibrary/Solution/Plan.h"

#include <math.h>

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

  m_dmLabel = _node.Read("dmLabel",true,"",
            "Distance metric to compute diastance between group cfgs.");

  m_lpLabel = _node.Read("lpLabel",true,"",
            "Local planner to check validity within tensor product roadmap.");
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

    // RRT Logic
    // - Qnear = Sample vertex (from heursitic)
    auto qNear = Select(mode,history);
    // - Qnew = Extend(Qnear,history,heuristic)
    auto qNew = Extend(qNear,history);
    if(qNew == MAX_INT)
      continue;
    // - Qbest = rewire(Qnew)
    auto qBest = Rewire(qNew,history);

    // Add Qnew to action extended graph and connect
    AddToActionExtendedGraph(qNew,qBest,history);

    // Check if Qnew is in a neighboring mode and create new vertex over there if so
    CheckForModeTransition(qNew);

    // Check if Qnew is a goal configuration and update the path if so
    CheckForGoal(qNew);
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

SimultaneousMultiArmEvaluator::TID
SimultaneousMultiArmEvaluator::
Select(size_t _modeID, size_t _history) {

  auto sample = SampleVertex(_modeID);

  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);

  auto candidates = m_historyVertices[_history];
 
  double minDistance = MAX_DBL;
  VID closest = MAX_INT;
 
  for(auto vid : candidates) {
    auto taskState = m_taskGraph->GetVertex(vid);
    auto cfg = m_tensorProductRoadmap->GetVertex(taskState.vid);
    auto distance = dm->Distance(sample,cfg);

    if(distance < minDistance) {
      minDistance = distance;
      closest = vid;
    }      
  }

  return closest;
}

GroupCfg
SimultaneousMultiArmEvaluator::
SampleVertex(size_t _modeID) {
  
  auto env = this->GetMPProblem()->GetEnvironment();
  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();
  auto mode = g->GetVertex(_modeID);

  // Set active formations in TPR
  m_tensorProductRoadmap->SetAllFormationsInactive();

  std::vector<Formation*> formations;
  for(auto kv : mode) {
    auto info = kv.second;
    if(!info.formation)
      continue;

    m_tensorProductRoadmap->AddFormation(info.formation);
  }

  // Sample random vertex
  GroupCfg cfg(m_tensorProductRoadmap.get());
  cfg.GetRandomGroupCfg(env->GetBoundary());

  return cfg;
}

SimultaneousMultiArmEvaluator::TID
SimultaneousMultiArmEvaluator::
Extend(TID _qNear, size_t _history) {

  auto prob = this->GetMPProblem();

  auto taskState = m_taskGraph->GetVertex(_qNear);
  auto cfg = m_tensorProductRoadmap->GetVertex(taskState.vid);

  // TODO::Use heuristic to select direction to extend
  // In the meantime, sample random vertex
  auto direction = SampleVertex(taskState.mode);

  auto computeAngle = [](GroupCfg& _cfg1, GroupCfg& _cfg2) {
    double dot = 0;
    for(size_t i = 0; i < _cfg1.GetNumRobots(); i++) {
      auto cfg1 = _cfg1.GetRobotCfg(i);
      auto cfg2 = _cfg2.GetRobotCfg(i);

      for(size_t j = 0; j < cfg1.DOF(); j++) {
        dot += cfg1[j] * cfg2[j];
      }
    }

    double cos = dot / (_cfg1.Magnitude() * _cfg2.Magnitude());
    return abs(acos(cos));
  };

  // Look at tensor product neighbors of start
  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();
  auto mode = g->GetVertex(taskState.mode);

  // Collect robot groups
  std::vector<GroupRoadmapType*> roadmaps;
  std::set<Robot*> used;
 
  // Collect groups with objects 
  for(auto kv : mode) {
    auto object = kv.first;
    used.insert(object);
    auto info = kv.second;
 
    std::vector<Robot*> robots = {object};
    std::string label = object->GetLabel();
 
    if(info.robot) {
      robots.push_back(info.robot);
      label += ("::" + info.robot->GetLabel());
      used.insert(info.robot);
    }

    auto group = prob->AddRobotGroup(robots,label);
    auto rm = sg->GetGroupRoadmap(group);
    rm->SetAllFormationsInactive();
    
    if(info.formation)
      rm->AddFormation(info.formation);
  
    roadmaps.push_back(rm);
  }

  // Collect groups only containing robots and no objects
  for(auto robot : cfg.GetGroupRoadmap()->GetGroup()->GetRobots()) {
    if(used.count(robot))
      continue;

    auto group = prob->AddRobotGroup({robot},robot->GetLabel());
    auto rm = sg->GetGroupRoadmap(group);
    rm->SetAllFormationsInactive();
    roadmaps.push_back(rm);
  }

  // Find nearest neighbor to direction for each group roadmap
  std::vector<VID> neighbors;

  for(auto rm : roadmaps) {

    // Create group cfgs within this roadmap
    GroupCfg start(rm);
    GroupCfg d(rm);
    auto group = rm->GetGroup();

    for(auto robot : group->GetRobots()) {
      auto individualCfg = cfg.GetRobotCfg(robot);
      start.SetRobotCfg(robot,std::move(individualCfg));
    }

    for(auto robot : group->GetRobots()) {
      auto individualCfg = direction.GetRobotCfg(robot);
      d.SetRobotCfg(robot,std::move(individualCfg));
    }

    auto vid = rm->GetVID(start);

    // Initialize minimum neighbor as staying put
    auto vec1 = d - start;
    GroupCfg empty(rm);
    double minAngle = computeAngle(vec1,empty);
    size_t newVID = vid;

    // Check each of the neighbors in the roadmap
    auto vit = rm->find_vertex(vid);
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {

      // Grab neighbor
      auto target = eit->target();
      auto neighbor = rm->GetVertex(target);
      auto vec2 = neighbor - start;

      double angle = computeAngle(vec1,vec2);

      // Check if this is a new min angle
      if(angle >= minAngle)
        continue;

      // If yes, save
      minAngle = angle;
      newVID = target;
    }

    neighbors.push_back(newVID);
  }

  // Create tensor product vertex
  GroupCfg qNew(m_tensorProductRoadmap.get());

  for(size_t i = 0; i < roadmaps.size(); i++) {
    auto rm = roadmaps[i];
    auto vid = neighbors[i];

    auto gcfg = rm->GetVertex(vid);

    for(auto robot : rm->GetGroup()->GetRobots()) {
      auto individualCfg = gcfg.GetRobotCfg(robot);
      qNew.SetRobotCfg(robot,std::move(individualCfg));
    }
  }


  // Connect qNew to qNear
  auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);
  GroupLPOutputType lpOut(m_tensorProductRoadmap.get());
  GroupCfg col(m_tensorProductRoadmap.get());
  auto env = prob->GetEnvironment();

  auto isConnected = lp->IsConnected(cfg,qNew,col,&lpOut,
                        env->GetPositionRes(),
                        env->GetOrientationRes(),
                        true,false,{});

  if(isConnected) {
    TaskState newState;
    newState.vid = m_tensorProductRoadmap->AddVertex(qNew);
    newState.mode = taskState.mode;

    return m_taskGraph->AddVertex(newState);
  }

  return MAX_INT;
}

SimultaneousMultiArmEvaluator::TID
SimultaneousMultiArmEvaluator::
Rewire(VID _qNew, size_t _history) {
  // TODO::Perform the rewire action
  return _qNew;
}
    
void
SimultaneousMultiArmEvaluator::
AddToActionExtendedGraph(TID _qNew, TID _qBest, size_t _history) {

  ActionExtendedState qNew;
  qNew.vid = _qNew;
  qNew.ahid = _history;

  auto newAID = m_actionExtendedGraph->AddVertex(qNew);

  ActionExtendedState qBest;
  qBest.vid = _qBest;
  qBest.ahid = _history;

  auto bestAID = m_actionExtendedGraph->GetVID(qBest);

  auto taskEdge = m_taskGraph->GetEdge(_qNew,_qBest);
  ActionExtendedEdge actionEdge;
  actionEdge.cost = taskEdge.cost;

  m_actionExtendedGraph->AddEdge(newAID,bestAID,actionEdge);
}

void
SimultaneousMultiArmEvaluator::
CheckForModeTransition(TID _qNew) {
  
  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();

  auto taskState = m_taskGraph->GetVertex(_qNew);
  auto mode = taskState.mode;
  auto tpv = m_tensorProductRoadmap->GetVertex(taskState.vid);
  auto cfgs = SplitTensorProductVertex(tpv,mode);

  std::map<TransitionVertex,size_t> vertexCounts;
  std::vector<TransitionVertex> ready;

  for(auto cfg : cfgs) {
    // Check if any transition vertices include this group cfg
    auto rm = cfg.GetGroupRoadmap();
    auto vid = rm->GetVID(cfg);

    auto pair = std::make_pair(rm,vid);

    for(auto kv : m_transitionMap) {
      auto transitionVertex = kv.first;

      for(auto p : transitionVertex) {
        if(p != pair)
          continue;

        // Iterate the count of present cfgs in the transition
        vertexCounts[transitionVertex] = vertexCounts[transitionVertex] + 1;

        // Check if all cfgs are present
        if(vertexCounts[transitionVertex] == transitionVertex.size())
          ready.push_back(transitionVertex);

        break;
      }
    } 
  }

  // Collect all permutations of transition sets
  std::vector<std::vector<TransitionVertex>> combos;

  for(auto tv : ready) {
    
    // Create copies of existing sets with this one appended
    size_t size = combos.size();
    for(size_t i = 0; i < size; i++) {
      auto set = combos[i];
      set.push_back(tv);
      combos.push_back(set);
    }

    // Add new set with on ly this tv
    combos.push_back({tv});
  } 

  // Add an edge and vertex for each valid transition
  for(auto trans : combos) {

    TaskState newState;
    ObjectMode newMode;
    std::vector<GroupCfg> cfgs;

    TaskEdge edge;
    size_t cost = 0;

    for(auto tv : trans) {
      auto map = m_transitionMap[tv];
      if(map.size() > 1)
        throw RunTimeException(WHERE) << "Not currently supported.";

      auto kv = map.begin();

      // Save target cfgs
      auto target = kv->first;

      edge.transitions.emplace_back(tv,target);

      for(auto pair : target) {
        auto rm = pair.first;
        auto vid = pair.second;
        auto cfg = rm->GetVertex(vid);
        cfgs.push_back(cfg);
      }

      // Get cost of transition
      auto interactionPath = kv->second;
    
      for(const auto& kv : *interactionPath) {
        cost = std::max(cost,kv.second.size());
      }
    }

    edge.cost = cost;

    // Construct new TPV
    m_tensorProductRoadmap->SetAllFormationsInactive();
    
    for(const auto& cfg : cfgs) {
      for(auto formation : cfg.GetFormations()) {
        m_tensorProductRoadmap->AddFormation(formation);
      }
    }

    GroupCfg newCfg(m_tensorProductRoadmap.get());
    std::set<Robot*> used;
    for(const auto& cfg : cfgs) {
      for(auto robot : cfg.GetGroupRoadmap()->GetGroup()->GetRobots()) {
        auto indCfg = cfg.GetRobotCfg(robot);
        newCfg.SetRobotCfg(robot,std::move(indCfg));
        used.insert(robot);
      }
    }

    // Copy values for any robots not involved in the transition
    for(auto robot : tpv.GetGroupRoadmap()->GetGroup()->GetRobots()) {
      if(used.count(robot))
        continue;

      auto indCfg = tpv.GetRobotCfg(robot);
      newCfg.SetRobotCfg(robot,std::move(indCfg));
    }

    // Add new cfg to tpv and save in task state
    auto newVID = m_tensorProductRoadmap->AddVertex(newCfg);
    newState.vid = newVID;

    // Reconstruct mode

    // Copy over modes of any non transitioning objects
    for(auto kv : g->GetVertex(mode)) {
      auto object = kv.first;
      if(used.count(object))
        continue;

      newMode[object] = kv.second;
    }
    
    for(auto tv : trans) {
      auto map = m_transitionMap[tv];
      if(map.size() > 1)
        throw RunTimeException(WHERE) << "Not currently supported.";

      auto kv = map.begin();

      // Collect new modes
      auto target = kv->first;

      for(auto pair : target) {
        auto rm = pair.first;
        auto group = rm->GetGroup();
        auto vid = pair.second;
        auto cfg = rm->GetVertex(vid);
        
        Robot* object = nullptr;
        Robot* robot = nullptr;

        for(auto r : group->GetRobots()) {
          if(r->GetMultiBody()->IsPassive()) {
            object = r;
          }
          else {
            robot = r;
          }
        }

        if(!object)
          continue;

        if(robot) {
          auto formation = cfg.GetFormations().begin();
          newMode[object] = ModeInfo(robot,*formation,nullptr);
        }
        else {
          const auto& terrainMap = this->GetMPProblem()->GetEnvironment()->GetTerrains();
          for(const auto& terrain : terrainMap.at(object->GetCapability())) {
            // If object in terrain
            auto indCfg = cfg.GetRobotCfg(object);
            if(terrain.InTerrain(indCfg)) {
              newMode[object] = ModeInfo(nullptr,nullptr,&terrain);
              break;
            }
          }
        } 
      }
    }

    auto modeID = g->GetVID(newMode);
    newState.mode = modeID;

    auto newTaskVID = m_taskGraph->AddVertex(newState);
    m_taskGraph->AddEdge(_qNew,newTaskVID,edge);
  }
}

void
SimultaneousMultiArmEvaluator::
CheckForGoal(TID _qNew) {

}

std::vector<GroupCfg>
SimultaneousMultiArmEvaluator::
SplitTensorProductVertex(GroupCfg _cfg, size_t _modeID) {

  // Look at tensor product neighbors of start
  auto prob = this->GetMPProblem();
  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();
  auto mode = g->GetVertex(_modeID);

  // Collect robot groups
  std::vector<GroupRoadmapType*> roadmaps;
  std::set<Robot*> used;
 
  // Collect groups with objects 
  for(auto kv : mode) {
    auto object = kv.first;
    used.insert(object);
    auto info = kv.second;
 
    std::vector<Robot*> robots = {object};
    std::string label = object->GetLabel();
 
    if(info.robot) {
      robots.push_back(info.robot);
      label += ("::" + info.robot->GetLabel());
      used.insert(info.robot);
    }

    auto group = prob->AddRobotGroup(robots,label);
    auto rm = sg->GetGroupRoadmap(group);
    rm->SetAllFormationsInactive();
    
    if(info.formation)
      rm->AddFormation(info.formation);
  
    roadmaps.push_back(rm);
  }

  // Collect groups only containing robots and no objects
  for(auto robot : _cfg.GetGroupRoadmap()->GetGroup()->GetRobots()) {
    if(used.count(robot))
      continue;

    auto group = prob->AddRobotGroup({robot},robot->GetLabel());
    auto rm = sg->GetGroupRoadmap(group);
    rm->SetAllFormationsInactive();
    roadmaps.push_back(rm);
  }

  std::vector<GroupCfg> splitCfgs;

  for(auto rm : roadmaps) {
    // Create group cfgs within this roadmap
    GroupCfg cfg(rm);
    auto group = rm->GetGroup();

    for(auto robot : group->GetRobots()) {
      auto individualCfg = _cfg.GetRobotCfg(robot);
      cfg.SetRobotCfg(robot,std::move(individualCfg));
    }
    
    splitCfgs.push_back(cfg);
  }

  return splitCfgs;
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
