#include "SimultaneousMultiArmEvaluator.h"

#include "Behaviors/Agents/Coordinator.h"

#include "ConfigurationSpace/Formation.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

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

  m_heuristicProb = _node.Read("heuristicProb",false,m_heuristicProb,0.,1.,
            "Probability of using heuristic in selecting direction to expand tree.");

  m_goalBias = _node.Read("goalBias",false,m_goalBias,0.,1.,
            "Probability of selecting mode towards goal.");
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

void
SimultaneousMultiArmEvaluator::
ComputeGoalBiasHeuristic() {

  // Compute mode graph distance to go
  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();

  auto c = this->GetPlan()->GetCoordinator();
  auto prob = this->GetMPProblem();
  auto decomp = prob->GetDecompositions(c->GetRobot())[0].get();

  ObjectMode goalMode;  

  for(auto st : decomp->GetGroupMotionTasks()) {
    auto gt = st->GetGroupMotionTask();
    for(auto iter = gt->begin(); iter != gt->end(); iter++) {
      for(auto& c : iter->GetGoalConstraints()) {
        auto robot = c->GetRobot();
        auto boundary = c->GetBoundary();

        if(boundary->Type() == Boundary::Space::Workspace) {
          throw RunTimeException(WHERE) << "Unsupported boundary type.";
        }

        auto dofs = boundary->GetCenter();
        Cfg cfg(robot);
        cfg.SetData(dofs);

        const auto& terrainMap = this->GetMPProblem()->GetEnvironment()->GetTerrains();
        for(const auto& terrain : terrainMap.at(robot->GetCapability())) {
          if(!terrain.InTerrain(cfg))
            continue;

          ModeInfo info(nullptr,nullptr,&terrain);
          goalMode[robot] = info;
          break;
        }
      }
    }
  }

  auto goalVID = g->GetVID(goalMode);
  if(goalVID == INVALID_VID)
    throw RunTimeException(WHERE) << "Failed to find or construct goal mode.";

  //TODO::Compute dijkstra from goal backwards and save cost to go from each vertex

  SSSPPathWeightFunction<GraphType> cost2goWeight(
    [](typename GraphType::adj_edge_iterator& _ei,
       const double _sourceDistance,
       const double _targetDistance) {
      
      //return _sourceDistance + _ei->property();
      return _sourceDistance + 1;
    }
  );

  // Compute distance to goal for each vertex in g
  m_goalBiasCosts = DijkstraSSSP(g,{goalVID},cost2goWeight).distance;

  std::vector<std::pair<double,size_t>> elems;

  for(auto kv : m_goalBiasCosts) {
    elems.emplace_back(kv.second,kv.first);
  }

  std::sort(elems.begin(),elems.end());

  for(auto elem : elems) {
    m_orderedModesToGoal.push_back(elem.second);
  }
}
/*----------------------------- Helper Functions -----------------------------*/

bool
SimultaneousMultiArmEvaluator::
Run(Plan* _plan) {

  ComputeGoalBiasHeuristic();

  Plan* plan = _plan ? _plan : this->GetPlan();
  plan->Print();

  auto prob = this->GetMPProblem();
  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();
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
    auto nextStep = ComputeMAPFSolution(g->GetVertex(mode));


    // Sample Transitions

    auto neighbors = GetModeNeighbors(mode);
    for(auto n : neighbors) {
      SampleTransition(mode, n);
    }

    // RRT Logic
    // - Qnear = Sample vertex (from heursitic)
    auto qNear = Select(mode,history,nextStep);
    // - Qnew = Extend(Qnear,history,heuristic)
    auto qNew = Extend(qNear,history,nextStep);
    if(qNew == MAX_INT)
      continue;
    // - Qbest = rewire(Qnew)
    auto qBest = Rewire(qNew,history);
    // TODO::Temp while Rewire does nothing
    qBest = qNear;

    // Add Qnew to action extended graph and connect
    auto aid = AddToActionExtendedGraph(qBest,qNew,history);

    // Check if Qnew is in a neighboring mode and create new vertex over there if so
    CheckForModeTransition(aid,history);

    // Check if Qnew is a goal configuration and update the path if so
    CheckForGoal(aid);
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

  ActionHistory initialHistory = {taskStartState.mode};
  actionStartState.ahid = AddHistory(initialHistory);

  m_historyVertices[actionStartState.ahid].insert(taskStart);

  return m_actionExtendedGraph->AddVertex(actionStartState);
}


std::pair<size_t,size_t>
SimultaneousMultiArmEvaluator::
SelectMode() {

  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();

  if(DRand() < m_goalBias) {
    for(auto mode : m_orderedModesToGoal) {
      
      auto histories = m_modeHistories[mode];
  
      if(histories.empty())
        continue;

      auto index = LRand() % histories.size();
      auto hid = histories[index];

      if(m_debug) {
        std::cout << "Selected Mode " << mode << " with history [ ";
        for(auto vid : m_actionHistories[hid]) {
          std::cout << vid << ", ";
        }
        std::cout << "]" << std::endl;
      }

      return std::make_pair(mode,hid);
    }
  }

  while(true) {
    // For now, sample random mode
    auto mode = LRand() % g->Size();
    
    auto histories = m_modeHistories[mode];
  
    if(histories.empty())
      continue;

    //temp for debugging
    if(mode == 2)
      std::cout << "HERE" << std::endl;

    auto index = LRand() % histories.size();
    auto hid = histories[index];

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

      auto iter = std::find(m_plannedInteractions.begin(),
                            m_plannedInteractions.end(),
                            pair);

      if(iter != m_plannedInteractions.end())
        continue;

      auto reverse = pair.first;
      auto roleMap = pair.second;

      // Create state
      State state;

      std::vector<RobotGroup*> passives;

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
        bool passive = true;
        
        for(auto role : f->GetRoles()) {
          auto robot = roleMap[role];
          robots.push_back(robot);
          label += ("::" + robot->GetLabel());
          passive = passive and robot->GetMultiBody()->IsPassive();
        }

        auto group = prob->AddRobotGroup(robots,label);
        if(passive)
          passives.push_back(group);

        // Add group to state
        state[group] = std::make_pair(nullptr,MAX_INT);
      }

      //Plan interaction
      for(size_t i = 0; i < m_maxAttempts; i++) {

        // Get cfg for each passive group
        for(auto group : passives) {
          auto rm = sg->GetGroupRoadmap(group);
          auto iter = rm->end();
          size_t vid = MAX_INT;
          while(iter == rm->end()) {
            vid = LRand() % rm->Size();
            iter = rm->find_vertex(vid);
          }

          state[group] = std::make_pair(rm,vid);
        }
 
        State end = state;
        bool success = is->operator()(interaction,end);

        if(success) {
          ConnectToExistingRoadmap(interaction,state,end,reverse,_source,_target);
          m_plannedInteractions.push_back(pair);
          break;
        }
      }
    }
  }

  return false;
}

void
SimultaneousMultiArmEvaluator::
ConnectToExistingRoadmap(Interaction* _interaction, State& _start, State& _end, bool _reverse,
                         size_t _sourceMode, size_t _targetMode) {

  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();

  auto startMode = g->GetVertex(_sourceMode);
  auto endMode = g->GetVertex(_targetMode);

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

  if(_reverse) {
    for(auto& kv : robotPaths) {
      std::reverse(kv.second.begin(),kv.second.end());
    }
  }

  // Add start and end group cfgs to individual roadmaps
  TransitionVertex startVertices;
  TransitionVertex endVertices;

  State start;
  State end;
  if(_reverse) {
    start = _end;
    end = _start;
  }
  else {
    start = _start;
    end = _end;
  }
    
  for(auto kv : start) {
    auto group = kv.first;
    auto rm = sg->GetGroupRoadmap(group);

    rm->SetAllFormationsInactive();
    for(auto robot : group->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive())
        continue;

      auto formation = startMode[robot].formation;
      if(formation)
        rm->AddFormation(formation);
    }

    GroupCfg gcfg(rm);

    for(auto robot : group->GetRobots()) {
      const auto& path = robotPaths[robot];
      auto cfg = path.front();
      gcfg.SetRobotCfg(robot,std::move(cfg));
    }

    auto vid = AddToRoadmap(gcfg);
    startVertices.emplace_back(rm,vid);
  }

  for(auto kv : end) {
    auto group = kv.first;
    //auto formations = kv.second.first->GetVertex(kv.second.second).GetFormations();
    auto rm = sg->GetGroupRoadmap(group);

    rm->SetAllFormationsInactive();
    for(auto robot : group->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive())
        continue;

      auto formation = endMode[robot].formation;
      if(formation)
        rm->AddFormation(formation);
    }

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
Select(size_t _modeID, size_t _history, std::unordered_map<Robot*,size_t> _heuristic) {

  auto biasKey = std::make_pair(_modeID,_history);
  auto iter = m_modeVertexBias.find(biasKey);
  
  if(iter != m_modeVertexBias.end())
    return iter->second;

  auto sample = SampleVertex(_modeID);

  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);

  auto candidates = m_historyVertices[_history];
 
  double minDistance = MAX_DBL;
  VID closest = MAX_INT;
 
  for(auto vid : candidates) {
    auto taskState = m_taskGraph->GetVertex(vid);
    auto cfg = m_tensorProductRoadmap->GetVertex(taskState.vid);
    auto distance = dm->Distance(sample,cfg);

    if(distance == std::numeric_limits<double>::infinity())
      throw RunTimeException(WHERE) << "Comapring invalid candidates.";

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
Extend(TID _qNear, size_t _history, std::unordered_map<Robot*,size_t> _heuristic) {

  auto prob = this->GetMPProblem();

  auto taskState = m_taskGraph->GetVertex(_qNear);
  auto cfg = m_tensorProductRoadmap->GetVertex(taskState.vid);

  // TODO::Use heuristic to select direction to extend
  // In the meantime, sample random vertex
  GroupCfg direction;
  if(DRand() <= m_heuristicProb)
    direction = GetHeuristicDirection(taskState.mode,_heuristic);
  else
    direction = SampleVertex(taskState.mode);

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
  /*
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
  */

  auto nearCfgs = SplitTensorProductVertex(cfg,taskState.mode);
  auto directionCfgs = SplitTensorProductVertex(direction,taskState.mode);

  // Find nearest neighbor to direction for each group roadmap
  std::vector<VID> neighbors;

  for(size_t i = 0; i < nearCfgs.size(); i++) {

    // Create group cfgs within this roadmap
    GroupCfg start = nearCfgs[i];
    GroupCfg d = directionCfgs[i];

    auto rm = start.GetGroupRoadmap();
    auto vid = rm->GetVID(start);

    // Check if this is a passive group
    auto group = rm->GetGroup();
    bool passive = true;
    for(auto robot : group->GetRobots()) {
      passive = passive and robot->GetMultiBody()->IsPassive();
    }

    // Leave cfg stationary if it is
    if(passive) { 
      neighbors.push_back(vid);
      continue;
    }

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

  for(size_t i = 0; i < nearCfgs.size(); i++) {
    auto rm = nearCfgs[i].GetGroupRoadmap();
    auto vid = neighbors[i];

    auto gcfg = rm->GetVertex(vid);

    for(auto robot : rm->GetGroup()->GetRobots()) {
      auto individualCfg = gcfg.GetRobotCfg(robot);
      qNew.SetRobotCfg(robot,std::move(individualCfg));
    }
  }

  if(qNew == cfg)
    return MAX_INT;

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
  
    double cost = lpOut.m_edge.first.GetIntermediates().size();

    // Add vertex to TPR
    auto vid = m_tensorProductRoadmap->AddVertex(qNew);

    // Add edge to TPR
    m_tensorProductRoadmap->AddEdge(taskState.vid,vid,lpOut.m_edge.first);

    // Add vertex to task graph
    TaskState newState;
    newState.vid = vid; 
    newState.mode = taskState.mode;

    auto tid = m_taskGraph->AddVertex(newState);

    // Add edge to task graph
    TaskEdge edge;
    edge.cost = cost;
    m_taskGraph->AddEdge(_qNear,tid,edge);

    // Mark history of this vertex
    m_historyVertices[_history].insert(tid);
    return tid;
  }

  return MAX_INT;
}

    
GroupCfg
SimultaneousMultiArmEvaluator::
GetHeuristicDirection(size_t _modeID, std::unordered_map<Robot*,size_t> _heuristic) {

  auto prob = this->GetMPProblem();
  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();
  auto sog = sg->GetSingleObjectModeGraph();

  auto mode = g->GetVertex(_modeID);

  std::map<Robot*,RobotGroup*> sourceObjectGroups;
  std::map<Robot*,RobotGroup*> targetObjectGroups;

  std::map<Robot*,const Terrain*> sourceObjectTerrains;
  std::map<Robot*,const Terrain*> targetObjectTerrains;

  // Collect current robot group of each object
  for(auto kv : mode) {
    auto object = kv.first;
    auto info = kv.second;

    std::vector<Robot*> robots;

    //if(info.formation) {
    //  for(auto robot : info.formation->GetRobots()) {
    //    robots.push_back(robot);
    //  }
    if(info.robot) {
      robots = {object,info.robot};
    }
    else {
      robots = {object};
      sourceObjectTerrains[object] = info.terrain;
    }

    auto group = prob->AddRobotGroup(robots,"temp");
    sourceObjectGroups[object] = group;
  }

  // Collect target robot group of each object
  for(auto kv : _heuristic) {
    auto object = kv.first;
    auto vid = kv.second;
    auto info = sog->GetVertex(vid);

    std::vector<Robot*> robots;

    //if(info.formation) {
    //  for(auto robot : info.formation->GetRobots()) {
    //    robots.push_back(robot);
    //  }
    if(info.robot) {
      robots = {object,info.robot};
    }
    else {
      robots = {object};
      targetObjectTerrains[object] = info.terrain;
    }

    auto group = prob->AddRobotGroup(robots,"temp");
    targetObjectGroups[object] = group;
  }

  // Check if any transitions match this switch
  std::map<Robot*,TransitionVertex> directions;
  for(auto kv : _heuristic) {
    auto object = kv.first;
    auto sourceGroup = sourceObjectGroups[object];
    auto targetGroup = targetObjectGroups[object];

    TransitionVertex direction;

    for(auto kv : m_transitionMap) {
      // Check if the group is the same
      auto tv = kv.first;
      bool contained = false;

      for(auto pair : tv) {
        if(sourceGroup == pair.first->GetGroup()) {
          if(sourceGroup->Size() == 1) {
            // Check that terrain matches
            auto cfg = pair.first->GetVertex(pair.second).GetRobotCfg(object);
            if(!sourceObjectTerrains[object]->InTerrain(cfg))
              break;
          }

          contained = true;
          break;
        }
      }

      if(!contained)
        continue;

      for(auto kv2 : kv.second) {
        auto tv2 = kv2.first;
        bool contained = false;

        for(auto pair : tv2) {
          if(targetGroup == pair.first->GetGroup()) {
            if(targetGroup->Size() == 1) {
              // Check that terrain matches
              auto cfg = pair.first->GetVertex(pair.second).GetRobotCfg(object);
              if(!targetObjectTerrains[object]->InTerrain(cfg))
                break;
            }

            contained = true;
            break;
          }
        }

        if(!contained)
          continue;

        direction = tv;
        break;
      }

      if(direction == tv) 
        break;
    }

    directions[object] = direction;
  }

  // Set active formations in TPR
  m_tensorProductRoadmap->SetAllFormationsInactive();

  std::vector<Formation*> formations;
  for(auto kv : mode) {
    auto info = kv.second;
    if(!info.formation)
      continue;

    m_tensorProductRoadmap->AddFormation(info.formation);
  }

  // Create direction vertex
  GroupCfg cfg(m_tensorProductRoadmap.get());
  for(auto kv : directions) {
    for(auto pair : kv.second) {
      auto rm = pair.first;
      auto vid = pair.second;
      auto gcfg = rm->GetVertex(vid);

      for(auto robot : rm->GetGroup()->GetRobots()) {
        auto rcfg = gcfg.GetRobotCfg(robot);
        cfg.SetRobotCfg(robot,std::move(rcfg));
      }
    }
  }

  return cfg;
  
}

SimultaneousMultiArmEvaluator::TID
SimultaneousMultiArmEvaluator::
Rewire(VID _qNew, size_t _history) {
  // TODO::Perform the rewire action
  return _qNew;
}
    
size_t
SimultaneousMultiArmEvaluator::
AddToActionExtendedGraph(TID _qBest, TID _qNew, size_t _history) {

  ActionExtendedState qNew;
  qNew.vid = _qNew;
  qNew.ahid = _history;

  auto newAID = m_actionExtendedGraph->AddVertex(qNew);

  ActionExtendedState qBest;
  qBest.vid = _qBest;
  qBest.ahid = _history;

  auto bestAID = m_actionExtendedGraph->GetVID(qBest);

  auto taskEdge = m_taskGraph->GetEdge(_qBest,_qNew);
  ActionExtendedEdge actionEdge;
  actionEdge.cost = taskEdge.cost;

  m_actionExtendedGraph->AddEdge(bestAID,newAID,actionEdge);

  return newAID;
}

void
SimultaneousMultiArmEvaluator::
CheckForModeTransition(size_t _aid, size_t _history) {
  
  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();

  auto actionExtendedState = m_actionExtendedGraph->GetVertex(_aid);
  auto qNew = actionExtendedState.vid;

  auto taskState = m_taskGraph->GetVertex(qNew);
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

    // Check that mode matches cfg
    {
      auto checkCfg = m_tensorProductRoadmap->GetVertex(newState.vid);
      for(auto f : checkCfg.GetFormations()) {
        bool match = false;
        for(auto kv : newMode) {
          auto info = kv.second;
          if(f == info.formation) {
            match = true;  
            break;
          }
        }
        if(!match) {
          throw RunTimeException(WHERE) << "Mismatched formations in mode and switch cfg.";
        }
      }
      for(auto kv : newMode) {
        auto info = kv.second;
        bool match = false;
        for(auto f : checkCfg.GetFormations()) {
          if(f == info.formation) {
            match = true;  
            break;
          }
        }
        if(!match) {
          throw RunTimeException(WHERE) << "Mismatched formations in mode and switch cfg.";
        }
      }
    }

    auto modeID = g->GetVID(newMode);
    newState.mode = modeID;

    auto newHistory = m_actionHistories[_history];
    newHistory.push_back(modeID);
    auto hid = AddHistory(newHistory);

    auto newTaskVID = m_taskGraph->AddVertex(newState);
    m_historyVertices[hid].insert(newTaskVID);
    m_taskGraph->AddEdge(qNew,newTaskVID,edge);

    ActionExtendedState actState;
    actState.vid = newTaskVID;
    actState.ahid = hid;

    auto newAID = m_actionExtendedGraph->AddVertex(actState);

    ActionExtendedEdge actEdge;
    actEdge.cost = edge.cost;

    m_actionExtendedGraph->AddEdge(_aid,newAID,actEdge);
  }
}

void
SimultaneousMultiArmEvaluator::
CheckForGoal(size_t _aid) {
  
  auto c = this->GetPlan()->GetCoordinator();
  auto prob = this->GetMPProblem();
  auto decomp = prob->GetDecompositions(c->GetRobot())[0].get();

  auto actState = m_actionExtendedGraph->GetVertex(_aid);
  auto taskState = m_taskGraph->GetVertex(actState.vid);
  auto vid = taskState.vid;

  auto gcfg = m_tensorProductRoadmap->GetVertex(vid);

  // Check if cfg satisfies all of the constraints in the decomp
  for(auto st : decomp->GetGroupMotionTasks()) {
    auto gt = st->GetGroupMotionTask();
    for(auto iter = gt->begin(); iter != gt->end(); iter++) {
      for(auto& c : iter->GetGoalConstraints()) {
        auto robot = c->GetRobot();
        auto cfg = gcfg.GetRobotCfg(robot);
        if(!c->Satisfied(cfg))
          return;
      }
    }
  }

  std::cout << "FOUND GOAL STATE AT " << gcfg.PrettyPrint() << std::endl;
  exit(1);
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

size_t
SimultaneousMultiArmEvaluator::
AddHistory(const ActionHistory& _history) {

  // Check if history exists already
  size_t i = 0;
  for(i=0; i < m_actionHistories.size(); i++) {
    if(_history == m_actionHistories[i])
      return i;
  }

  // Add new hisotry to set of histories
  m_actionHistories.push_back(_history);

  // Create entry for initial mode and history
  m_modeHistories[_history.back()].push_back(i);

  return i;
}

/*-------------------------- Heuristic Functions -----------------------------*/

std::unordered_map<Robot*,size_t>
SimultaneousMultiArmEvaluator::
ComputeMAPFSolution(ObjectMode _objectMode) {


  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetSingleObjectModeGraph();

  // Compute each object's starting vertex
  for(auto kv : _objectMode) {
    auto object = kv.first;
    auto info = kv.second;
    info.formation = nullptr;
    auto start = g->GetVID(info);

    m_heuristicStarts[object] = start;
  }

  // Compute each object's goal vertex
  if(m_heuristicGoals.empty()) {
    auto c = this->GetPlan()->GetCoordinator();
    auto prob = this->GetMPProblem();
    auto decomp = prob->GetDecompositions(c->GetRobot())[0].get();
    const auto& terrainMap = this->GetMPProblem()->GetEnvironment()->GetTerrains();

    for(auto st : decomp->GetGroupMotionTasks()) {
      auto gt = st->GetGroupMotionTask();
      for(auto iter = gt->begin(); iter != gt->end(); iter++) {
        for(auto& c : iter->GetGoalConstraints()) {
          auto object = c->GetRobot();
          auto boundary = c->GetBoundary();
          auto center = boundary->GetCenter();

          Point3d p(center[0],center[1],center[2]);

          for(const auto& terrain : terrainMap.at(object->GetCapability())) {
            if(!terrain.InTerrain(p))
              continue;
    
            ModeInfo info(nullptr,nullptr,&terrain);
            auto goal = g->GetVID(info);
            m_heuristicGoals[object] = goal;
          }
        }
      }
    }
  }

  // Configure CBS Functions
  CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution> lowLevel(
    [this](CBSNodeType& _node, Robot* _task) {
      return LowLevelPlanner(_node,_task);
    }
  );

  CBSValidationFunction<Robot,CBSConstraint,CBSSolution> validation(
    [this](CBSNodeType& _node) {
      return this->ValidationFunction(_node);
    }
  );

  CBSCostFunction<Robot,CBSConstraint,CBSSolution> cost(
    [this](CBSNodeType& _node) {
      return this->CostFunction(_node);
    }
  );

  CBSSplitNodeFunction<Robot,CBSConstraint,CBSSolution> splitNode(
    [this](CBSNodeType& _node, std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
           CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
           CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost) {
      return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
    }
  );

  std::vector<Robot*> objects = sg->GetObjects();

  CBSNodeType solution = CBS(objects,validation,splitNode,lowLevel,cost);

  if(m_debug) {
    std::cout << "Heuristic Paths" << std::endl;
    for(auto kv : solution.solutionMap) {
      auto robot = kv.first;
      auto path = *kv.second;
      std::cout << "\t" << robot->GetLabel() << ": ";
      for(auto vid : path) {
        std::cout << vid << ", ";
      }
      std::cout << std::endl;
    }
  }

  // Extract next step for each object

  std::unordered_map<Robot*,size_t> nextStep;

  for(auto kv : solution.solutionMap) {
    auto robot = kv.first;
    auto path = *kv.second;
    auto vid = path.size() > 1 ? path[1] : path[0];
    //auto info = g->GetVertex(vid);
    nextStep[robot] = vid;
  }

  return nextStep;
}

bool
SimultaneousMultiArmEvaluator::
LowLevelPlanner(CBSNodeType& _node, Robot* _robot) {
  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetSingleObjectModeGraph();

  auto h = std::shared_ptr<HeuristicSearch>(new HeuristicSearch(_robot));

  auto constraints = _node.constraintMap[_robot];

  // Get start and goal of robot in terms of vid in g
  auto start = m_heuristicStarts[_robot];
  size_t goal = m_heuristicGoals[_robot];

  SSSPPathWeightFunction<SingleObjectModeGraph> cost2goWeight(
    [](typename SingleObjectModeGraph::adj_edge_iterator& _ei,
       const double _sourceDistance,
       const double _targetDistance) {
      
      return _sourceDistance + _ei->property();
    }
  );

  // Compute distance to goal for each vertex in g
  auto dist2go = DijkstraSSSP(g,{goal},cost2goWeight).distance;  

  auto startVertex = std::make_pair(start,0);
  auto startVID = h->AddVertex(startVertex);

  SSSPTerminationCriterion<HeuristicSearch> termination(
    [goal](typename HeuristicSearch::vertex_iterator& _vi,
           const SSSPOutput<HeuristicSearch>& _sssp) {
      return goal == _vi->property().first ? SSSPTermination::EndSearch
                                           : SSSPTermination::Continue;
    }
  );

  SSSPPathWeightFunction<HeuristicSearch> weight(
    [constraints,h](typename HeuristicSearch::adj_edge_iterator& _ei,
       const double _sourceDistance,
       const double _targetDistance) {
     
      auto source = h->GetVertex(_ei->source()).first;
      auto target = h->GetVertex(_ei->target()).first;

      auto timestep = h->GetVertex(_ei->source()).second;

      auto edgeConstraint = std::make_pair(std::make_pair(source,target),timestep);
      auto vertexConstraint = std::make_pair(std::make_pair(target,target),timestep+1);

      if(constraints.count(edgeConstraint) or constraints.count(vertexConstraint))
        return std::numeric_limits<double>::infinity();

      return _sourceDistance + _ei->property();
    }
  );

  SSSPHeuristicFunction<HeuristicSearch> heuristic(
    [dist2go](const HeuristicSearch* _h, 
       typename HeuristicSearch::vertex_descriptor _source,
       typename HeuristicSearch::vertex_descriptor _target) {

      // Distance to go heuristic
      auto vertex = _h->GetVertex(_target);
      double toGo = dist2go.at(vertex.first);

      // Subtract epsilon from any neighboring vertex that is not waiting in place
      // to encourage forward movement
      if(_h->GetVertex(_source).first != vertex.first) {
        //toGo -= std::numeric_limits<double>::epsilon();
        toGo -= .01;
      }
      return std::max(0.0,toGo);
    }
  );

  SSSPNeighborsFunction<HeuristicSearch> neighbors(
    [g](HeuristicSearch* _h, typename HeuristicSearch::vertex_descriptor _vid) {
      auto vertex = _h->GetVertex(_vid);
      auto gvid = vertex.first;
      auto timestep = vertex.second;
      
      auto vit = g->find_vertex(gvid);

      for(auto eit = vit->begin(); eit != vit->end(); eit++) {
        auto target = eit->target();
        auto neighbor = std::make_pair(target,timestep+1);
        auto edge = eit->property();

        auto nvid = _h->AddVertex(neighbor);
        _h->AddEdge(_vid,nvid,edge);
      } 
    }
  );

  std::vector<size_t> starts = {startVID};
  std::vector<size_t> goals = {goal};

  auto sssp = AStarSSSP(h.get(),starts,goals,weight,heuristic,neighbors,termination);

  // Check that a path was found
  const size_t last = sssp.ordering.back();
  if(h->GetVertex(last).first != goal) {
    if(m_debug) {
      std::cout << "Failed to find a path for " << _robot->GetLabel() << std::endl;
    }
    return false;
  }

  // Reconstruct the path
  std::vector<size_t> path = {h->GetVertex(last).first};
  auto current = last;
  do {
    current = sssp.parent.at(current);
    path.push_back(h->GetVertex(current).first);
  } while(current != startVID);
  std::reverse(path.begin(),path.end());

  // Save path in solution
  if(!_node.solutionMap[_robot])
    _node.solutionMap[_robot] = new vector<size_t>();
  *(_node.solutionMap[_robot]) = path;

  return true;
}

std::vector<std::pair<Robot*,SimultaneousMultiArmEvaluator::CBSConstraint>>
SimultaneousMultiArmEvaluator::
ValidationFunction(CBSNodeType& _node) {

  size_t maxTimestep = 0;
  for(auto kv : _node.solutionMap) {
    maxTimestep = std::max(maxTimestep,kv.second->size());
  }

  //TODO::Currently only allows capacity of 1, need to update to check for edge conflicts in loop and build 
  //      capacity counts of vertices at each timestep that's checked at the end of the timestep loop iterations.

  for(size_t i = 0; i < maxTimestep; i++) {
    for(auto iter1 = _node.solutionMap.begin(); iter1 != _node.solutionMap.end(); iter1++) {

      auto object1 = iter1->first;
      auto path1 = *(iter1->second);
      auto s1 = std::min(i,path1.size()-1);
      auto t1 = std::min(i+1,path1.size()-1);
      auto source1 = path1[s1];
      auto target1 = path1[t1];

      auto iter2 = iter1;
      iter2++;
      for(;iter2 != _node.solutionMap.end(); iter2++) {
        auto object2 = iter2->first;
        auto path2 = *(iter2->second);
        auto s2 = std::min(i,path1.size()-1);
        auto t2 = std::min(i+1,path1.size()-1);
        auto source2 = path2[s2];
        auto target2 = path2[t2];

        if(source1 == source2) {
          if(m_debug) {
            std::cout << "Found vertex conflict at timestep " << i
                      << " between " << object1->GetLabel()
                      << " and " << object2->GetLabel() 
                      << std::endl;
          }

          auto constraint = std::make_pair(std::make_pair(source1,source1),i);
          std::vector<std::pair<Robot*,CBSConstraint>> constraints;
          constraints.push_back(std::make_pair(object1,constraint));
          constraints.push_back(std::make_pair(object2,constraint));

          return constraints;
        }

        if(source1 == target2 or target1 == source2) {
          if(m_debug) {
            std::cout << "Found edge conflict at timestep " << i
                      << " between " << object1->GetLabel()
                      << " and " << object2->GetLabel() 
                      << std::endl;
          }

          auto constraint1 = std::make_pair(std::make_pair(source1,target1),i);
          auto constraint2 = std::make_pair(std::make_pair(source2,target2),i);
          //auto constraint1 = std::make_pair(std::make_pair(target1,target1),i+1);
          //auto constraint2 = std::make_pair(std::make_pair(target2,target2),i+1);
          std::vector<std::pair<Robot*,CBSConstraint>> constraints;
          constraints.push_back(std::make_pair(object1,constraint1));
          constraints.push_back(std::make_pair(object2,constraint2));

          return constraints;
        }
      }
    }
  }
  
  return {};
}

double
SimultaneousMultiArmEvaluator::
CostFunction(CBSNodeType& _node) {

  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetSingleObjectModeGraph();

  double cost = 0;
  for(auto kv : _node.solutionMap) {
    //cost = std::max(cost,double(kv.second->size()));
    double pathLength = 0;
    const auto& path = *kv.second;
    for(size_t i = 1; i < path.size(); i++) {
      auto source = path[i-1];
      auto target = path[i];
      auto edge = g->GetEdge(source,target);
      pathLength += edge;
    }
    cost += pathLength;
  }

  return cost;
}

std::vector<SimultaneousMultiArmEvaluator::CBSNodeType>
SimultaneousMultiArmEvaluator::
SplitNodeFunction(CBSNodeType& _node,
                  std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
                  CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
                  CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost) {

  std::vector<CBSNodeType> newNodes;

  for(auto pair : _constraints) {
    // Unpack constraint info
    auto task = pair.first;
    auto constraint = pair.second;

    // Copy parent node
    CBSNodeType child = _node;
  
    // Add new constraint
    child.constraintMap[task].insert(constraint);

    // Replan tasks affected by constraint. Skip if no valid replanned path is found
    if(!_lowLevel(child,task)) 
      continue;

    // Update the cost and add to set of new nodes
    double cost = _cost(child);
    child.cost = cost;
    newNodes.push_back(child);
  }

  return newNodes;
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
