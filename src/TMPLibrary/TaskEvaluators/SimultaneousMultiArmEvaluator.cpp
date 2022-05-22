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

  //Run();

}

void
SimultaneousMultiArmEvaluator::
ComputeGoalBiasHeuristic() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ComputeGoalBiasHeuristic");

  // Compute mode graph distance to go
  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();

  GraphType* inverse = new GraphType(nullptr);

  for(auto vit = g->begin(); vit != g->end(); vit++) {
    auto vid = inverse->AddVertex(vit->property());
    if(vid != vit->descriptor())
      throw RunTimeException(WHERE) << "Mismatched graphs.";
  }

  for(auto vit = g->begin(); vit != g->end(); vit++) {
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      auto source = eit->target();
      auto target = eit->source();
      auto edge = eit->property();
  
      inverse->AddEdge(source,target,edge);
    }
  }

  auto c = this->GetPlan()->GetCoordinator();
  auto prob = this->GetMPProblem();
  auto decomp = prob->GetDecompositions(c->GetRobot())[0].get();

  //ObjectMode goalMode;  

  // TODO::Create goal mode options
  std::vector<ObjectMode> goalModes = {ObjectMode()};
  std::set<SemanticTask*> seen;
  for(auto st : decomp->GetGroupMotionTasks()) {
    auto parent = st->GetParent();
    if(seen.count(parent))
      continue;

    if(parent->GetSubtaskRelation() == SemanticTask::SubtaskRelation::XOR)
      seen.insert(parent);

    std::vector<ObjectMode> newGoalModes;

    for(auto child : parent->GetSubtasks()) {

      // Gather new ModeInfo for the task
      auto gt = child->GetGroupMotionTask();
      std::vector<std::pair<Robot*,ModeInfo>> newInfos;
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
            //goalMode[robot] = info;
            newInfos.emplace_back(robot,info);
            break;
          }
        } 
      }

      // Add Mode Infos to the goal modes
      for(auto gm : goalModes) {
        // Grab old mode and add infos to it
        for(auto pair : newInfos) {
          gm[pair.first] = pair.second;
        }

        // Add updated mode to new goal modes
        newGoalModes.push_back(gm);
      }
    }

    goalModes = newGoalModes;
  }

  if(m_debug) {
    std::cout << "Single object mode graph details" << std::endl;
    for(auto vit = g->begin(); vit != g->end(); vit++) {
      std::cout <<  "Vertex: " << vit->descriptor() << std::endl;
      auto vertex = vit->property();
      for(auto kv : vertex) {
        auto robot = kv.second.robot;
        std::cout << kv.first->GetLabel() << " ";
        if(robot) {
          std::cout << "is held by: " << robot->GetLabel() << std::endl;
        }
        else {
          auto boundary = kv.second.terrain->GetBoundaries()[0].get();
          std::cout << "is at " << boundary->GetCenter() << std::endl;
        }
      }
      for(auto eit = vit->begin(); eit != vit->end(); eit++) {
        std::cout << "\t-> " << eit->target() << std::endl;
      }
    }
    std::cout << "Inverse single object mode graph details" << std::endl;
    for(auto vit = inverse->begin(); vit != inverse->end(); vit++) {
      std::cout <<  "Vertex: " << vit->descriptor() << std::endl;
      auto vertex = vit->property();
      for(auto kv : vertex) {
        auto robot = kv.second.robot;
        std::cout << kv.first->GetLabel() << " ";
        if(robot) {
          std::cout << "is held by: " << robot->GetLabel() << std::endl;
        }
        else {
          auto boundary = kv.second.terrain->GetBoundaries()[0].get();
          std::cout << "is at " << boundary->GetCenter() << std::endl;
        }
      }
      for(auto eit = vit->begin(); eit != vit->end(); eit++) {
        std::cout << "\t-> " << eit->target() << std::endl;
      }
    }
  }


  std::vector<size_t> goalVIDs;
  for(auto gm : goalModes) {
    auto goalVID = g->GetVID(gm);
    if(goalVID != MAX_INT and goalVID != INVALID_VID)
      goalVIDs.push_back(goalVID);
  }

  if(goalVIDs.empty())
    throw RunTimeException(WHERE) << "Failed to find or construct goal mode.";

  //auto goalVID = g->GetVID(goalMode);
  //if(goalVID == INVALID_VID)
  //  throw RunTimeException(WHERE) << "Failed to find or construct goal mode.";

  // Compute dijkstra from goal backwards and save cost to go from each vertex

  SSSPPathWeightFunction<GraphType> cost2goWeight(
    [](typename GraphType::adj_edge_iterator& _ei,
       const double _sourceDistance,
       const double _targetDistance) {
      
      //return _sourceDistance + _ei->property();
      return _sourceDistance + 1;
    }
  );

  // Compute distance to goal for each vertex in g
  m_goalBiasCosts = DijkstraSSSP(inverse,goalVIDs,cost2goWeight).distance;

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
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");

  ComputeGoalBiasHeuristic();

  Plan* plan = _plan ? _plan : this->GetPlan();
  plan->Print();

  auto prob = this->GetMPProblem();
  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  //auto g = sg->GetObjectModeGraph();
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

    // Compute Heuristic
    auto nextStep = ComputeMAPFSolution(mode);

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
    auto qBest = Rewire(qNew,qNear,history);
    if(qBest == MAX_INT)
      qBest = qNear;

    // Add Qnew to action extended graph and connect
    auto aid = AddToActionExtendedGraph(qBest,qNew,history);

    // Check if Qnew is in a neighboring mode and create new vertex over there if so
    bool foundGoal = CheckForModeTransition(aid,history);

    // Check if Qnew is a goal configuration and update the path if so
    if(foundGoal or CheckForGoal(aid)) {
      //TODO::Save plan in this->GetPlan();
      stats->SetStat("Success",1);
      stats->SetStat("Steps",i);
      return true;
    }
  }

  stats->SetStat("Success",0);
  stats->SetStat("Steps",m_maxIters);
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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SelectMode");

  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();

  if(DRand() < m_goalBias) {
    for(auto mode : m_orderedModesToGoal) {
     
      // TODO::Allow for random tie breaks 
      auto histories = m_modeHistories[mode];
  
      if(histories.empty())
        continue;

      //auto index = LRand() % histories.size();
      size_t index = 0;
      auto hid = histories[index];

      if(m_debug) {
        std::cout << "Selected (Heuristic) Mode " << mode << " with history [ ";
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
      std::cout << "Selected (Random) Mode " << mode << " with history [ ";
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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GetModeNeighbors");

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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SampleTransition");

  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto prob = this->GetMPProblem();
  auto sg = static_cast<ObjectCentricModeGraph*>(
                this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetObjectModeGraph();
  auto c = plan->GetCoordinator();

  auto edge = g->GetEdge(_source, _target);

  // Set all robots to virtual
  for(auto& robot : prob->GetRobots()) {
    robot->SetVirtual(true);
  } 

  // Plan each interaction
  bool fullSuccess = true;

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

      // Set relevant robots non virtual
      for(auto kv : state) {
        auto group = kv.first;
        for(auto robot : group->GetRobots()) {
          robot->SetVirtual(false);
        }
      }

      //Plan interaction
      bool success = false;
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
        success = is->operator()(interaction,end);

        if(success and ConnectToExistingRoadmap(interaction,state,end,reverse,_source,_target)) {
          m_plannedInteractions.push_back(pair);
          break;
        }
        else {
          success = false;
        }
      }

      fullSuccess = success and fullSuccess;

      if(m_debug) {
        if(!success) {
          std::cout << "Failed to find interaction for " << interaction->GetLabel()
                    << "with ";
          for(auto kv : state) {
            std::cout << kv.first->GetLabel() << " ";
          }
          std::cout << std::endl;
          std::cout << std::endl;
        }
        else {
          std::cout << "Found interaction for " << interaction->GetLabel()
                    << "with ";
          for(auto kv : state) {
            std::cout << kv.first->GetLabel() << " ";
          }
          std::cout << std::endl;
          std::cout << std::endl;
        }
      }
          
      // Set relevant robots back to virtual
      for(auto kv : state) {
        auto group = kv.first;
        for(auto robot : group->GetRobots()) {
          robot->SetVirtual(false);
        }
      }
    }
  }

  if(m_debug and fullSuccess) {
    std::cout << "Found full set of interactions from " 
              << _source << "->" << _target << std::endl;
  }

  for(auto& robot : prob->GetRobots()) {
    if(robot.get() != c->GetRobot()) {
      robot->SetVirtual(false);
    }
  } 
  return false;
}

bool
SimultaneousMultiArmEvaluator::
ConnectToExistingRoadmap(Interaction* _interaction, State& _start, State& _end, bool _reverse,
                         size_t _sourceMode, size_t _targetMode) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ConnectToExistingRoadmap");

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
    if(vid != MAX_INT) {
      startVertices.emplace_back(rm,vid);
    }
    else {
      for(auto pair : startVertices) {
        auto group = pair.first->GetGroup();
        bool passive = true;
        for(auto robot : group->GetRobots()) {
          if(!robot->GetMultiBody()->IsPassive()) {
            passive = false;
            break;
          }
        }
        if(!passive)
          pair.first->DeleteVertex(pair.second);
      }
      return false;
    }
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
    if(vid != MAX_INT) {
      endVertices.emplace_back(rm,vid);
    }
    else {
      for(auto pair : endVertices) {
        bool passive = true;
        for(auto robot : group->GetRobots()) {
          if(!robot->GetMultiBody()->IsPassive()) {
            passive = false;
            break;
          }
        }
        if(!passive)
          pair.first->DeleteVertex(pair.second);
      }
      for(auto pair : startVertices) {
        bool passive = true;
        for(auto robot : group->GetRobots()) {
          if(!robot->GetMultiBody()->IsPassive()) {
            passive = false;
            break;
          }
        }
        if(!passive)
          pair.first->DeleteVertex(pair.second);
      }
      return false;
    }
  }

  // Save key to edge in transition map
  m_transitionMap[startVertices][endVertices] = m_interactionPaths.back().get();
  return true;
}

SimultaneousMultiArmEvaluator::VID
SimultaneousMultiArmEvaluator::
AddToRoadmap(GroupCfg _cfg) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::AddToRoadmap");

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

  //if(m_debug) {
    std::cout << "Connected new vid (" << vid 
              << ") to " << rm->get_degree(vid)
              << " vertices." << std::endl;
  //}
  
  bool passive = true;
  for(auto robot : _cfg.GetRobots()) {
    if(!robot->GetMultiBody()->IsPassive()) {
      passive = false;
      break;
    }
  }

  if(!passive and rm->get_degree(vid) == 0) {
    if(m_debug) {
      std::cout << "Could not connect interaction to roadmap" << std::endl;
    }
    rm->DeleteVertex(vid);
    return MAX_INT;
  }

  return vid;
}

SimultaneousMultiArmEvaluator::VID
SimultaneousMultiArmEvaluator::
CreateTensorProductVertex(const std::vector<GroupCfg>& _cfgs) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CreateTensorProductVertex");
  
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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Selecting");

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
      throw RunTimeException(WHERE) << "Comparing invalid candidates.";

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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SampleVertex");
  
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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Extending");

  auto taskState = m_taskGraph->GetVertex(_qNear);
  auto cfg = m_tensorProductRoadmap->GetVertex(taskState.vid);

  // Use heuristic to select direction to extend
  GroupCfg direction;
  if(DRand() <= m_heuristicProb) {
    direction = GetHeuristicDirection(taskState.mode,_heuristic);
    if(m_debug) {
      std::cout << "Choosing heuristic direction: " << direction.PrettyPrint() << std::endl;
    }
  }
  // Or sample random vertex
  else {
    direction = SampleVertex(taskState.mode);
    if(m_debug) {
      std::cout << "Choosing random direction: " << direction.PrettyPrint() << std::endl;
    }
  }

  auto computeAngle = [stats,this](GroupCfg& _cfg1, GroupCfg& _cfg2) {
    MethodTimer mt(stats,this->GetNameAndLabel() + "::Extend::ComputeAngle");

    if(_cfg1 == _cfg2)
      return 0.;

    double dot = 0;
    for(size_t i = 0; i < _cfg1.GetNumRobots(); i++) {
      auto cfg1 = _cfg1.GetRobotCfg(i);
      auto cfg2 = _cfg2.GetRobotCfg(i);

      if(cfg1.GetRobot()->GetMultiBody()->IsPassive())
        continue;

      for(size_t j = 0; j < cfg1.DOF(); j++) {
        dot += cfg1[j] * cfg2[j];
      }
      // Assuming that there is only one active robot
      double cos = dot / (cfg1.Magnitude() * cfg2.Magnitude());
      double angle = acos(cos);
      return abs(angle);
    }

    return 0.;
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
    MethodTimer mt(stats,this->GetNameAndLabel() + "::Extend::FindNearestNeighbor");

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
    for(auto robot : group->GetRobots()) {
      Cfg cfg(robot);
      for(size_t i = 0; i < cfg.DOF(); i++) {
        cfg[i] = 0.000001;
      }
      empty.SetRobotCfg(robot,std::move(cfg));
    }
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
    MethodTimer mt(stats,this->GetNameAndLabel() + "::Extend::Construction");
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
  return ExtendTaskVertices(_qNear, qNew, _history);
}

size_t
SimultaneousMultiArmEvaluator::
ExtendTaskVertices(const TID& _source, const GroupCfg& _target, size_t _history) {
    
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ExtendTaskVertices");

  auto prob = this->GetMPProblem();
  auto env = prob->GetEnvironment();

  auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);
  GroupLPOutputType lpOut(m_tensorProductRoadmap.get());
  GroupCfg col(m_tensorProductRoadmap.get());

  auto taskState = m_taskGraph->GetVertex(_source);
  auto cfg = m_tensorProductRoadmap->GetVertex(taskState.vid);

  auto isConnected = lp->IsConnected(cfg,_target,col,&lpOut,
                        env->GetPositionRes(),
                        env->GetOrientationRes(),
                        true,false,{});

  if(isConnected) {
  
    double cost = lpOut.m_edge.first.GetTimeSteps();

    // Add vertex to TPR
    auto vid = m_tensorProductRoadmap->AddVertex(_target);

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
    m_taskGraph->AddEdge(_source,tid,edge);

    // Mark history of this vertex
    m_historyVertices[_history].insert(tid);

    m_tpgDistance[tid] = m_tpgDistance[_source] + TensorProductGraphDistance(_source,tid);

    return tid;
  }

  return MAX_INT;
}

    
GroupCfg
SimultaneousMultiArmEvaluator::
GetHeuristicDirection(size_t _modeID, std::unordered_map<Robot*,size_t> _heuristic) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GetHeuristicDirection");

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

    if(direction.empty() and sourceGroup != targetGroup)
      throw RunTimeException(WHERE) << "Empty direction for " << object->GetLabel();
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
Rewire(VID _qNew, VID _qNear, size_t _history) {

  // Gather neighbors and check if any of them would even be an optimal rewire, 
  // then, in order, try to extend that way, otherwise it's too expensive.


  //auto lib = this->GetMPLibrary();
  //auto dm = lib->GetDistanceMetric(m_dmLabel);

  // Get individual roadmap cfgs
  auto newTaskState = m_taskGraph->GetVertex(_qNew);
  auto newCfg = m_tensorProductRoadmap->GetVertex(newTaskState.vid);
  auto newCfgs = SplitTensorProductVertex(newCfg,newTaskState.mode);

  const double originalDistance = m_tpgDistance[_qNew];

  // Collect neighbors 
  //std::unordered_set<size_t> neighbors;
  std::vector<std::pair<double,size_t>> neighbors;
  std::vector<std::pair<double,size_t>> backNeighbors;
  for(auto tid : m_historyVertices[_history]) {
    auto taskState = m_taskGraph->GetVertex(tid);

    if(taskState.mode != newTaskState.mode) 
      throw RunTimeException(WHERE) << "Mismatched mdoes within same history vertices.";

    // Check that there is an edge in the implicit tensor product roadmap
    auto cfg = m_tensorProductRoadmap->GetVertex(taskState.vid);
    auto cfgs = SplitTensorProductVertex(cfg,taskState.mode);
    bool isEdge = true;

    for(size_t i = 0; i < cfgs.size(); i++) {
      auto rm = cfgs[i].GetGroupRoadmap();
      auto vid1 = rm->GetVID(cfgs[i]);
      auto vid2 = rm->GetVID(newCfgs[i]);

      if(!rm->IsEdge(vid1,vid2)) {
        isEdge = false;
        break;
      }
    }

    if(!isEdge)
      continue;
    
    // Check if edge is a valid transition
    //auto newVID = ExtendTaskVertices(tid, newCfg, _history);
    //if(newVID == MAX_INT)
    //  continue;
    
    double distance = TensorProductGraphDistance(tid,_qNew);
    backNeighbors.emplace_back(distance,tid);
    distance += m_tpgDistance[tid];
    if(distance < originalDistance) {
      neighbors.emplace_back(distance,tid);
    }

  }

  // Sort neighbors by rewire distance
  std::sort(neighbors.begin(), neighbors.end());

  size_t qBest = MAX_INT;

  for(auto pair : neighbors) {
    auto tid = pair.second;
    auto newVID = ExtendTaskVertices(tid, newCfg, _history);
    if(newVID == MAX_INT)
      continue;

    qBest = tid;
    m_tpgDistance[_qNew] = pair.first;

    // Check if this was the OG edge, other wise delete the old one
    if(tid != _qNear) {
      m_tensorProductRoadmap->DeleteEdge(_qNear,_qNew);
    }

    break;
  }

  // Attempt to rewire the neighbors through qNew
  for(auto pair : backNeighbors) {
    const size_t tid = pair.second;
    const double currentDistance = m_tpgDistance[tid];
    const double newDistance = m_tpgDistance[_qNew] + pair.first;
    
    if(newDistance < currentDistance) {

      // Try to connect the vertices
      auto newVID = ExtendTaskVertices(tid, newCfg, _history);
      if(newVID == MAX_INT)
        continue;

      // Delete existing edge to kv.first
      auto pred = m_tensorProductRoadmap->GetPredecessors(tid);
      for(auto vid : pred) {
        if(m_tensorProductRoadmap->IsEdge(vid,tid))
          m_tensorProductRoadmap->DeleteEdge(vid,tid);
      }
      
      m_tpgDistance[pair.second] = newDistance;
    }
  }

  return qBest;
}
    
size_t
SimultaneousMultiArmEvaluator::
AddToActionExtendedGraph(TID _qBest, TID _qNew, size_t _history) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::AddToActionExtendedGraph");

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

bool
SimultaneousMultiArmEvaluator::
CheckForModeTransition(size_t _aid, size_t _history) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CheckForModeTransition");
  
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

    std::set<Robot*> used;

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

        auto group = rm->GetGroup();
        
        Robot* object = nullptr;
        Robot* robot = nullptr;

        for(auto r : group->GetRobots()) {
          used.insert(r);
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

      // Get cost of transition
      auto interactionPath = kv->second;
    
      for(const auto& kv : *interactionPath) {
        cost = std::max(cost,kv.second.size());
      }
    }

    // Reconstruct mode

    // Copy over modes of any non transitioning objects
    for(auto kv : g->GetVertex(mode)) {
      auto object = kv.first;
      if(used.count(object))
        continue;

      newMode[object] = kv.second;
    }

    edge.cost = cost;

    // Construct new TPV
    m_tensorProductRoadmap->SetAllFormationsInactive();

    /*    
    for(const auto& cfg : cfgs) {
      for(auto formation : cfg.GetFormations()) {
        m_tensorProductRoadmap->AddFormation(formation);
      }
    }
    */
    for(auto kv : newMode) {
      auto formation = kv.second.formation;
      if(formation)
        m_tensorProductRoadmap->AddFormation(formation);
    }

    GroupCfg newCfg(m_tensorProductRoadmap.get());
    //std::set<Robot*> used;
    for(const auto& cfg : cfgs) {
      for(auto robot : cfg.GetGroupRoadmap()->GetGroup()->GetRobots()) {
        auto indCfg = cfg.GetRobotCfg(robot);
        newCfg.SetRobotCfg(robot,std::move(indCfg));
        //used.insert(robot);
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

  /*
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
        auto vid = pair.second;
        auto cfg = rm->GetVertex(vid);
        auto group = rm->GetGroup();
        
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
    */
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
        if(!info.formation)
          continue;
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

    m_tpgDistance[newAID] = m_tpgDistance[_aid] + edge.cost;

    if(CheckForGoal(newAID))
      return true;
  }

  return false;
}

bool
SimultaneousMultiArmEvaluator::
CheckForGoal(size_t _aid) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CheckForGoal");
  
  auto c = this->GetPlan()->GetCoordinator();
  auto prob = this->GetMPProblem();
  auto decomp = prob->GetDecompositions(c->GetRobot())[0].get();

  auto actState = m_actionExtendedGraph->GetVertex(_aid);
  auto taskState = m_taskGraph->GetVertex(actState.vid);
  auto vid = taskState.vid;

  auto gcfg = m_tensorProductRoadmap->GetVertex(vid);

  // Check if cfg satisfies all of the constraints in the decomp
  std::set<SemanticTask*> satisfied;
  for(auto st : decomp->GetGroupMotionTasks()) {
    auto parent = st->GetParent();
    if(satisfied.count(parent))
      continue;

    auto gt = st->GetGroupMotionTask();

    for(auto iter = gt->begin(); iter != gt->end(); iter++) {
      for(auto& c : iter->GetGoalConstraints()) {
        auto robot = c->GetRobot();
        auto cfg = gcfg.GetRobotCfg(robot);
        if(!c->Satisfied(cfg))
          return false;
        if(parent->GetSubtaskRelation() == SemanticTask::SubtaskRelation::XOR)
          satisfied.insert(parent);
      }
    }
  }

  //if(m_debug) {
    std::cout << "FOUND GOAL STATE AT " << gcfg.PrettyPrint() << std::endl;
  //}
  stats->SetStat("PathLength",m_tpgDistance[_aid]);
  return true;
}

std::vector<GroupCfg>
SimultaneousMultiArmEvaluator::
SplitTensorProductVertex(GroupCfg _cfg, size_t _modeID) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SplitTensorProductVertex");

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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::AddHistory");

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
ComputeMAPFSolution(size_t _objectMode) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ComputeMAPFSolution");

  if(m_cachedHeuristics.find(_objectMode) != m_cachedHeuristics.end()) {
    return m_cachedHeuristics.at(_objectMode);
  }


  auto sg = static_cast<ObjectCentricModeGraph*>(
      this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetSingleObjectModeGraph();

  auto objectMode = sg->GetObjectModeGraph()->GetVertex(_objectMode);

  // Compute each object's starting vertex
  for(auto kv : objectMode) {
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
      return this->LowLevelPlanner(_node,_task);
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

  m_cachedHeuristics[_objectMode] = nextStep;

  return nextStep;
}

bool
SimultaneousMultiArmEvaluator::
LowLevelPlanner(CBSNodeType& _node, Robot* _robot) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::LowLevelPlanner");

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
      auto vertexConstraint = std::make_pair(std::make_pair(target,MAX_INT),timestep+1);

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
      //if(_h->GetVertex(_source).first != vertex.first) {
        //toGo -= std::numeric_limits<double>::epsilon();
      //  toGo -= .01;
      //}
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
  //if(!_node.solutionMap[_robot])
    _node.solutionMap[_robot] = new vector<size_t>();
  *(_node.solutionMap[_robot]) = path;

  return true;
}

std::vector<std::pair<Robot*,SimultaneousMultiArmEvaluator::CBSConstraint>>
SimultaneousMultiArmEvaluator::
ValidationFunction(CBSNodeType& _node) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

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
        auto s2 = std::min(i,path2.size()-1);
        auto t2 = std::min(i+1,path2.size()-1);
        auto source2 = path2[s2];
        auto target2 = path2[t2];

        if(source1 == source2) {
          //if(m_debug) {
          //  std::cout << "Found vertex conflict at timestep " << i
          //            << " between " << object1->GetLabel()
          //            << " and " << object2->GetLabel() 
          //            << std::endl;
          //}

          auto constraint = std::make_pair(std::make_pair(source1,MAX_INT),i);
          std::vector<std::pair<Robot*,CBSConstraint>> constraints;
          constraints.push_back(std::make_pair(object1,constraint));
          constraints.push_back(std::make_pair(object2,constraint));

          return constraints;
        }

        if(source1 == target2 or target1 == source2) {
          //if(m_debug) {
          //  std::cout << "Found edge conflict at timestep " << i
          //            << " between " << object1->GetLabel()
          //            << " and " << object2->GetLabel() 
          //            << std::endl;
          //}

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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CostFunction");

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
    //cost += pathLength;
    cost = std::max(cost,pathLength);
  }

  return cost;
}

std::vector<SimultaneousMultiArmEvaluator::CBSNodeType>
SimultaneousMultiArmEvaluator::
SplitNodeFunction(CBSNodeType& _node,
                  std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
                  CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
                  CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SplitNodeFunction");

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
    
double
SimultaneousMultiArmEvaluator::
TensorProductGraphDistance(VID _source, VID _target) {
  // Get distance in tensor product space
  auto env = this->GetMPProblem()->GetEnvironment();
  const double timeRes = env->GetTimeRes();

  // Check if distance is already saved in the graph
  if(m_tensorProductRoadmap->IsEdge(_source,_target))
    return double(m_tensorProductRoadmap->GetEdge(_source,_target).GetTimeSteps()) * timeRes;

  // Compute maximum individual robot timesteps in transitions
  auto source = m_tensorProductRoadmap->GetVertex(_source);
  auto target = m_tensorProductRoadmap->GetVertex(_target);

  const double posRes = env->GetTimeRes();
  const double oriRes = env->GetTimeRes();
  int maxSteps = 0;

  for(size_t i = 0; i < source.GetNumRobots(); i++) {
    auto cfg1 = source.GetRobotCfg(i);
    auto cfg2 = source.GetRobotCfg(i);

    // Only consider active robots
    if(cfg1.GetMultiBody()->IsPassive())
      continue;

    int steps = 0;
    cfg1.FindIncrement(cfg1,cfg2,&steps,posRes,oriRes);
    maxSteps = std::max(maxSteps,steps);
  }

  return double(maxSteps) * timeRes;
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
