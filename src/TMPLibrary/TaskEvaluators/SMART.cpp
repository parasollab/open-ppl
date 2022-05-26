#include "SMART.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
#include "MPProblem/MPTask.h"

#include "TMPLibrary/StateGraphs/OCMG.h"
#include "TMPLibrary/Solution/Plan.h"

/*------------------------------- Construction -------------------------------*/

SMART::
SMART() {
  this->SetName("SMART");
}

SMART::
SMART(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("SMART");

  m_maxIterations = _node.Read("maxIters",false,1000,1,MAX_INT,
        "Maximum number of iterations to run through the algorithm.");

  m_heuristicProb = _node.Read("heuristicProb",false,m_heuristicProb,0.,1.,
            "Probability of using heuristic in selecting direction to expand tree.");

  m_goalBias = _node.Read("goalBias",false,m_goalBias,0.,1.,
            "Probability of selecting mode towards goal.");

  m_dmLabel = _node.Read("dmLabel",true,"",
            "Distance metric to compute diastance between group cfgs.");

}

/*---------------------------- Initialization --------------------------------*/

void
SMART::
Initialize() {

}

/*-------------------------------- Helpers -----------------------------------*/

bool
SMART::
Run(Plan* _plan) {

  // Initialize search trees
  CreateSMARTreeRoot(); 

  for(size_t i = 0; i < m_maxIterations; i++) {
    
    auto modePair = SelectMode();
    auto modeID = modePair.first;
    auto historyID = modePair.second;

    auto heuristics = ComputeMAPFHeuristic(modeID);

    auto vertexPair = SelectVertex(modeID,historyID,heuristics.nextMode);
    auto qNear = vertexPair.first;
    auto direction = vertexPair.second;

    auto qNew = Extend(qNear,direction,modeID,historyID,heuristics.nextMode);
    if(qNew >= MAX_INT)
      continue;

    auto qBest = Rewire(qNew,modeID,historyID);

    if(qBest >= MAX_INT)
      qBest = qNear;

    bool foundGoal = CheckForModeSwitch(qNew);

    if(foundGoal or CheckForGoal(qNew)) {
      std::cout << "FOUND GOAL!!!" << std::endl;
      // For now, quit at first solution
      return true;
    }
  }

  return false;
}

void
SMART::
CreateSMARTreeRoot() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CreateSMARTreeRoots");
  
  auto coordinator = plan->GetCoordinator();
  auto sg = static_cast<OCMG*>(this->GetStateGraph(m_sgLabel).get());
  auto omg = sg->GetSingleObjectModeGraph();
  auto problem = this->GetMPProblem();

  // Initialize tree

  m_tensorProductRoadmap = std::unique_ptr<TensorProductRoadmap>(
      new TensorProductRoadmap(coordinator->GetRobot()));

  m_actionExtendedGraph = std::unique_ptr<ActionExtendedGraph>(
      new ActionExtendedGraph(coordinator->GetRobot())); 
  

  // Convert start state to root vertex and build initial mode
  Vertex root;
  Mode mode;
  
  for(auto& kv : coordinator->GetInitialRobotGroups()) {
    auto group = kv.first;
    auto rm = sg->GetGroupRoadmap(group);

    // Get vid from rm of initial cfg
    GroupCfg gcfg(rm);

    for(auto robot : group->GetRobots()) {
      auto cfg = problem->GetInitialCfg(robot);
      gcfg.SetRobotCfg(robot,std::move(cfg));
    }

    auto vid = rm->GetVID(gcfg);
    if(vid >= MAX_INT) {
      throw RunTimeException(WHERE) << "Roadmap missing robot initial cfg.";
    }

    root.cfgs.emplace_back(rm,vid);

    // If group has a passive object, collect mode info
    Robot* passive = nullptr;
    Robot* active = nullptr;
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive()) {
        passive = robot;
      }
      else {
        active = robot;
      }
    }

    if(!passive)
      continue;

    OCMG::ModeInfo info;
    if(active) {
      info.robot = active;
    }
    else {
      for(auto kv : sg->GetTerrainVIDs()) {
        auto vids = kv.second[rm];
        if(!vids.count(vid))
          continue;

        info.terrain = kv.first;
        break;
      }
    }

    auto omgVID = omg->GetVID(info);
    if(omgVID >= MAX_INT) {
      throw RunTimeException(WHERE) << "Failed to find object mode vertex.";
    }

    mode[passive] = omgVID;
  }

  // Save starting mode
  m_modes.clear();
  m_modes.push_back(mode);
  root.modeID = 0;
  
  m_biasedModes.push_back(root.modeID);

  if(m_debug) {
    std::cout << "Starting mode: " << std::endl;;
    for(auto kv : mode) {
      std::cout << "\t" << kv.first->GetLabel() << " : " << kv.second << std::endl;
    }
  }

  // Save root vertex in tree
  auto rootVID = m_tensorProductRoadmap->AddVertex(root);
 
  ActionHistory history = {root.modeID};
  m_actionHistories.push_back(history);

  m_modeHistories[root.modeID] = {0};
 
  ActionExtendedState aes;
  aes.vid = rootVID;
  aes.ahid = {0};

  m_actionExtendedGraph->AddVertex(aes);

  m_historyVIDs[0].insert(rootVID);
}

std::pair<size_t,size_t>
SMART::
SelectMode() {

  size_t modeID = MAX_INT;
  size_t historyID = MAX_INT;

  // With m_modeBias probability, return mode closest to goal
  if(DRand() < m_goalBias) {
    
    size_t index = LRand() % m_biasedModes.size();
    modeID = m_biasedModes[index];
    historyID = m_modeHistories[modeID][0];

  }
  else {

    modeID = LRand() % m_modes.size();
    size_t index = LRand() % m_modeHistories[modeID].size();
    historyID = m_modeHistories[modeID][index];

  }

  std::cout << "Selecting mode " << modeID << " with action history [";
  for(auto id : m_actionHistories[historyID]) {
    std::cout << id << ", ";
  }
  std::cout << "]" << std::endl;

  return std::make_pair(modeID,historyID);
}

std::pair<size_t,SMART::Direction>
SMART::
SelectVertex(size_t _modeID, size_t _historyID, Mode _heuristic) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SelectVertex");

  // Sample random direction
  // Do not need to fix formations because we only care about active robots
  auto random = GetRandomDirection(_historyID);

  // Check if bias is valid for this history
  auto iter = m_historyVIDBias.find(_historyID);
  if(iter != m_historyVIDBias.end()) {
    auto aeid = m_actionExtendedGraph->GetVertex(m_historyVIDBias.at(_historyID));
    return std::make_pair(aeid.vid,random);
  }

  auto mode = m_modes[_modeID];

  // Find history vid closest to the randomly sampled vertex 
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  auto candidates = m_historyVIDs[_historyID];
 
  double minDistance = MAX_DBL;
  VID closest = MAX_INT;

  for(auto vid : candidates) {
    auto vertex = m_tensorProductRoadmap->GetVertex(vid);

    double distance = 0;
    for(auto pair : vertex.cfgs) {

      // Compute distance to random sample
      auto rm = pair.first;
      auto gcfg1 = rm->GetVertex(pair.second);
      auto gcfg2 = random[rm];

      for(auto robot : rm->GetGroup()->GetRobots()) {
        if(robot->GetMultiBody()->IsPassive())
          continue;

        distance += dm->Distance(gcfg1.GetRobotCfg(robot),
                                 gcfg2.GetRobotCfg(robot));
      }

    }

    // If closer than previous best, update closest neighbor
    if(distance < minDistance) {
      minDistance = distance;
      closest = vid;
    }
  }

  return std::make_pair(closest,random);
}

size_t
SMART::
Extend(size_t _qNear, Direction _direction, size_t _modeID, 
       size_t _historyID, Mode _heuristic) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Extend");

  size_t qNew = MAX_INT;
  m_historyVIDBias[_historyID] = qNew;

  // With m_heuristicProb probability, choose a heuristic direction
  if(DRand() <= m_heuristicProb) {
    _direction = GetHeuristicDirection(_modeID,_heuristic);
  }

  // Compute angle between vectors
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

  auto vertex = m_tensorProductRoadmap->GetVertex(_qNear);
  
  Vertex neighbor;
  neighbor.modeID = vertex.modeID;
  // Find individual gcfg that minimizes angle for each robot
  for(auto pair : vertex.cfgs) {
    auto grm = pair.first;
    auto vid = pair.second;
    auto start = grm->GetVertex(vid);
    
    // Check if this is a passive group
    auto group = grm->GetGroup();
    bool passive = true;
    for(auto robot : group->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive()) {
        passive = false;
        break;
      }
    }

    // Leave cfg stationary if it is
    if(passive) { 
      neighbor.cfgs.push_back(pair);
      continue;
    }

    // Check if direction is given for group
    // If not, given stationary cfg
    if(_direction.find(grm) == _direction.end()) {
      neighbor.cfgs.push_back(pair);
      continue;
    }

    // Initialize minimum neighbor as staying put
    auto d = _direction[grm];
    auto vec1 = d - start;

    GroupCfg empty(grm);
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
    auto vit = grm->find_vertex(vid);
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {

      // Grab neighbor
      auto target = eit->target();
      auto n = grm->GetVertex(target);
      auto vec2 = n - start;

      double angle = computeAngle(vec1,vec2);

      // Check if this is a new min angle
      if(angle >= minAngle)
        continue;

      // If yes, save
      minAngle = angle;
      newVID = target;
    }

    neighbor.cfgs.push_back(std::make_pair(grm,newVID));
  }

  // Check if there is a valid connection to the neighbor
  if(!ValidConnection(vertex,neighbor))
    return qNew;

  // Add vertex to tensor product roadmap
  qNew = m_tensorProductRoadmap->AddVertex(neighbor);

  // Add edge to qNear
  std::vector<std::pair<GroupRoadmapType*,std::pair<size_t,size_t>>> transitions;
  size_t cost = 0;

  for(auto pair1 : vertex.cfgs) {
    auto grm = pair1.first;

    for(auto pair2 : neighbor.cfgs) {
      if(pair2.first == grm) {
        
        auto vids = std::make_pair(pair1.second,pair2.second);
        transitions.emplace_back(grm,vids);

        if(vids.first != vids.second)
          cost = std::max(cost,grm->GetEdge(vids.first,vids.second).GetTimeSteps());

        break;
      }
    }

  }

  Edge edge;
  edge.transitions = transitions;
  edge.cost = double(cost);

  m_tensorProductRoadmap->AddEdge(_qNear,qNew,edge);

  // Add to action extended graph
  ActionExtendedState state;
  state.vid = _qNear;
  state.ahid = _historyID;

  auto aeSource = m_actionExtendedGraph->GetVID(state);

  state.vid = qNew;

  auto aeTarget = m_actionExtendedGraph->AddVertex(state);
  
  ActionExtendedEdge aeEdge;
  aeEdge.cost = double(cost);

  m_actionExtendedGraph->AddEdge(aeSource,aeTarget,aeEdge);

  m_historyVIDs[_historyID].insert(qNew);
  m_historyVIDBias[_historyID] = aeTarget;

  return qNew;
}

size_t
SMART::
Rewire(size_t _qNew, size_t _modeID, size_t _historyID) {

  // TODO::Get TPR vertex from AEG vertex

  // TODO:: Task 4

  // Basic rewire logic

  // Get all neighbors with cost less than current
  // TODO::Add map in header to track cost to each action extended vertex

  // In order of best cost from source, check if there is a valid connection in the tensor product roadmap

  return MAX_INT;
}

bool
SMART::
ValidConnection(const Vertex& _source, const Vertex& _target) {

  // TODO:: Task 3

  // Reconstruct local plans for each robot

  // Check if they are in collision

  return true;
}

bool 
SMART::
CheckForModeSwitch(size_t _qNew) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GetRandomDirection");

  auto sg = static_cast<OCMG*>(this->GetStateGraph(m_sgLabel).get());
  auto omg = sg->GetSingleObjectModeGraph();

  // Get mode
  auto aes = m_actionExtendedGraph->GetVertex(_qNew);
  auto vertex = m_tensorProductRoadmap->GetVertex(aes.vid);
  auto modeID = vertex.modeID;
  auto mode = m_modes[modeID];

  // For each object, check if it can transition to any of its neighbors
  std::vector<std::vector<std::pair<Robot*,size_t>>> modeSwitches;

  for(auto kv : mode) {
    auto object = kv.first;
    auto source = kv.second;

    // Get vertex iterator
    auto vit = omg->find_vertex(source);
    if(vit == omg->end())
      throw RunTimeException(WHERE) << "Failed to find single object mode in graph.";

    // Iterator through outgoing edges and check if vertex matches
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      auto target = eit->target();

      auto transition = sg->GetSingleObjectModeGraphEdgeTransitions(source,target,object);
      auto start = transition.first;

      if(start.empty())
        continue;

      bool match = true;
      for(auto kv : start) {

        match = false;
        for(auto pair : vertex.cfgs) {
          auto grm = pair.first;
          auto group = grm->GetGroup();
          
          // Check if this is the current group being checked
          if(group != kv.first)
            continue;

          // Check if the cfg info matches
          if(pair != kv.second)
            continue;

          match = true;
          break;
        }

        if(!match)
          break;
      }

      if(!match)
        continue;

      // Add transition to set of mode switches
      auto copy = modeSwitches;
      auto newSwitch = std::make_pair(object,target);
      modeSwitches.push_back({newSwitch});

      for(auto modeSwitch : copy) {
        modeSwitch.push_back(newSwitch);
        modeSwitches.push_back(modeSwitch);
      }

    }
  }

  // Build full set of mode switches available
  for(auto modeSwitch : modeSwitches) {
    Vertex target;

    // Build new mode
    Mode newMode = mode;
    for(auto pair : modeSwitch) {
      newMode[pair.first] = pair.second;
    }

    // Get mode id for new mode
    bool alreadyFound = false;
    for(size_t i = 0; i < m_modes.size(); i++) {
      if(m_modes[i] == newMode) {
        alreadyFound = true;
        target.modeID = i;
      }
    }

    if(!alreadyFound) {
      target.modeID = m_modes.size();
      m_modes.push_back(newMode);
    }

    // Build cfgs of new vertex
    for(auto pair : modeSwitch) {
      auto object = pair.first;
      auto targetID = pair.second;
      auto sourceID = mode[object];

      auto transition = sg->GetSingleObjectModeGraphEdgeTransitions(sourceID,targetID,object);
      auto goal = transition.second;

      for(auto kv : goal) {
        target.cfgs.push_back(kv.second);
      }
    }

    // Add vertex to tensor product roadmap
    auto vid = m_tensorProductRoadmap->AddVertex(target);
    
    // Connect to source vertex
    Edge edge;
    edge.cost = 0.;
    m_tensorProductRoadmap->AddEdge(aes.vid,vid,edge);

    // Add transition to history
    auto history = m_actionHistories[aes.ahid];
    history.push_back(target.modeID);
    m_actionHistories.push_back(history);

    // Add vertex to action extended graph 
    ActionExtendedState aeState;
    aeState.vid = vid;
    aeState.ahid = m_actionHistories.size() - 1;
    auto aeVID = m_actionExtendedGraph->AddVertex(aeState);

    // Connect to source vertex
    ActionExtendedEdge aeEdge;
    aeEdge.cost = 0.;

    m_actionExtendedGraph->AddEdge(_qNew,aeVID,aeEdge);

    // Log extra tracking info
    m_historyVIDs[aeState.ahid].insert(vid);
    m_historyVIDBias[aeState.ahid] = vid;
    m_modeHistories[target.modeID].push_back(aeState.ahid);

    if(CheckForGoal(aeVID))
      return true;

    // Get cost to go
    auto costToGo = ComputeMAPFHeuristic(target.modeID).costToGo;

    auto currentBest = ComputeMAPFHeuristic(m_biasedModes.front()).costToGo;
    if(costToGo < currentBest) {
      m_biasedModes = {target.modeID};
    }
    else if(currentBest == costToGo) {
      // Check that mode is not already in the set
      bool exists = false;
      for(auto id : m_biasedModes) {
        if(id = target.modeID) {
          exists = true;
          break;
        }
      }
    
      if(!exists)
        m_biasedModes.push_back(target.modeID);
    }
  }

  return false;
}

bool
SMART::
CheckForGoal(size_t _qNew) {

  // TODO:: Task 2
  
  // Check if the state satisfies the goal constraints in the decomposition

  return false;
}

std::map<SMART::GroupRoadmapType*,GroupCfg>
SMART::
GetRandomDirection(size_t _historyID) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GetRandomDirection");

  auto problem = this->GetMPProblem();
  auto env = problem->GetEnvironment();

  auto illustrative = m_tensorProductRoadmap->GetVertex(
        *(m_historyVIDs[_historyID].begin()));

  std::map<GroupRoadmapType*,GroupCfg> random;
  
  for(auto pair : illustrative.cfgs) {
    auto rm = pair.first;

    rm->SetAllFormationsInactive();
    for(auto f : rm->GetVertex(pair.second).GetFormations()) {
      rm->SetFormationActive(f);
    }

    GroupCfg gcfg(rm);
    gcfg.GetRandomGroupCfg(env->GetBoundary());
    random[rm] = gcfg;
  }

  return random;
}

std::map<SMART::GroupRoadmapType*,GroupCfg>
SMART::
GetHeuristicDirection(size_t _modeID, Mode _heuristic) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GetHeuristicDirection");

  //auto problem = this->GetMPProblem();
  auto sg = static_cast<OCMG*>(this->GetStateGraph(m_sgLabel).get());
  //auto omg = sg->GetSingleObjectModeGraph();
  auto mode = m_modes[_modeID];

  std::map<GroupRoadmapType*,GroupCfg> direction;

  for(auto kv1 : mode) {
    auto object = kv1.first;
    auto source = kv1.second;
    auto target = _heuristic[object];
    auto transition = sg->GetSingleObjectModeGraphEdgeTransitions(source,target,object);

    auto goal = transition.first;
    for(auto kv2 : goal) {
      auto pair = kv2.second;
      auto grm = pair.first;
      auto gcfg = grm->GetVertex(pair.second);
      direction[grm] = gcfg;
    }
  }


  return direction;
  /*// Collect source groups
  std::vector<GroupRoadmapType*> sourceGrms;
  
  for(auto kv : mode) {

    // Collect relevant robots
    auto object = kv.first;
    auto info = omg->GetVertex(kv.second);

    std::vector<Robot*> robots = {object};
    if(info.robot)
      robots.push_back(info.robot);

    // Get group from problem
    auto group = problem->AddRobotGroup(robots,"");

    // Add roadmap to set
    auto rm = sg->GetGroupRoadmap(group);
    sourceGrms.push_back(rm);
  }

  // Collect target groups
  std::vector<GroupRoadmapType*> targetGrms;
  
  for(auto kv : _heuristic) {

    // Collect relevant robots
    auto object = kv.first;
    auto info = omg->GetVertex(kv.second);

    std::vector<Robot*> robots = {object};
    if(info.robot)
      robots.push_back(info.robot);

    // Get group from problem
    auto group = problem->AddRobotGroup(robots,"");

    // Add roadmap to set
    auto rm = sg->GetGroupRoadmap(group);
    sourceGrms.push_back(rm);
  }*/
}

/*--------------------------- Heuristic Functions ----------------------------*/

SMART::HeuristicValues
SMART::
ComputeMAPFHeuristic(size_t _modeID) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ComputeMAPFHeuristic");
  
  if(m_cachedHeuristics.find(_modeID) != m_cachedHeuristics.end()) {
    return m_cachedHeuristics.at(_modeID);
  }

  auto sg = static_cast<OCMG*>(this->GetStateGraph(m_sgLabel).get());
  auto omg = sg->GetSingleObjectModeGraph();

  // Set each object's start
  m_MAPFStarts = m_modes[_modeID];

  // Set each object's goal
  if(!m_cachedMAPFGoals) {
    m_cachedMAPFGoals = true;

    // Go through each task in the decomposition
    auto coordinator = plan->GetCoordinator();
    auto decomp = this->GetMPProblem()->GetDecompositions(coordinator->GetRobot())[0].get();
    for(auto st : decomp->GetGroupMotionTasks()) {
      auto gt = st->GetGroupMotionTask();
      for(auto iter = gt->begin(); iter != gt->end(); iter++) {

        // Isolate the object 
        auto object = iter->GetRobot();
        auto rm = sg->GetGroupRoadmap(gt->GetRobotGroup());

        auto constraint = iter->GetGoalConstraints()[0].get();

        // Find the VIDs that satisfy the goal constraints
        for(auto vit = rm->begin(); vit != rm->end(); vit++) {
          auto vid = vit->descriptor();
          auto gcfg = vit->property();
          size_t index = 0;
          auto cfg = gcfg.GetRobotCfg(index);
          if(!constraint->Satisfied(cfg)) {
            continue;
          }

          for(auto& kv1 : sg->GetTerrainVIDs()) {
            auto vids = kv1.second[rm];
            if(!vids.count(vid))
              continue;

            OCMG::ModeInfo info(nullptr,nullptr,kv1.first);
            auto omgVID = omg->GetVID(info);
            if(omgVID >= MAX_INT)
              throw RunTimeException(WHERE) << "Failed to find matching goal terrain.";

            // Check if object already has a specified goal and make sure it does not conflict
            if(m_MAPFGoals.find(object) != m_MAPFGoals.end()) {
              if(m_MAPFGoals[object] != omgVID)
                throw RunTimeException(WHERE) << "Too many goal terrains.";
            }

            m_MAPFGoals[object] = omgVID;
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

  Mode heuristic;
  for(auto kv : solution.solutionMap) {
    auto object = kv.first;
    auto path = *kv.second;
    heuristic[object] = path.size() > 1 ? path[1] : path[0];
  }

  HeuristicValues values;
  values.nextMode = heuristic;
  values.costToGo = solution.cost;

  m_cachedHeuristics[_modeID] = values;

  return values;
}

bool
SMART::
LowLevelPlanner(CBSNodeType& _node, Robot* _robot) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::LowLevelPlanner");

  auto sg = static_cast<OCMG*>(
      this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetSingleObjectModeGraph();

  auto h = std::shared_ptr<HeuristicSearch>(new HeuristicSearch(_robot));

  auto constraints = _node.constraintMap[_robot];

  // Get start and goal of robot in terms of vid in g
  auto start = m_MAPFStarts[_robot];
  size_t goal = m_MAPFGoals[_robot];

  SSSPPathWeightFunction<OCMG::SingleObjectModeGraph> cost2goWeight(
    [](typename OCMG::SingleObjectModeGraph::adj_edge_iterator& _ei,
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

std::vector<std::pair<Robot*,SMART::CBSConstraint>>
SMART::
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
SMART::
CostFunction(CBSNodeType& _node) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CostFunction");

  auto sg = static_cast<OCMG*>(
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

std::vector<SMART::CBSNodeType> 
SMART::
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
/*----------------------------------------------------------------------------*/

istream&
operator>>(std::istream& _is, const SMART::Vertex) {
  return _is;
}

ostream&
operator<<(std::ostream& _os, const SMART::Vertex) {
  return _os;
}

istream&
operator>>(std::istream& _is, const SMART::Edge) {
  return _is;
}

ostream&
operator<<(std::ostream& _os, const SMART::Edge) {
  return _os;
}

istream&
operator>>(std::istream& _is, const SMART::ActionExtendedState) {
  return _is;
}

ostream&
operator<<(std::ostream& _os, const SMART::ActionExtendedState) {
  return _os;
}

istream&
operator>>(std::istream& _is, const SMART::ActionExtendedEdge) {
  return _is;
}

ostream&
operator<<(std::ostream& _os, const SMART::ActionExtendedEdge) {
  return _os;
}
