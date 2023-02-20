#include "SMART.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
#include "MPProblem/MPTask.h"

#include "TMPLibrary/StateGraphs/OCMG.h"
#include "TMPLibrary/Solution/Plan.h"

#include "Traits/CfgTraits.h"

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
            "Distance metric to compute distance between group cfgs.");

  m_cdLabel = _node.Read("cdLabel",true,"",
            "Collision detection method to check for interrobot collision.");
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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");

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
    if(qNew >= MAX_UINT)
      continue;

    auto qBest = Rewire(qNew,qNear,modeID,historyID);

    if(qBest >= MAX_UINT)
      qBest = qNear;

    bool foundGoal = CheckForModeSwitch(qNew);

    if(foundGoal or CheckForGoal(qNew)) {
      std::cout << "FOUND GOAL!!!" << std::endl;
      // For now, quit at first solution

      stats->SetStat("Success",1);
      stats->SetStat("Steps",i);
      stats->SetStat("Conflicts",m_conflictCount);
      stats->SetStat(this->GetNameAndLabel()+"::BestCost",m_goalDistance);

      return true;
    }
  }

  stats->SetStat("Success",0);
  stats->SetStat("Steps",m_maxIterations);
  stats->SetStat("Conflicts",m_conflictCount);

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
    GroupCfgType gcfg(rm);

    for(auto robot : group->GetRobots()) {
      auto cfg = problem->GetInitialCfg(robot);
      gcfg.SetRobotCfg(robot,std::move(cfg));
    }

    auto vid = rm->GetVID(gcfg);
    if(vid >= MAX_UINT) {
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
    if(omgVID >= MAX_UINT) {
      throw RunTimeException(WHERE) << "Failed to find object mode vertex for "
                                    << passive->GetLabel();
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

  rootVID = m_actionExtendedGraph->AddVertex(aes);

  m_historyVIDs[0].insert(rootVID);
}

std::pair<size_t,size_t>
SMART::
SelectMode() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SelectMode");

  size_t modeID = MAX_UINT;
  size_t historyID = MAX_UINT;

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

  if(m_debug) {
    std::cout << "Selecting mode " << modeID << " with action history [";
    for(auto id : m_actionHistories[historyID]) {
      std::cout << id << ", ";
    }
    std::cout << "]" << std::endl;
  }

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
  if(iter != m_historyVIDBias.end() and iter->second != MAX_UINT) {
    //auto aeid = m_actionExtendedGraph->GetVertex(m_historyVIDBias.at(_historyID));
    return std::make_pair(m_historyVIDBias.at(_historyID),random);
  }

  auto mode = m_modes[_modeID];

  // Find history vid closest to the randomly sampled vertex 
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  auto candidates = m_historyVIDs[_historyID];
 
  double minDistance = MAX_DBL;
  VID closest = MAX_UINT;

  for(auto vid : candidates) {
    auto aeVertex = m_actionExtendedGraph->GetVertex(vid);
    auto vertex = m_tensorProductRoadmap->GetVertex(aeVertex.vid);

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

  size_t qNew = MAX_UINT;
  m_historyVIDBias[_historyID] = qNew;

  auto aeState = m_actionExtendedGraph->GetVertex(_qNear);
  auto qNear = aeState.vid;

  // With m_heuristicProb probability, choose a heuristic direction
  if(DRand() <= m_heuristicProb) {
    _direction = GetHeuristicDirection(qNear,_modeID,_heuristic);
  }

  // Compute angle between vectors
  auto computeAngle = [stats,this](GroupCfgType& _cfg1, GroupCfgType& _cfg2) {
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

  auto vertex = m_tensorProductRoadmap->GetVertex(qNear);
  
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

    // If direction is to go to current vertex, stay put
    if(d == start) {
      neighbor.cfgs.push_back(pair);
      continue;
    }

    GroupCfgType empty(grm);
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

  m_tensorProductRoadmap->AddEdge(qNear,qNew,edge);

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

  m_historyVIDs[_historyID].insert(aeTarget);
  m_historyVIDBias[_historyID] = aeTarget;

  m_distanceMap[aeTarget] = m_distanceMap[aeSource] + aeEdge.cost;

  return aeTarget;
}

size_t
SMART::
Rewire(size_t _qNew, size_t _qNear, size_t _modeID, size_t _historyID) {

  // Get TPR vertex from AEG vertex
  auto aeState = m_actionExtendedGraph->GetVertex(_qNew);
  auto vid1 = aeState.vid;
  const double originalDistance = m_distanceMap[_qNew];

  // Basic rewire logic

  auto candidates = m_historyVIDs[_historyID];
  auto vertex1 = m_tensorProductRoadmap->GetVertex(vid1);

  // Get all neighbors with cost less than current

  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  std::vector<std::pair<double,size_t>> neighbors;
  std::vector<std::pair<double,size_t>> backNeighbors;

  for(auto candidate : candidates) {
    auto aeState = m_actionExtendedGraph->GetVertex(candidate);
    auto vid2 = aeState.vid;
    auto vertex2 = m_tensorProductRoadmap->GetVertex(vid2);

    // Check that there is an edge in the implicit tensor product roadmap
    bool isEdge = true;
    bool isBackEdge = true;
    double distance = 0;
    for(auto pair1 : vertex1.cfgs) {
      auto grm = pair1.first;
      auto target = pair1.second;

      for(auto pair2 : vertex2.cfgs) {
        if(pair2.first != grm)
          continue;

        auto source = pair2.second;

        if(!grm->IsEdge(source,target)) {
          isEdge = false;
        }

        if(!grm->IsEdge(target,source)) {
          isBackEdge = false;
        }

        if(!isEdge and !isBackEdge)
          break;

        distance += dm->Distance(grm->GetVertex(source),grm->GetVertex(target));
      }

      if(!isEdge and !isBackEdge)
        break;
    }

    if(isBackEdge) {
      backNeighbors.emplace_back(distance,candidate);
    }

    distance += m_distanceMap[candidate];
    if(isEdge and distance < originalDistance) {
      neighbors.emplace_back(distance,candidate);
    }
  }

  // Sort the neighbors by rewire distance
  std::sort(neighbors.begin(), neighbors.end());

  // In order of best cost from source, check if there is a valid connection in the tensor product roadmap
  size_t qBest = MAX_UINT;
  
  for(auto pair : neighbors) {
    auto cand = pair.second;
    auto vid2 = m_actionExtendedGraph->GetVertex(cand).vid;
    
    // Check if it is connected
    auto vertex2 = m_tensorProductRoadmap->GetVertex(vid2);
    if(!ValidConnection(vertex2,vertex1))
      continue;

    qBest = cand;
    m_distanceMap[_qNew] = pair.first;

    // Check if this was the original edge
    if(cand == _qNear)
      break;

    // Otherwise, delete the old edge, and add this one
    if(m_actionExtendedGraph->IsEdge(_qNear,_qNew))
      m_actionExtendedGraph->DeleteEdge(_qNear,_qNew);

    ActionExtendedEdge edge;
    edge.cost = m_distanceMap[_qNew] - m_distanceMap[cand];
    
    m_actionExtendedGraph->AddEdge(cand,_qNew,edge);
    
    break;
  }

  // Attempt to rewire the nighbors through qNew
  for(auto pair : backNeighbors) {
    auto cand = pair.second;
    auto vid2 = m_actionExtendedGraph->GetVertex(cand).vid;
    
    // Check if rewire is useful
    if(pair.second + m_distanceMap[_qNew] >= m_distanceMap[cand])
      continue;

    // Check if it is connected
    auto vertex2 = m_tensorProductRoadmap->GetVertex(vid2);
    if(!ValidConnection(vertex1,vertex2))
      continue;

    // Delete parent of cand
    auto parents = m_actionExtendedGraph->GetPredecessors(cand);
    for(auto parent : parents) {
      if(m_actionExtendedGraph->IsEdge(parent,cand))
        m_actionExtendedGraph->DeleteEdge(parent,cand);
    }

    // Connect qNew to cand
    ActionExtendedEdge edge;
    edge.cost = m_distanceMap[_qNew] + pair.first;

    m_actionExtendedGraph->AddEdge(_qNew,cand,edge);
  }

  return qBest;
}

bool
SMART::
ValidConnection(const Vertex& _source, const Vertex& _target) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidConnection");

  if(_source == _target)
    return false;

  if(_source.cfgs.size() != _target.cfgs.size())
      throw RunTimeException(WHERE) << "Mismatched group roadmaps.";

  // Collect edge vids for each robot group
  std::map<GroupRoadmapType*,std::pair<size_t,size_t>> edges;

  for(auto pair : _source.cfgs) {
    auto grm = pair.first;
    auto vid = pair.second;

    edges[grm] = std::make_pair(vid,MAX_UINT);
  }

  for(auto pair : _target.cfgs) {
    auto grm = pair.first;
    auto vid = pair.second;

    if(edges.find(grm) == edges.end())
      throw RunTimeException(WHERE) << "Mismatched group roadmaps.";

    auto& edge = edges[grm];
    edge.second = vid;
  }

  // Reconstruct local plans for each robot group
  auto sg = static_cast<OCMG*>(this->GetStateGraph(m_sgLabel).get());
  auto lib = this->GetMPLibrary();
  lib->SetMPSolution(sg->GetMPSolution());
  auto problem = this->GetMPProblem();
  auto env = problem->GetEnvironment();

  std::map<RobotGroup*,std::vector<GroupCfgType>> localPlans;
  size_t maxTimestep = 0;

  for(auto kv : edges) {
    auto grm = kv.first;
    auto source = kv.second.first;
    auto target = kv.second.second;
    auto group = grm->GetGroup();

    if(source == target) {
      localPlans[group] = {grm->GetVertex(source)};
      continue;
    }

    localPlans[group] = lib->ReconstructEdge(grm,source,target,
                       env->GetPositionRes(),env->GetOrientationRes());

    maxTimestep = std::max(maxTimestep,localPlans[group].size());
  }

  // Check if they are in collision
  auto cd = dynamic_cast<CollisionDetectionValidity<MPTraits<Cfg>>*>(lib->GetValidityChecker(m_cdLabel));

  for(size_t t = 0; t < maxTimestep; t++) {
    for(auto iter1 = localPlans.begin(); iter1 != localPlans.end(); iter1++) {
      auto group1 = iter1->first;
      const auto& path1 = iter1->second;
      const size_t t1 = path1.size() > t ? t : path1.size() - 1;
      const auto& gcfg1 = path1[t1];
    
      gcfg1.ConfigureRobot();

      auto iter2 = iter1;
      iter2++;
      for(; iter2 != localPlans.end(); iter2++) {
        auto group2 = iter2->first;
        const auto& path2 = iter2->second;
        const size_t t2 = path2.size() > t ? t : path2.size() - 1;
        const auto& gcfg2 = path2[t2];

        gcfg2.ConfigureRobot();

        for(auto r1 : group1->GetRobots()) {
          for(auto r2 : group2->GetRobots()) {
            CDInfo cdInfo;
            if(cd->IsMultiBodyCollision(cdInfo,
                r1->GetMultiBody(),r2->GetMultiBody(),this->GetNameAndLabel())) {

              return false;
            }
          }
        }
      }
    }
  }

  return true;
}

bool 
SMART::
CheckForModeSwitch(size_t _qNew) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CheckForModeSwitch");

  auto sg = static_cast<OCMG*>(this->GetStateGraph(m_sgLabel).get());
  auto omg = sg->GetSingleObjectModeGraph();

  // Get mode
  auto aes = m_actionExtendedGraph->GetVertex(_qNew);
  auto vertex = m_tensorProductRoadmap->GetVertex(aes.vid);
  auto modeID = vertex.modeID;
  auto mode = m_modes[modeID];

  // For each object, check if it can transition to any of its neighbors
  std::vector<std::vector<std::pair<Robot*,size_t>>> modeSwitches;

  std::map<Robot*,std::pair<OCMG::State,OCMG::State>> matchedTransitions;

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

      auto transitions = sg->GetSingleObjectModeGraphEdgeTransitions(source,target,object);
      bool match = false;
      for(auto transition : transitions) {
        auto start = transition.first;

        if(start.empty())
          continue;

        match = true;
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

        matchedTransitions[object] = transition;

        if(match)
          break;
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
    std::set<Robot*> used;

    // Add transition cfgs
    for(auto pair : modeSwitch) {
      auto object = pair.first;
      //auto targetID = pair.second;
      //auto sourceID = mode[object];

      //auto transition = sg->GetSingleObjectModeGraphEdgeTransitions(sourceID,targetID,object);
      auto transition = matchedTransitions[object];
      auto goal = transition.second;

      for(auto kv : goal) {
        target.cfgs.push_back(kv.second);

        for(auto robot : kv.first->GetRobots()) {
          used.insert(robot);
        }
      }
    }

    // Copy in cfgs that did not transition
    for(auto pair : vertex.cfgs) {
      auto group = pair.first->GetGroup();
      bool accountedFor = false;
      for(auto robot : group->GetRobots()) {
        if(used.count(robot)) {
          accountedFor = true;
          continue;
        }
        else if(accountedFor) {
          throw RunTimeException(WHERE) << "Partial accounting of robot group in transition.";
        }
      }

      if(!accountedFor)
        target.cfgs.push_back(pair);
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
    m_historyVIDs[aeState.ahid].insert(aeVID);
    m_historyVIDBias[aeState.ahid] = aeVID;
    m_modeHistories[target.modeID].push_back(aeState.ahid);

    m_distanceMap[aeVID] = m_distanceMap[_qNew] + aeEdge.cost;

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
        if(id == target.modeID) {
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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CheckForGoal");

  auto coordinator = plan->GetCoordinator();
  auto problem = this->GetMPProblem();
  auto decomp = problem->GetDecompositions(coordinator->GetRobot())[0].get();

  // Grab current cfgs
  auto aeState = m_actionExtendedGraph->GetVertex(_qNew);
  auto vertex = m_tensorProductRoadmap->GetVertex(aeState.vid);

  std::map<Robot*,Cfg> currentCfgs;
  for(auto pair : vertex.cfgs) {
    auto grm = pair.first;
    auto vid = pair.second;
    auto gcfg = grm->GetVertex(vid);

    for(auto robot : grm->GetGroup()->GetRobots()) {
      auto cfg = gcfg.GetRobotCfg(robot);
      currentCfgs[robot] = cfg;
    }
  }

  // Check that each task is satisfied
  std::set<SemanticTask*> satisfied;
  for(auto st : decomp->GetGroupMotionTasks()) {

    auto parent = st->GetParent();
    std::vector<SemanticTask*> tasks; 

    auto relation = parent->GetSubtaskRelation();

    if(relation == SemanticTask::SubtaskRelation::XOR) {
      if(satisfied.count(parent))
        continue;

      tasks = parent->GetSubtasks();
    }
    else {
      tasks = {st};
    }


    bool isSatisfied = true;
    for(auto task : tasks) {
      auto gt = task->GetGroupMotionTask();

      for(auto iter = gt->begin(); iter != gt->end(); iter++) {
        auto constraint = iter->GetGoalConstraints()[0].get();

        auto robot = constraint->GetRobot();
        auto cfg = currentCfgs[robot];

        if(constraint->Satisfied(cfg))
          continue;

        isSatisfied = false;
        break;
      }

      if(isSatisfied)
        break;
    }

    if(isSatisfied) {
      satisfied.insert(parent);
      continue;
    }

    return false;
  }
 
  m_goalDistance = m_distanceMap[_qNew];

  return true;
}

std::map<SMART::GroupRoadmapType*,SMART::GroupCfgType>
SMART::
GetRandomDirection(size_t _historyID) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GetRandomDirection");

  auto problem = this->GetMPProblem();
  auto env = problem->GetEnvironment();

  auto aeState = m_actionExtendedGraph->GetVertex(*(m_historyVIDs[_historyID].begin()));
  auto illustrative = m_tensorProductRoadmap->GetVertex(aeState.vid);

  std::map<GroupRoadmapType*,GroupCfgType> random;
  
  for(auto pair : illustrative.cfgs) {
    auto rm = pair.first;

    rm->SetAllFormationsInactive();
    for(auto f : rm->GetVertex(pair.second).GetFormations()) {
      rm->SetFormationActive(f);
    }

    GroupCfgType gcfg(rm);
    gcfg.GetRandomGroupCfg(env->GetBoundary());
    random[rm] = gcfg;
  }

  return random;
}

std::map<SMART::GroupRoadmapType*,SMART::GroupCfgType>
SMART::
GetHeuristicDirection(size_t _vid, size_t _modeID, Mode _heuristic) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GetHeuristicDirection");

  //auto problem = this->GetMPProblem();
  auto sg = static_cast<OCMG*>(this->GetStateGraph(m_sgLabel).get());
  //auto omg = sg->GetSingleObjectModeGraph();
  auto mode = m_modes[_modeID];

  auto vertex = m_tensorProductRoadmap->GetVertex(_vid);

  std::map<GroupRoadmapType*,GroupCfgType> direction;

  for(auto kv1 : mode) {
    auto object = kv1.first;
    auto source = kv1.second;
    auto target = _heuristic[object];
    auto transitions = sg->GetSingleObjectModeGraphEdgeTransitions(source,target,object);

    for(auto transition : transitions) {
      auto goal = transition.first;

      // Find group containing object
      RobotGroup* group = nullptr;
      for(auto kv : goal) {
        for(auto robot : kv.first->GetRobots()) {
          if(robot == object) {
            group = kv.first;
            break;
          }
        }
      }

      // Check if formations match
      bool formationMatch = true;
      for(auto pair : vertex.cfgs) {
        if(pair.first->GetGroup() != group)
          continue;

        auto gcfg1 = goal[group].first->GetVertex(goal[group].second);
        auto gcfg2 = pair.first->GetVertex(pair.second);

        auto form1 = gcfg1.GetFormations();
        auto form2 = gcfg2.GetFormations();

        if(form1.size() != form2.size()) {
          formationMatch = false;
          break;
        }

        for(auto f1 : form1) {
          formationMatch = false;
          for(auto f2 : form2) {

            if(f1 == f2 or *f1 == *f2) {
              formationMatch = true;
              break;
            }
          }

          if(!formationMatch)
            break;
        }

        if(formationMatch == false)
          break;

        for(auto f2 : form2) {
          formationMatch = false;
          for(auto f1 : form1) {

            if(f1 == f2 or *f1 == *f2) {
              formationMatch = true;
              break;
            }
          }

          if(!formationMatch)
            break;
        }

        break;
      }

      if(!formationMatch)
        continue;

      for(auto kv2 : goal) {
        auto pair = kv2.second;
        auto grm = pair.first;
        auto gcfg = grm->GetVertex(pair.second);
        direction[grm] = gcfg;
      }

      break;
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
            if(omgVID >= MAX_UINT)
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

  // Check that start does not violate a constraint and get min end time
  size_t minEndTimestep = 0;
  for(auto constraint : constraints) {

    minEndTimestep = std::max(minEndTimestep,constraint.second.second);

    if(start != constraint.first)
      continue;

    if(constraint.second.first <= 0)
      return false;
  }

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
    [goal,minEndTimestep](typename HeuristicSearch::vertex_iterator& _vi,
           const SSSPOutput<HeuristicSearch>& _sssp) {
      
      auto vertex = _vi->property();

      if(goal == vertex.first and minEndTimestep <= vertex.second)
        return SSSPTermination::EndSearch;

      return SSSPTermination::Continue;
    }
  );

  SSSPPathWeightFunction<HeuristicSearch> weight(
    [constraints,h](typename HeuristicSearch::adj_edge_iterator& _ei,
       const double _sourceDistance,
       const double _targetDistance) {
     
      //auto source = h->GetVertex(_ei->source()).first;
      auto target = h->GetVertex(_ei->target()).first;

      //auto timestep = h->GetVertex(_ei->source()).second;
      auto timestep = h->GetVertex(_ei->source()).second + 1;

      /*auto edgeConstraint = std::make_pair(std::make_pair(source,target),timestep);
      auto vertexConstraint = std::make_pair(std::make_pair(target,MAX_UINT),timestep+1);

      if(constraints.count(edgeConstraint) or constraints.count(vertexConstraint))
        return std::numeric_limits<double>::infinity();
      */

      for(auto constraint : constraints) {
        if(target != constraint.first)
          continue;
    
        auto range = constraint.second;
        if(timestep >= range.first and timestep <= range.second)
          return std::numeric_limits<double>::infinity();
      }

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
  if(h->GetVertex(last).first != goal or h->GetVertex(last).second < minEndTimestep) {
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
      //auto t1 = std::min(i+1,path1.size()-1);
      auto source1 = path1[s1];
      //auto target1 = path1[t1];

      //auto iter2 = iter1;
      //iter2++;
      for(auto iter2 = _node.solutionMap.begin(); iter2 != _node.solutionMap.end(); iter2++) {
        auto object2 = iter2->first;

        if(object1 == object2)
          continue;

        auto path2 = *(iter2->second);
        auto s2 = std::min(i,path2.size()-1);
        auto t2 = std::min(i+1,path2.size()-1);
        auto source2 = path2[s2];
        auto target2 = path2[t2];

        auto p2 = s2 == t2 ? s2 : s2 == 0 ? 0 : s2 - 1;
        auto parent2 = path2[p2];

        size_t conflictTimestep = MAX_UINT;

        if(source1 == parent2) {
          conflictTimestep = p2;
        }
        else if(source1 == source2) {
          conflictTimestep = s2;
        }
        else if(source1 == target2) {
          conflictTimestep = t2;
        }
        else {
          continue;
        }

        std::vector<std::pair<Robot*,CBSConstraint>> constraints;

        if(s1 != path1.size() - 1) {
          auto constraint1 = std::make_pair(source1,std::make_pair(
                              conflictTimestep > 0 ? conflictTimestep-1 : 0,
                              conflictTimestep + 1));
          constraints.emplace_back(object1,constraint1);
        }

        if(conflictTimestep != path2.size() - 1) {
          auto constraint2 = std::make_pair(source1,std::make_pair(
                              s1 > 0 ? s1-1 : 0,
                              s1 + 1));
          constraints.emplace_back(object2,constraint2);
        }

        return constraints;

        /*if(source1 == source2) {
          //if(m_debug) {
          //  std::cout << "Found vertex conflict at timestep " << i
          //            << " between " << object1->GetLabel()
          //            << " and " << object2->GetLabel() 
          //            << std::endl;
          //}

          auto constraint = std::make_pair(std::make_pair(source1,MAX_UINT),i);
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
        }*/
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
