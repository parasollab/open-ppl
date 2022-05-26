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
    
    auto pair = SelectMode();
    auto modeID = pair.first;
    auto historyID = pair.second;

    auto heuristics = ComputeMAPFHeuristic(modeID);

    auto qNear = Select(modeID,historyID,heuristics.nextMode);

    auto qNew = Extend(qNear,modeID,historyID,heuristics.nextMode);
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

size_t
SMART::
Select(size_t _modeID, size_t _historyID, Mode _heuristic) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SelectVertex");

  // Check if bias is valid for this history
  auto iter = m_historyVIDBias.find(_historyID);
  if(iter != m_historyVIDBias.end())
    return m_historyVIDBias.at(_historyID);

  auto mode = m_modes[_modeID];

  // Sample random direction
  // Do not need to fix formations because we only care about active robots
  auto random = GetRandomDirection(_historyID);

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

  return closest;
}

size_t
SMART::
Extend(size_t _qNear, size_t _modeID, size_t _historyID, Mode _heuristic) {

  size_t qNew = MAX_INT;


  m_historyVIDBias[_historyID] = qNew;
  return qNew;
}

size_t
SMART::
Rewire(size_t _qNew, size_t _modeID, size_t _historyID) {
  return MAX_INT;
}

bool
SMART::
ValidConnection(const Vertex& _source, const Vertex& _target) {
  return false;
}

bool 
SMART::
CheckForModeSwitch(size_t _qNew) {
  return false;
}

bool
SMART::
CheckForGoal(size_t _qNew) {
  return false;
}

std::map<SMART::GroupRoadmapType*,GroupCfg>
SMART::
GetRandomDirection(size_t _historyID) {
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
