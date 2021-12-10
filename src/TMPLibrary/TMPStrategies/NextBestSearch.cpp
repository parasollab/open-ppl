#include "NextBestSearch.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/StateGraphs/ModeGraph.h"
#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

#include <algorithm>

/*----------------------- Construction -----------------------*/

NextBestSearch::
NextBestSearch() {
  this->SetName("NextBestSearch");
}

NextBestSearch::
NextBestSearch(XMLNode& _node) : TMPStrategyMethod(_node) {
  this->SetName("NextBestSearch");

  m_queryLabel = _node.Read("queryLabel",true,"",
        "Map Evaluator to use to query individual solutions.");

  m_queryStrategy = _node.Read("queryStrategy",true,"",
        "MPStrategy to use to query individual mode solution.");

  m_vcLabel = _node.Read("vcLabel",true,"",
        "Validity checker to use for multirobot collision checking.");
}

NextBestSearch::
~NextBestSearch() {}

/*------------------------ Interface -------------------------*/

/*--------------------- Helper Functions ---------------------*/

void
NextBestSearch::
PlanTasks() {

  auto plan = this->GetPlan();
  auto originalDecomp = plan->GetDecomposition();
  auto te = this->GetTaskEvaluator(m_teLabel);
  te->Initialize();

  // Initialize bounds
  Node bestNode;
  bestNode.cost = MAX_DBL;
  double lowerBound = 0;

  // Store set of solutions
  std::vector<Decomposition*> taskSolutions;

  while(bestNode.cost > lowerBound) {
    lowerBound = FindTaskPlan(originalDecomp);
    taskSolutions.push_back(plan->GetDecomposition());

    if(bestNode.cost > lowerBound) {
      ComputeMotions(bestNode);
    }

    // TODO::Store solution in solution set
  }

  // TODO::Save plan in proper format
}
    
double
NextBestSearch::
FindTaskPlan(Decomposition* _decomp) {
  auto plan = this->GetPlan();
  plan->SetDecomposition(_decomp);

  auto te = this->GetTaskEvaluator(m_teLabel);
  if(te->operator()())
    return plan->GetCost();
  else 
    return MAX_DBL;
}

void
NextBestSearch::
ComputeMotions(Node& _bestNode) {

  // Configure CBS Functions
  CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType> lowLevel(
    [this](Node& _node, SemanticTask* _task) {
      return LowLevelPlanner(_node,_task);
    }
  );

  CBSValidationFunction<SemanticTask,Constraint,GroupPathType> validation(
    [this](Node& _node) {
      return this->ValidationFunction(_node);
    }
  );

  CBSCostFunction<SemanticTask,Constraint,GroupPathType> cost(
    [this](Node& _node) {
      return this->CostFunction(_node);
    }
  );

  CBSSplitNodeFunction<SemanticTask,Constraint,GroupPathType> splitNode(
    [this](Node& _node, std::vector<std::pair<SemanticTask*,Constraint>> _constraints,
           CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType>& _lowLevel,
           CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost) {
      return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
    }
  );

  CBSInitialFunction<SemanticTask,Constraint,GroupPathType> initial(
    [this](std::vector<Node>& _root, std::vector<SemanticTask*> _tasks,
           CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType>& _lowLevel,
           CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost) {
      return this->InitialSolutionFunction(_root,_tasks,_lowLevel,_cost);
    } 
  );

  // Collect tasks
  auto decomp = this->GetPlan()->GetDecomposition();
  auto tasks = decomp->GetGroupMotionTasks();

  // Call CBS
  Node solution = CBS(tasks,validation,splitNode,lowLevel,cost,initial);

  // Check if new cost is better than input best cost
  if(solution.cost < _bestNode.cost)
    _bestNode = solution;
}

/*----------------------- CBS Functors -----------------------*/

bool 
NextBestSearch::
LowLevelPlanner(Node& _node, SemanticTask* _task) {

  std::unordered_map<SemanticTask*,double> startTimes;
  std::unordered_map<SemanticTask*,double> endTimes;
  std::set<SemanticTask*> solved;
 
  // Initialize maps
  for(auto kv : _node.solutionMap) {
    auto task = kv.first;
    startTimes[task] = 0;

    if(task->GetDependencies().empty() and task != _task) {
      if(kv.second) {
        endTimes[task] = kv.second->Length();
        solved.insert(task);
      }
    }
  }

  // Add all unaffected task solutions to solved set
  auto size = solved.size();
  do {
    size = solved.size();
    // Iterate through task set
    for(auto kv : _node.solutionMap) {
      auto task = kv.first;
      if(solved.count(task))
        continue;

      double startTime = 0;

      // Check if all dependencies have been solved
      bool missingDep = false;
      for(auto dep : task->GetDependencies()) {
        for(auto t : dep.second) {
          if(!solved.count(t)) {
            missingDep = true;
            break;
          }
          startTime = std::max(startTime,endTimes[t]);
        }
        if(missingDep)
          break;
      }

      if(missingDep or !kv.second)
        continue;

      if(task != _task)
        solved.insert(task);

      startTimes[task] = startTime;
      endTimes[task] = startTime + kv.second->Length();
    }
  } while(solved.size() != size);
 
  // Init queue of tasks to solve
  std::priority_queue<std::pair<double,SemanticTask*>,
                      std::vector<std::pair<double,SemanticTask*>>,
                      std::greater<std::pair<double,SemanticTask*>>> pq;

  pq.push(std::make_pair(startTimes[_task],_task));

  std::set<SemanticTask*> unsolved;
  for(auto kv : _node.solutionMap) {
    if(solved.count(kv.first))
      continue;
    unsolved.insert(kv.first);
  }

  // Plan unsolved tasks
  while(!pq.empty()) {
    auto current = pq.top();
    pq.pop();
    auto task = current.second;
    auto startTime = current.first;

    // Compute new path
    auto path = QueryPath(task,startTime,_node);

    // Check if path was found
    if(!path)
      return false;

    // Save path to solution
    _node.solutionMap[task] = path;
    solved.insert(task);
    endTimes[task] = startTime + path->Length();

    // Check if new tasks are available to plan
    std::vector<SemanticTask*> toRemove;
    for(auto t : unsolved) {
      
      // Check if all dependencies have been solved
      bool missingDep = false;
      double st = 0;
      for(auto dep : t->GetDependencies()) {
        for(auto dt : dep.second) {
          if(!solved.count(dt)) {
            missingDep = true;
            break;
          }
          st = std::max(st,endTimes[dt]);
        }
        if(missingDep)
          break;
      }

      if(missingDep)
        continue;

      startTimes[t] = st;
      toRemove.push_back(t);
      pq.push(std::make_pair(st,t));
    }
    for(auto t : toRemove) {
      unsolved.erase(t);
    }
  }

  if(solved.size() != _node.solutionMap.size())
    throw RunTimeException(WHERE) << "Did not solve all the tasks.";

  return true;
}

NextBestSearch::GroupPathType*
NextBestSearch::
QueryPath(SemanticTask* _task, const double& _startTime, const Node& _node) {

  auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto solution = mg->GetMPSolution();
  auto lib = this->GetMPLibrary();
  auto problem = this->GetMPProblem();
  auto group = _task->GetGroupMotionTask()->GetRobotGroup();

  // Set formations
  auto grm = solution->GetGroupRoadmap(group);
  grm->SetAllFormationsInactive();
  for(auto f : _task->GetFormations()) {
    grm->SetFormationActive(f);
  }

  // Configure GroupQuery
  auto query = dynamic_cast<GroupQuery<MPTraits<Cfg>>*>(
    lib->GetMapEvaluator(m_queryLabel)
  );

  query->SetPathWeightFunction(
    [this](typename GroupRoadmapType::adj_edge_iterator& _ei, 
           const double _sourceDistance,
           const double _targetDistance) {
      return this->RobotGroupPathWeight(_ei,_sourceDistance,_targetDistance);
    }
  );

  // Set current constraint set to this nodes constraint for this task
  m_currentConstraints = &(_node.constraintMap.at(_task));

  // Solve task
  lib->Solve(problem,_task->GetGroupMotionTask().get(),solution,m_queryStrategy,
             LRand(),this->GetNameAndLabel()+"::"+_task->GetLabel());

  query->SetPathWeightFunction(nullptr);
 
  auto path = solution->GetGroupPath(group);
  if(path->Size() == 0)
    return nullptr;

  // Make path copy
  auto newPath = new GroupPathType(solution->GetGroupRoadmap(group));
  *newPath = *path;

  return newPath;
}

std::vector<std::pair<SemanticTask*,NextBestSearch::Constraint>>
NextBestSearch::
ValidationFunction(Node& _node) {

  std::vector<SemanticTask*> ordering;

  auto lib = this->GetMPLibrary();
  auto vc = static_cast<CollisionDetectionValidity<MPTraits<Cfg>>*>(
              lib->GetValidityChecker(m_vcLabel));

  std::unordered_map<SemanticTask*,std::vector<GroupCfg>> cfgPaths;

  std::unordered_map<SemanticTask*,size_t> startTimes;
  std::unordered_map<SemanticTask*,size_t> endTimes;
  size_t finalTime = 0;


  // Build in order sequence of tasks
  while(ordering.size() < _node.solutionMap.size()) {

    // Find the set of tasks that are ready to be validated
    for(auto kv :_node.solutionMap) {
      auto task = kv.first;

      // Skip if already validated
      if(std::find(ordering.begin(),ordering.end(),task) != ordering.end())
        continue;

      // Check if dependencies have been validated
      size_t startTime = 0;
      bool ready = true;
      for(auto dep : task->GetDependencies()) {
        for(auto t : dep.second) {
          if(std::find(ordering.begin(),ordering.end(),t) == ordering.end()) {
            ready = false;
            break;
          }

          startTime = std::max(endTimes[t],startTime);
        }
      }

      if(!ready)
        continue;

      // Recreate the paths at resolution level
      const auto& path = kv.second;
      auto start = path->GetRoadmap()->GetVertex(path->VIDs().front());

      cfgPaths[task] = {start};
      const auto cfgs = path->FullCfgs(lib);
      for(const auto& cfg : cfgs) {
        cfgPaths[task].push_back(cfg);
      }

      startTimes[task] = startTime;
      endTimes[task] = startTime + cfgPaths[task].size() - 1;
      finalTime = std::max(endTimes[task],finalTime);
      ordering.push_back(task);
    }
  }

  // Validate the paths in the plans
  for(size_t t = 0; t < finalTime; t++) {
    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end(); iter1++) {
      auto t1 = iter1->first;

      // TODO::Check backfill of time gaps between robots doing anything
      // Check that timesteps lies within task range
      if(startTimes[t1] > t or endTimes[t1] < t)
        continue;

      // Configure first group at timestep
      const auto& path1  = iter1->second;
      const size_t step1 = std::min(t,endTimes[t1]) - startTimes[t1];
      const auto& cfg1   = path1[step1];
      const auto group1  = cfg1.GetGroupRoadmap()->GetGroup();
      for(auto robot : group1->GetRobots()) {
        cfg1.GetRobotCfg(robot).ConfigureRobot();
      }

      auto iter2 = iter1;
      iter2++;
      for(; iter2 != cfgPaths.end(); iter2++) {
        auto t2 = iter2->first;

        // TODO::Check backfill of time gaps between robots doing anything
        // Check that timesteps lies within task range
        if(startTimes[t2] > t or endTimes[t2] < t)
          continue;

        // Configure second group at timestep
        const auto& path2  = iter2->second;
        const size_t step2 = std::min(t,endTimes[t2]) - startTimes[t2];
        const auto& cfg2   = path2[step2];
        const auto group2  = cfg2.GetGroupRoadmap()->GetGroup();
        for(auto robot : group2->GetRobots()) {
          cfg2.GetRobotCfg(robot).ConfigureRobot();
        }

        // Check for collision. If none, move on
        bool collision = false;
        for(auto robot1 : group1->GetRobots()) {
          for(auto robot2 : group2->GetRobots()) {

            if(robot1 == robot2)
              continue;

            auto multibody1 = robot1->GetMultiBody();
            auto multibody2 = robot2->GetMultiBody();
            CDInfo cdInfo;
            collision = collision or vc->IsMultiBodyCollision(cdInfo,
                multibody1,multibody2, this->GetNameAndLabel());

            if(collision) {
              if(m_debug) {
                std::cout << "Collision found between :" 
                  << robot1->GetLabel()
                  << " and "
                  << robot2->GetLabel()
                  << ", at timestep: "
                  << t
                  << " and positions\n\t1: "
                  << cfg1.GetRobotCfg(robot1).PrettyPrint()
                  << "\n\t2: "
                  << cfg2.GetRobotCfg(robot2).PrettyPrint()
                  << std::endl;
              }

              break;
            }
          }
        }

        if(!collision)
          continue;

        // Create constraints
        std::vector<std::pair<SemanticTask*,Constraint>> constraints;
        Constraint constraint1 = std::make_pair(t,cfg2);
        Constraint constraint2 = std::make_pair(t,cfg1);
        constraints.emplace_back(t1,constraint1);
        constraints.emplace_back(t2,constraint2);
        return constraints;
      }
    }
  }

  return {};
}

double
NextBestSearch::
CostFunction(Node& _node) {
  std::unordered_map<SemanticTask*,double> startTimes;
  std::unordered_map<SemanticTask*,double> endTimes;
  std::set<SemanticTask*> solved;
 
  double cost = 0;

  // Initialize maps
  for(auto kv : _node.solutionMap) {
    auto task = kv.first;
    startTimes[task] = 0;

    if(task->GetDependencies().empty()) {
      endTimes[task] = kv.second->Length();
      solved.insert(task);
    }
  }

  // Add all unaffected task solutions to solved set
  auto size = solved.size();
  do {
    size = solved.size();
    // Iterate through task set
    for(auto kv : _node.solutionMap) {
      auto task = kv.first;
      if(solved.count(task))
        continue;

      double startTime = 0;

      // Check if all dependencies have been solved
      bool missingDep = false;
      for(auto dep : task->GetDependencies()) {
        for(auto t : dep.second) {
          if(!solved.count(t)) {
            missingDep = true;
            break;
          }
          startTime = std::max(startTime,endTimes[t]);
        }
        if(missingDep)
          break;
      }

      if(missingDep)
        continue;
        
      solved.insert(task);

      startTimes[task] = startTime;
      double endTime = startTime + kv.second->Length();
      endTimes[task] = endTime;
      cost = std::max(endTime,cost);
    }
  } while(solved.size() != size);

  if(solved.size() != _node.solutionMap.size())
    throw RunTimeException(WHERE) << "Solved and solution map should be the same size.";

  return cost;
}

std::vector<NextBestSearch::Node> 
NextBestSearch::
SplitNodeFunction(Node& _node, 
                    std::vector<std::pair<SemanticTask*,Constraint>> _constraints,
                    CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType>& _lowLevel,
                    CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost) {

  std::vector<Node> newNodes;

  for(auto pair : _constraints) {
    // Unpack constraint info
    auto task = pair.first;
    auto constraint = pair.second;

    // Copy parent node
    Node child = _node;
  
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
    
void
NextBestSearch::
InitialSolutionFunction(std::vector<Node>& _root, std::vector<SemanticTask*> _tasks,
                        CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType>& _lowLevel,
                        CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost) {

  Node node;

  // Find one of the initial tasks
  SemanticTask* initTask = nullptr;
  for(auto task : _tasks) {
    if(!initTask and task->GetDependencies().empty()) {
      initTask = task;
    }

    node.solutionMap[task] = nullptr;
    node.constraintMap[task] = {};
  }

  // Plan tasks
  _lowLevel(node,initTask);

  // Set node cost
  node.cost = _cost(node);

  _root.push_back(node);
}

double
NextBestSearch::
RobotGroupPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
              const double _sourceTimestep, const double _bestTimestep) const {

  // Compute time when we will end this edge.
  const size_t startTime = static_cast<size_t>(std::llround(_sourceTimestep));
  const size_t endTime = startTime + _ei->property().GetTimeSteps();

  // If this end time isn't better than current best, we won't use it.
  // Return without checking conflicts to save computation.
  if(endTime >= static_cast<size_t>(std::llround(_bestTimestep)))
    return endTime;

  // If there are no current conflicts, there is nothing to check
  if(!m_currentConstraints)
    return endTime;

  // There is at least one conflict. Find the set which occurs between this
  // edge's start and end time.
  auto lower = LowerBound(startTime);
  auto upper = UpperBound(endTime);

  // If all of the conflicts happen before or after now, there is nothing to check.
  const bool beforeNow = lower == m_currentConstraints->end();
  if(beforeNow)
    return endTime;

  const bool afterNow = upper == m_currentConstraints->begin();
  if(afterNow)
    return endTime;

  // Check the conflict set to see if this edge hits any of them.
  for(auto iter = lower; iter != upper; ++iter) {
    // TODO::ADD BACK IN SIPP BEHAVIORS

    // Check if the conflict gcfg hits this edge
    const bool hitsEdge = !IsEdgeSafe(_ei->source(), _ei->target(), *iter, startTime);
    if(!hitsEdge)
      continue;

    if(this->m_debug) {
      const GroupCfg& gcfg = iter->second;
      std::cout << "Edge (" << _ei->source() << ","
                << _ei->target() << ") collides against group "
                << gcfg.GetGroupRoadmap()->GetGroup()->GetLabel()
                << " at " << gcfg.PrettyPrint()
                << "." << std::endl;
    }

    // The conflict blocks this edge
    return std::numeric_limits<double>::infinity();
  }

  // There is no conflict and the end time is better!
  return endTime;
}

bool
NextBestSearch::
IsEdgeSafe(const VID _source, const VID _target, const Constraint _constraint,
           const size_t _startTime) const {

  auto lib = this->GetMPLibrary();
  auto group = lib->GetGroupTask()->GetRobotGroup();
  auto grm = lib->GetMPSolution()->GetGroupRoadmap(group);

  // Reconstruct edge path at resolution level
  std::vector<GroupCfg> path;
  path.push_back(grm->GetVertex(_source));
  std::vector<GroupCfg> edge = lib->ReconstructEdge(grm,_source,_target);
  path.insert(path.end(),edge.begin(),edge.end());
  path.push_back(grm->GetVertex(_target));

  // Get validity checker and make sure it is a collision detection method
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits<Cfg>>*>(
                    lib->GetValidityChecker(m_vcLabel));

  // Configure the other group at the constraint
  auto constraintCfg = _constraint.second;
  constraintCfg.ConfigureRobot();
  auto constraintGroup = constraintCfg.GetGroupRoadmap()->GetGroup();

  // Check each configuration in the resolution-level path for 
  // collision with the constraint cfg
  CDInfo cdInfo;
  for(const auto& gcfg : path) {
    gcfg.ConfigureRobot();

    for(auto r1 : group->GetRobots()) {
      auto mb1 = r1->GetMultiBody();
      for(auto r2 : constraintGroup->GetRobots()) {
        auto mb2 = r2->GetMultiBody();
        if(vc->IsMultiBodyCollision(cdInfo,mb1,mb2,this->GetNameAndLabel())) {
          return false;
        }
      }
    }
  }

  return true;
}
    
NextBestSearch::ConstraintSet::iterator 
NextBestSearch::
LowerBound(size_t _bound) const {
  auto boundIt = m_currentConstraints->end();
  for(auto it = m_currentConstraints->begin(); it != m_currentConstraints->end(); it++) {
    if(it->first >= _bound) {
      boundIt = it;
      break;
    }
  }

  return boundIt;
}

NextBestSearch::ConstraintSet::iterator 
NextBestSearch::
UpperBound(size_t _bound) const {
  auto boundIt = m_currentConstraints->end();
  for(auto it = m_currentConstraints->begin(); it != m_currentConstraints->end(); it++) {
    if(it->first > _bound) {
      boundIt = it;
      break;
    }
  }

  return boundIt;
}
/*------------------------------------------------------------*/
