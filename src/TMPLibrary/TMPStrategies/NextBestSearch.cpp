#include "NextBestSearch.h"

#include "MPLibrary/MapEvaluators/GroupSIPPMethod.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"
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

  m_safeIntervalLabel = _node.Read("safeIntervalLabel",true,"",
        "Safe interval tool to use for compute safe intervals of roadmap.");              
}

NextBestSearch::
~NextBestSearch() {}

/*------------------------ Interface -------------------------*/

/*--------------------- Helper Functions ---------------------*/

void
NextBestSearch::
PlanTasks() {

  if(m_debug) 
    std::cout << this->GetNameAndLabel() + "::Starting to PlanTasks." << std::endl;

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::PlanTasks");
  
  stats->SetStat(this->GetNameAndLabel() + "::CollisionFound",0);

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

    std::cout << "Computing plans. Current upperbound: "
              << bestNode.cost
              << ". Current lowerbound: "
              << lowerBound
              << "."
              << std::endl;
  
    lowerBound = FindTaskPlan(originalDecomp);
    taskSolutions.push_back(plan->GetDecomposition());

    if(bestNode.cost > lowerBound) {
      ComputeMotions(bestNode);
    }

    // TODO::Store solution in solution set
  }

  // Save plan in proper format
  SaveSolution(bestNode);
}
    
double
NextBestSearch::
FindTaskPlan(Decomposition* _decomp) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::FindTaskPlan");

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

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ComputeMotions");

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

void
NextBestSearch::
ComputeIntervals(GroupRoadmapType* _grm) {
  MethodTimer mt(this->GetPlan()->GetStatClass(),
    this->GetNameAndLabel()+"::ComputeIntervals");

  m_vertexIntervals.clear();
  m_edgeIntervals.clear();

  auto si = this->GetMPLibrary()->GetMPTools()->GetSafeIntervalTool(m_safeIntervalLabel);

  for(auto vit = _grm->begin(); vit != _grm->end(); vit++) {

    m_vertexIntervals[vit->descriptor()] = si->ComputeIntervals(
        vit->property());

    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      m_edgeIntervals[eit->source()][eit->target()] = si->ComputeIntervals(
            eit->property(),eit->source(),eit->target(),_grm);
    }
  }
}

/*----------------------- CBS Functors -----------------------*/

bool 
NextBestSearch::
LowLevelPlanner(Node& _node, SemanticTask* _task) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CBSLowLevelPlanner");

  std::unordered_map<SemanticTask*,double> startTimes;
  std::unordered_map<SemanticTask*,double> endTimes;
  std::set<SemanticTask*> solved;
 
  // Initialize maps
  for(auto kv : _node.solutionMap) {
    auto task = kv.first;
    startTimes[task] = 0;

    if(task->GetDependencies().empty() and task != _task) {
      if(kv.second) {
        auto timesteps = kv.second->TimeSteps();
        if(timesteps > 0)
          endTimes[task] = timesteps-1;
        else
          endTimes[task] = 0;
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
      //endTimes[task] = startTime + kv.second->TimeSteps();
      auto timesteps = kv.second->TimeSteps();
      if(timesteps > 0)
        endTimes[task] = startTime + timesteps;// - 1;
      else 
        endTimes[task] = startTime;
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
    if(!path) {
      if(m_debug) {
        std::cout << "Failed to find path for " << task->GetLabel() << std::endl;
      }
      return false;
    }

    // Save path to solution
    _node.solutionMap[task] = path;
    solved.insert(task);

    auto timesteps = path->TimeSteps();
    if(timesteps > 0)
      endTimes[task] = startTime + timesteps;// - 1;
    else 
      endTimes[task] = startTime;

    if(m_debug) {
      std::cout << "Found path for " << task->GetLabel() 
                << " from " << startTime << " to "
                << startTime + timesteps << std::endl;
    }

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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::QueryPath");

  auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto solution = mg->GetMPSolution();
  auto lib = this->GetMPLibrary();
  lib->SetTask(nullptr);
  lib->SetGroupTask(_task->GetGroupMotionTask().get());
  auto problem = this->GetMPProblem();
  auto group = _task->GetGroupMotionTask()->GetRobotGroup();

  // Set formations
  auto grm = solution->GetGroupRoadmap(group);
  grm->SetAllFormationsInactive();
  for(auto f : _task->GetFormations()) {
    grm->SetFormationActive(f);
  }

  // Configure GroupQuery
  /*auto query = dynamic_cast<GroupQuery<MPTraits<Cfg>>*>(
    lib->GetMapEvaluator(m_queryLabel)
  );

  query->SetPathWeightFunction(
    [this](typename GroupRoadmapType::adj_edge_iterator& _ei, 
           const double _sourceDistance,
           const double _targetDistance) {
      return this->RobotGroupPathWeight(_ei,_sourceDistance,_targetDistance);
    }
  );*/

  // Set current constraint set to this nodes constraint for this task
  m_currentConstraints = &(_node.constraintMap.at(_task));

  // Turn conflicts into dyanmic obstacles
  double lastTimestep = 0;
  for(auto c : *m_currentConstraints) {

    lastTimestep = std::max(double(c.first.second),lastTimestep);

    if(false) {
      std::cout << "Check for caching here" << std::endl;
    }
    else {
      RobotGroup* constraintGroup = c.second.GetGroupRoadmap()->GetGroup();
      for(auto robot : constraintGroup->GetRobots()) {
        auto cfg = c.second.GetRobotCfg(robot);
        std::vector<Cfg> path = {cfg,cfg,cfg};
        
        auto duration = c.first.second - c.first.first;
        for(size_t i = 0; i < duration; i++) {
          path.push_back(cfg);
        }

        DynamicObstacle dob(cfg.GetRobot(),path);
        dob.SetStartTime(c.first.first-1);
        this->GetMPProblem()->AddDynamicObstacle(std::move(dob));
      } 
    }
  }

  ComputeIntervals(grm);

  auto q = dynamic_cast<GroupSIPPMethod<MPTraits<Cfg>>*>(
    this->GetMPLibrary()->GetMapEvaluator(m_queryLabel)
  );
  q->SetMinEndtime(lastTimestep);
  q->SetStartTime(_startTime);

  q->SetEdgeIntervals(m_edgeIntervals);
  q->SetVertexIntervals(m_vertexIntervals);

  // Solve task
  lib->Solve(problem,_task->GetGroupMotionTask().get(),solution,m_queryStrategy,
             LRand(),this->GetNameAndLabel()+"::"+_task->GetLabel());

  //query->SetPathWeightFunction(nullptr);
  this->GetMPProblem()->ClearDynamicObstacles();
 
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
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

  std::vector<SemanticTask*> ordering;

  auto lib = this->GetMPLibrary();
  auto vc = static_cast<CollisionDetectionValidity<MPTraits<Cfg>>*>(
              lib->GetValidityChecker(m_vcLabel));

  std::unordered_map<SemanticTask*,std::vector<GroupCfg>> cfgPaths;

  std::unordered_map<SemanticTask*,size_t> startTimes;
  std::unordered_map<SemanticTask*,size_t> endTimes;
  size_t finalTime = 0;

  // Set of tasks which have no tasks dependent on them
  std::set<SemanticTask*> finalTasks;

  // Build in order sequence of tasks
  while(ordering.size() < _node.solutionMap.size()) {

    // Find the set of tasks that are ready to be validated
    for(auto kv :_node.solutionMap) {
      auto task = kv.first;
      lib->SetGroupTask(task->GetGroupMotionTask().get()); 

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
     
      const auto cfgs = path->FullCfgsWithWait(lib);
      for(size_t i = 0; i < cfgs.size(); i++) {
        cfgPaths[task].push_back(cfgs[i]);
      }

      startTimes[task] = startTime;

      auto timesteps = cfgPaths[task].size();
      if(timesteps > 0)
        endTimes[task] = startTime + timesteps - 1;
      else
        endTimes[task] = startTime;

      finalTime = std::max(endTimes[task],finalTime);
      ordering.push_back(task);

      finalTasks.insert(task);

      // Update the end times of the preceeding tasks
      auto group1 = task->GetGroupMotionTask()->GetRobotGroup();
      for(auto dep : task->GetDependencies()) {
        for(auto t : dep.second) {

          // Remove from final tasks
          finalTasks.erase(t);

          // Check if robots overlap
          bool overlap = false;
          auto group2 = t->GetGroupMotionTask()->GetRobotGroup();
          for(auto r1 : group1->GetRobots()) {
            for(auto r2 : group2->GetRobots()) {
              if(r1 != r2)
                continue;

              overlap = true;
              endTimes[t] = startTime;
              break;
            }
            if(overlap)
              break;
          }
        }
      }
    }
  }

  // Validate the paths in the plans
  for(size_t t = 0; t < finalTime; t++) {
    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end(); iter1++) {
      auto t1 = iter1->first;

      // TODO::Check backfill of time gaps between robots doing anything
      // Check that timesteps lies within task range
      if(startTimes[t1] > t or (endTimes[t1] < t and !finalTasks.count(t1)))
        continue;

      // Configure first group at timestep
      const auto& path1  = iter1->second;
      const size_t step1 = std::min(t - startTimes[t1],path1.size()-1);
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
        if(startTimes[t2] > t or (endTimes[t2] < t and !finalTasks.count(t2)))
          continue;

        // Configure second group at timestep
        const auto& path2  = iter2->second;
        const size_t step2 = std::min(t - startTimes[t2], path2.size()-1);
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
                  << " during tasks "
                  << t1->GetLabel()
                  << " and "
                  << t2->GetLabel()
                  << std::endl;
              }

              auto endT = t;
              if(robot1->GetMultiBody()->IsPassive()) {
                endT = endTimes[t1];
              }
              else if(robot2->GetMultiBody()->IsPassive()) {
                endT = endTimes[t2];
              }

              stats->IncStat(this->GetNameAndLabel() + "::CollisionFound");

              // Temp hack for demo
              endT = std::max(endT,t+100);

              // Create constraints
              std::vector<std::pair<SemanticTask*,Constraint>> constraints;
              Constraint constraint1 = std::make_pair(std::make_pair(t,endT),cfg2);
              Constraint constraint2 = std::make_pair(std::make_pair(t,endT),cfg1);
              constraints.emplace_back(t1,constraint1);
              constraints.emplace_back(t2,constraint2);
              return constraints;
            }
          }
        }
      }
    }
  }

  return {};
}

double
NextBestSearch::
CostFunction(Node& _node) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CostFunction");

  std::unordered_map<SemanticTask*,double> startTimes;
  std::unordered_map<SemanticTask*,double> endTimes;
  std::set<SemanticTask*> solved;
 
  double cost = 0;

  // Initialize maps
  for(auto kv : _node.solutionMap) {
    auto task = kv.first;
    startTimes[task] = 0;

    if(task->GetDependencies().empty()) {
      auto timesteps = kv.second->TimeSteps();
      if(timesteps > 0)
        endTimes[task] = timesteps;// - 1;
      else
        endTimes[task] = 0;
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
      //double endTime = startTime + kv.second->TimeSteps();
      auto timesteps = kv.second->TimeSteps();
      double endTime;
      if(timesteps > 0)
        endTime = startTime + timesteps;// - 1;
      else 
        endTime = startTime;
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
  if(!_lowLevel(node,initTask))
    throw RunTimeException(WHERE) << "No initial plan.";

  // Set node cost
  node.cost = _cost(node);

  _root.push_back(node);
}

double
NextBestSearch::
RobotGroupPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
              const double _sourceTimestep, const double _bestTimestep) const {

  // Compute time when we will end this edge.
  const double timeRes = this->GetMPProblem()->GetEnvironment()->GetTimeRes();
  const size_t startTime = static_cast<size_t>(std::llround(_sourceTimestep) * timeRes);
  const size_t endTime = startTime + (_ei->property().GetTimeSteps() * timeRes);

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
    if(it->first.first >= _bound) {
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
    if(it->first.second > _bound) {
      boundIt = it;
      break;
    }
  }

  return boundIt;
}

void
NextBestSearch::
SaveSolution(const Node& _node) {
  // TODO::Decide on final format. For now convert to paths for individual robots
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SaveSolution");

  // Collect all of the robots
  std::unordered_map<Robot*,std::vector<Cfg>> robotPaths;
  for(auto kv : _node.solutionMap) {
    auto group = kv.first->GetGroupMotionTask()->GetRobotGroup();
    for(auto robot : group->GetRobots()) {
      robotPaths[robot] = {};
    }
  }

  
  std::vector<SemanticTask*> ordering;

  auto lib = this->GetMPLibrary();

  std::unordered_map<SemanticTask*,std::vector<GroupCfg>> cfgPaths;

  std::unordered_map<SemanticTask*,size_t> startTimes;
  std::unordered_map<SemanticTask*,size_t> endTimes;
  size_t finalTime = 0;

  // Build in order sequence of tasks
  while(ordering.size() < _node.solutionMap.size()) {

    // Find the set of tasks that are ready to be validated
    for(auto kv :_node.solutionMap) {
      auto task = kv.first;
      lib->SetGroupTask(task->GetGroupMotionTask().get()); 

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

      const auto cfgs = path->FullCfgsWithWait(lib);
      for(size_t i = 0; i < cfgs.size(); i++) {
        cfgPaths[task].push_back(cfgs[i]);
      }

      startTimes[task] = startTime;
      auto timesteps = cfgPaths[task].size();
      if(timesteps > 0)
        endTimes[task] = startTime + cfgPaths[task].size() - 1;
      else
        endTimes[task] = startTime;

      finalTime = std::max(endTimes[task],finalTime);
      ordering.push_back(task);

      // Update the end times of the preceeding tasks
      for(auto dep : task->GetDependencies()) {
        for(auto t : dep.second) {
          endTimes[t] = startTime;
        }
      }
    }
  }

  // Add the cfgs to the paths
  for(size_t t = 0; t <= finalTime; t++) {
    std::unordered_set<Robot*> used;

    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end(); iter1++) {
      auto t1 = iter1->first;

      // TODO::Check backfill of time gaps between robots doing anything
      // Check that timesteps lies within task range
      if(startTimes[t1] > t or endTimes[t1] < t)
        continue;

      // Configure first group at timestep
      const auto& path1  = iter1->second;
      const size_t step1 = std::min(t - startTimes[t1],path1.size()-1);
      const auto& cfg1   = path1[step1];
      const auto group1  = cfg1.GetGroupRoadmap()->GetGroup();
      for(auto robot : group1->GetRobots()) {
        // Account for overlap at start/end points in tasks
        if(used.count(robot))
          continue;
        used.insert(robot);
        robotPaths[robot].push_back(cfg1.GetRobotCfg(robot));
      }
    }
  }

  for(auto kv : robotPaths) {
    std::cout << "PATH FOR: " << kv.first << std::endl;
    std::cout << "PATH LENGTH: " << kv.second.size() << std::endl;
    for(size_t i = 0; i < kv.second.size(); i++) {
      auto cfg = kv.second[i];
      std::cout << "\t" << i << ": " << cfg.PrettyPrint() << std::endl;
    }
    //::WritePath("hypergraph-"+kv.first->GetLabel()+".rdmp.path",kv.second);
  }

  // Naive way to create paths - will lose synchronization
  // Initialize a decomposition
  auto top = std::shared_ptr<SemanticTask>(new SemanticTask());
  Decomposition* decomp = new Decomposition(top);
  plan->SetDecomposition(decomp);
  
  for(auto kv : robotPaths) {
    auto robot = kv.first;
    auto cfgs = kv.second;

    // Create a motion task
    auto mpTask = std::shared_ptr<MPTask>(new MPTask(robot));

    // Create a semantic task
    const std::string label = robot->GetLabel() + ":PATH";
    auto task = new SemanticTask(label,top.get(),decomp,
                 SemanticTask::SubtaskRelation::AND,false,true,mpTask);

    // Create a task solution
    auto sol = std::shared_ptr<TaskSolution>(new TaskSolution(task));
    sol->SetRobot(robot);

    // Initialize mp solution and path
    auto mpsol = new MPSolution(robot);
    auto rm = mpsol->GetRoadmap(robot);

    std::vector<size_t> vids;
    for(auto cfg : cfgs) {
      auto vid = rm->AddVertex(cfg);
      vids.push_back(vid);
    }
    
    auto path = mpsol->GetPath(robot);
    *path += vids;

    // Save mp solution in task solution
    sol->SetMotionSolution(mpsol);

    // Save task solution in plan
    plan->SetTaskSolution(task,sol);
  }
  plan->Print();
}
/*------------------------------------------------------------*/
