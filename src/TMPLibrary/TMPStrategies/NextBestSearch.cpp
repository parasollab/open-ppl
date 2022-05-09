#include "NextBestSearch.h"

#include "MPLibrary/MapEvaluators/SIPPMethod.h"

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

  m_savePaths =_node.Read("savePaths",false,m_savePaths,
        "Flag to indicate if full paths should be output in files for reuse.");
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
      stats->SetStat(this->GetNameAndLabel() + "::LowerBound",lowerBound);
      ComputeMotions(bestNode);
    }

    // TODO::Store solution in solution set
  }

  stats->SetStat(this->GetNameAndLabel() + "::BestCost",bestNode.cost);
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
ComputeIntervals(SemanticTask* _task, const Node& _node) {
  MethodTimer mt(this->GetPlan()->GetStatClass(),
    this->GetNameAndLabel()+"::ComputeIntervals");

  auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto solution = mg->GetMPSolution();
  auto group = _task->GetGroupMotionTask()->GetRobotGroup();
  auto grm = solution->GetGroupRoadmap(group);

  m_vertexIntervals.clear();
  m_edgeIntervals.clear();

  /*
  auto si = this->GetMPLibrary()->GetMPTools()->GetSafeIntervalTool(m_safeIntervalLabel);

  for(auto vit = grm->begin(); vit != grm->end(); vit++) {

    m_vertexIntervals[vit->descriptor()] = si->ComputeIntervals(
        vit->property());

    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      m_edgeIntervals[eit->source()][eit->target()] = si->ComputeIntervals(
            eit->property(),eit->source(),eit->target(),grm);
    }
  }*/

  const auto& constraints = _node.constraintMap.at(_task);

  UnsafeVertexIntervals vertexUnsafeIntervals;
  UnsafeEdgeIntervals edgeUnsafeIntervals;

  for(auto c : constraints) {
    size_t index = c.second;
    const auto& vertexIntervals = m_unsafeVertexIntervalMap[index][_task];
    for(const auto& kv : vertexIntervals) {
      const auto& vid = kv.first;
      const auto& unsafes = kv.second;
      for(const auto& unsafe : unsafes) {
        vertexUnsafeIntervals[vid].push_back(unsafe);
      }
    }

    const auto& edgeIntervals = m_unsafeEdgeIntervalMap[index][_task];
    for(const auto& kv : edgeIntervals) {
      const auto& edge = kv.first;
      const auto& unsafes = kv.second;
      for(const auto& unsafe : unsafes) {
        edgeUnsafeIntervals[edge].push_back(unsafe);
      }
    }
  }

  for(auto vit = grm->begin(); vit != grm->end(); vit++) {

    m_vertexIntervals[vit->descriptor()] = ConstructSafeIntervals(vertexUnsafeIntervals[vit->descriptor()]);
 
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      m_edgeIntervals[eit->source()][eit->target()] = ConstructSafeIntervals(
        edgeUnsafeIntervals[std::make_pair(eit->source(),eit->target())]);
    }
  }
  
}

std::vector<Range<double>>
NextBestSearch::
ConstructSafeIntervals(std::vector<Range<double>> _unsafeIntervals) {

  // Return infinite interval if there are no unsafe intervals
  if(_unsafeIntervals.empty())
    return {Range<double>(0,std::numeric_limits<double>::infinity())};

  // Merge unsafe intervals
  std::set<size_t> merged;
  std::vector<Range<double>> unsafeIntervals;

  const size_t size = _unsafeIntervals.size();
  for(size_t i = 0; i < size; i++) {
    if(merged.count(i))
      continue;

    const auto& interval1 = _unsafeIntervals[i];

    double min = interval1.min;
    double max = interval1.max;

    for(size_t j = i+1; j < _unsafeIntervals.size(); j++) {
      if(merged.count(j))
        continue;

      const auto& interval2 = _unsafeIntervals[j];
      
      // Check if there is no overlap
      if(interval2.min > max or interval2.max < min)
        continue;

      // If there is, merge the intervals
      min = std::min(min,interval2.min);
      max = std::max(max,interval2.max);

      merged.insert(j);
    }

    Range<double> interval(min,max);
    unsafeIntervals.push_back(interval);
  }

  struct less_than {
    inline bool operator()(const Range<double>& _r1, const Range<double>& _r2) {
      return _r1.min < _r2.min;
    }
  };

  std::sort(unsafeIntervals.begin(), unsafeIntervals.end(),less_than());

  // Construct set of intervals
  std::vector<Range<double>> intervals;

  const double timeRes = this->GetMPProblem()->GetEnvironment()->GetTimeRes();
  double min = 0;
  double max = std::numeric_limits<double>::infinity();

  auto iter = unsafeIntervals.begin();
  while(iter != unsafeIntervals.end()) {
    max = iter->min - timeRes;
    if(min < max)
      intervals.emplace_back(min,max);
    min = iter->max + timeRes;
    iter++;
  }

  max = std::numeric_limits<double>::infinity();
  intervals.emplace_back(min,max);

  return intervals;
}

/*----------------------- CBS Functors -----------------------*/

bool 
NextBestSearch::
LowLevelPlanner(Node& _node, SemanticTask* _task) {
  const double timeRes = this->GetMPProblem()->GetEnvironment()->GetTimeRes();
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
        //if(timesteps > 0) {
        //  endTimes[task] = startTime + timesteps; //- 1;
        //  if(startTime == 0) {
        //    endTimes[task] = endTimes[task] - 1;
        //  }
        //}
        else {
          //endTimes[task] = 0;
          endTimes[task] = -1;
        }
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
      //if(timesteps > 0)
      //  endTimes[task] = startTime + timesteps;// - 1;
      //else 
      //  endTimes[task] = startTime;
      if(timesteps > 0) {
        endTimes[task] = startTime + timesteps; //- 1;
        if(startTime == 0) {
          endTimes[task] = endTimes[task] - 1;
        }
      }
      else {
        endTimes[task] = 0;
      }
    }
  } while(solved.size() != size);
 
  // Init queue of tasks to solve
  std::priority_queue<std::pair<double,SemanticTask*>,
                      std::vector<std::pair<double,SemanticTask*>>,
                      std::greater<std::pair<double,SemanticTask*>>> pq;

  pq.push(std::make_pair(startTimes[_task],_task));

  std::set<SemanticTask*> unsolved;
  for(auto kv : _node.solutionMap) {
    if(solved.count(kv.first) or kv.first == _task)
      continue;
    unsolved.insert(kv.first);
  }

  // Plan unsolved tasks
  while(!pq.empty()) {
    auto current = pq.top();
    pq.pop();
    auto task = current.second;
    auto startTime = current.first;

    if(m_debug) {
      std::cout << "Start time for " << task->GetLabel() << " : TIME: " << startTime << std::endl;
    }
    // Compute new path
    auto path = QueryPath(task,startTime*timeRes,_node);

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
    //if(timesteps > 0)
    //  endTimes[task] = startTime + timesteps;// - 1;
    //else 
    //  endTimes[task] = startTime;
    if(timesteps > 0) {
      endTimes[task] = startTime + timesteps; //- 1;
      if(startTime == 0) {
        endTimes[task] = endTimes[task] - 1;
      }
    }
    else {
      endTimes[task] = 0;
    }

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
QueryPath(SemanticTask* _task, const double _startTime, const Node& _node) {
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
      auto constraintCfg = m_conflicts[c.second];
      RobotGroup* constraintGroup = constraintCfg.GetGroupRoadmap()->GetGroup();
      for(auto robot : constraintGroup->GetRobots()) {
        auto cfg = constraintCfg.GetRobotCfg(robot);
        std::vector<Cfg> path = {cfg,cfg,cfg};
        
        //auto duration = c.first.second - c.first.first;
        //for(size_t i = 0; i < duration; i++) {
        //  path.push_back(cfg);
        //}

        DynamicObstacle dob(cfg.GetRobot(),path);
        dob.SetStartTime(c.first.first-1);
        dob.SetEndTime(c.first.second);
        this->GetMPProblem()->AddDynamicObstacle(std::move(dob));
      } 
    }
  }

  ComputeIntervals(_task,_node);

  const double timeRes = this->GetMPProblem()->GetEnvironment()->GetTimeRes();
  auto q = dynamic_cast<SIPPMethod<MPTraits<Cfg>>*>(
    this->GetMPLibrary()->GetMapEvaluator(m_queryLabel)
  );
  q->SetMinEndTime(double(lastTimestep) * timeRes);
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

  if(m_debug) {
    std::cout << "VALIDATING NODE" << std::endl;
  }

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  const double timeRes = this->GetMPProblem()->GetEnvironment()->GetTimeRes();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

  std::vector<SemanticTask*> ordering;

  auto lib = this->GetMPLibrary();
  auto vc = static_cast<CollisionDetectionValidityMethod<MPTraits<Cfg>>*>(
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

      //auto timesteps = cfgPaths[task].size();
      auto timesteps = path->TimeSteps();
      if(timesteps > 0) {
        endTimes[task] = startTime + timesteps; //- 1;
        if(startTime == 0) {
          endTimes[task] = endTimes[task] - 1;
        }
      }
      else {
        endTimes[task] = startTime;
      }

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
                  
                std::cout << "Task starts: " << startTimes[t1] << " " << startTimes[t2] << std::endl;
              }

              auto endT = t;
              bool group1Passive = true;
              bool group2Passive = true;
              for(auto r : group1->GetRobots()) {
                if(!r->GetMultiBody()->IsPassive()) {
                  group1Passive = false;
                  break;
                }
              }
              for(auto r : group2->GetRobots()) {
                if(!r->GetMultiBody()->IsPassive()) {
                  group2Passive = false;
                  break;
                }
              }
              if(group1Passive) {
                endT = endTimes[t1];
              }
              else if(group2Passive) {
                endT = endTimes[t2];
              }

              stats->IncStat(this->GetNameAndLabel() + "::CollisionFound");

              // Temp hack for demo
              //endT = std::max(endT,t+100);

              // Check if conflict has been found before
              size_t c1Index = MAX_INT;
              size_t c2Index = MAX_INT;
              //for(size_t i = 0; i < m_conflicts.size(); i++) {
              for(auto constraint : _node.constraintMap[t1]) {
                size_t i = constraint.second;
                if(m_conflicts[i] == cfg2) {
                  c1Index = i;
                  break;
                }
              }
              for(auto constraint : _node.constraintMap[t2]) {
                size_t i = constraint.second;
                if(m_conflicts[i] == cfg1) {
                  c2Index = i;
                  break;
                }
              }

              // Create constraints
              if(c1Index == MAX_INT and c2Index == MAX_INT) {

                // Configure constraint 1
                c1Index = m_conflicts.size();
                m_conflicts.push_back(cfg2);

                std::map<SemanticTask*,UnsafeVertexIntervals> taskVertexIntervals;
                std::map<SemanticTask*,UnsafeEdgeIntervals> taskEdgeIntervals;

                taskVertexIntervals[t1] = {};
                taskEdgeIntervals[t1] = {};

                m_unsafeVertexIntervalMap.push_back(taskVertexIntervals);
                m_unsafeEdgeIntervalMap.push_back(taskEdgeIntervals);

                // Configure constraint 2
                c2Index = m_conflicts.size();
                m_conflicts.push_back(cfg1);

                taskVertexIntervals = std::map<SemanticTask*,UnsafeVertexIntervals>();
                taskEdgeIntervals = std::map<SemanticTask*,UnsafeEdgeIntervals>();

                taskVertexIntervals[t2] = {};
                taskEdgeIntervals[t2] = {};

                m_unsafeVertexIntervalMap.push_back(taskVertexIntervals);
                m_unsafeEdgeIntervalMap.push_back(taskEdgeIntervals);
              }

              std::vector<std::pair<SemanticTask*,Constraint>> constraints;
              Constraint constraint1 = std::make_pair(std::make_pair(t,endT),c1Index);
              Constraint constraint2 = std::make_pair(std::make_pair(t,endT),c2Index);
 
              if(c1Index != MAX_INT)
                constraints.emplace_back(t1,constraint1);
              if(c2Index != MAX_INT)
                constraints.emplace_back(t2,constraint2);


              auto edge1 = _node.solutionMap[t1]->GetEdgeAtTimestep(t-startTimes[t1]);
              auto edge2 = _node.solutionMap[t2]->GetEdgeAtTimestep(t-startTimes[t2]);

              if(m_debug) {
                std::cout << "Edge 1: " << edge1 << std::endl;
                std::cout << "Edge 2: " << edge2 << std::endl;
              }

              double duration1 = 0;
              double duration2 = 0;

              if(edge1.first != edge1.second) {
                duration1 = double(_node.solutionMap[t1]->GetRoadmap()->GetEdge(
                                   edge1.first,edge1.second).GetTimeSteps()) * timeRes;
              }
      
              if(edge2.first != edge2.second) {
                duration2 = double(_node.solutionMap[t2]->GetRoadmap()->GetEdge(
                                   edge2.first,edge2.second).GetTimeSteps()) * timeRes;
              }

              const double constraintStart = double(t) * timeRes;
              const double constraintEnd = double(endT) * timeRes;

              Range<double> interval1(std::max(0.,constraintStart-duration1),constraintEnd);
              Range<double> interval2(std::max(0.,constraintStart-duration2),constraintEnd);

              if(c1Index != MAX_INT) {
                if(edge1.first == edge1.second) {
                  /*
                  UnsafeVertexIntervals intervals;
                  intervals[edge1.first] = {interval1};
  
                  std::map<SemanticTask*,UnsafeVertexIntervals> taskVertexIntervals;
                  std::map<SemanticTask*,UnsafeEdgeIntervals> taskEdgeIntervals;
  
                  taskVertexIntervals[t1] = intervals;
                  taskEdgeIntervals[t1] = {};
  
                  m_unsafeVertexIntervalMap.push_back(taskVertexIntervals);
                  m_unsafeEdgeIntervalMap.push_back(taskEdgeIntervals);
                  */
  
                  auto& intervals = m_unsafeVertexIntervalMap[c1Index][t1][edge1.first];

                  // Debug - remove when working
                  for(auto elem : intervals) {
                    if(elem == interval1) {
                      std::cout << "OH CRAP" << std::endl;
                    }
                  }

                  intervals.push_back(interval1);

                }
                else {
                  /*
                  UnsafeEdgeIntervals intervals;
                  intervals[edge1] = {interval1};
  
                  std::map<SemanticTask*,UnsafeVertexIntervals> taskVertexIntervals;
                  std::map<SemanticTask*,UnsafeEdgeIntervals> taskEdgeIntervals;
  
                  taskVertexIntervals[t1] = {};
                  taskEdgeIntervals[t1] = intervals;
  
                  m_unsafeVertexIntervalMap.push_back(taskVertexIntervals);
                  m_unsafeEdgeIntervalMap.push_back(taskEdgeIntervals);
                  */
    
                  auto& intervals = m_unsafeEdgeIntervalMap[c1Index][t1][edge1];
                  
                  // Debug - remove when working
                  for(auto elem : intervals) {
                    if(elem == interval1) {
                      std::cout << "OH CRAP" << std::endl;
                    }
                  }

                  intervals.push_back(interval1);
                }
              }

              if(c2Index != MAX_INT) {
                if(edge2.first == edge2.second) {
                  /*
                  UnsafeVertexIntervals intervals;
                  intervals[edge2.first] = {interval2};
  
                  std::map<SemanticTask*,UnsafeVertexIntervals> taskVertexIntervals;
                  std::map<SemanticTask*,UnsafeEdgeIntervals> taskEdgeIntervals;
  
                  taskVertexIntervals[t2] = intervals;
                  taskEdgeIntervals[t2] = {};
  
                  m_unsafeVertexIntervalMap.push_back(taskVertexIntervals);
                  m_unsafeEdgeIntervalMap.push_back(taskEdgeIntervals);
                  */
  
                  auto& intervals = m_unsafeVertexIntervalMap[c2Index][t2][edge2.first];

                  // Debug - remove when working
                  for(auto elem : intervals) {
                    if(elem == interval2) {
                      std::cout << "OH CRAP" << std::endl;
                    }
                  }

                  intervals.push_back(interval2);
                }
                else {
                  /*
                  UnsafeEdgeIntervals intervals;
                  intervals[edge2] = {interval2};
  
                  std::map<SemanticTask*,UnsafeVertexIntervals> taskVertexIntervals;
                  std::map<SemanticTask*,UnsafeEdgeIntervals> taskEdgeIntervals;
  
                  taskVertexIntervals[t2] = {};
                  taskEdgeIntervals[t2] = intervals;
  
                  m_unsafeVertexIntervalMap.push_back(taskVertexIntervals);
                  m_unsafeEdgeIntervalMap.push_back(taskEdgeIntervals);
                  */
  
                  auto& intervals = m_unsafeEdgeIntervalMap[c2Index][t2][edge2];

                  // Debug - remove when working
                  for(auto elem : intervals) {
                    if(elem == interval2) {
                      std::cout << "OH CRAP" << std::endl;
                    }
                  }

                  intervals.push_back(interval2);
                }
              }

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
      solved.insert(task);
      if(timesteps > 0)
        //endTimes[task] = timesteps; // - 1;
        endTimes[task] = timesteps - 1;
      else
        endTimes[task] = 0;
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
      double endTime = 0;
      //if(timesteps > 0)
      //  endTimes[task] = startTime + timesteps;// - 1;
      //else 
      //  endTimes[task] = startTime;
      if(timesteps > 0) {
        endTimes[task] = startTime + timesteps; //- 1;
        if(startTime == 0) {
          endTimes[task] = endTimes[task] - 1;
        }
      }
      else {
        endTimes[task] = 0;
      }
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
      const GroupCfg& gcfg = m_conflicts[iter->second];
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
  auto e = grm->GetEdge(_source,_target);
  std::vector<GroupCfg> edge = !e.GetIntermediates().empty() ? e.GetIntermediates()
                             : lib->ReconstructEdge(grm,_source,_target);
  path.insert(path.end(),edge.begin(),edge.end());
  path.push_back(grm->GetVertex(_target));

  // Get validity checker and make sure it is a collision detection method
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits<Cfg>>*>(
                    lib->GetValidityChecker(m_vcLabel));

  // Configure the other group at the constraint
  auto constraintCfg = m_conflicts[_constraint.second];
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
      auto timesteps = path->TimeSteps();
      //auto timesteps = cfgPaths[task].size();
      //if(timesteps > 0)
      //  endTimes[task] = startTime + timesteps;// - 1;
      //else 
      //  endTimes[task] = startTime;
      if(timesteps > 0) {
        endTimes[task] = startTime + timesteps; //- 1;
        if(startTime == 0) {
          endTimes[task] = endTimes[task] - 1;
        }
      }
      else {
        endTimes[task] = 0;
      }

      if(m_debug) {
        std::cout << task->GetLabel() 
                  << " start: "
                  << startTime
                  << ". end: "
                  << endTimes[task]
                  << std::endl;
      }

      finalTime = std::max(endTimes[task],finalTime);
      ordering.push_back(task);

      // Update the end times of the preceeding tasks
      for(auto dep : task->GetDependencies()) {
        for(auto t : dep.second) {
          endTimes[t] = startTime;

          if(m_debug) {
            std::cout << "Updating end time of " << t->GetLabel()
                      << " to " << startTime
                      << " because of " << task->GetLabel()
                      << std::endl;
          } 
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
        // Hack bc of direspect to path constraints
        auto cfg = cfg1.GetRobotCfg(robot);
        if(!robot->GetMultiBody()->IsPassive() and group1->Size() > 1)
          cfg[1] = .0001;
        robotPaths[robot].push_back(cfg);
      }
    }
  }

  // Find changes in gripper dof and add buffers
  std::vector<size_t> switches;
  for(size_t t = 1; t < finalTime; t++) {
    for(auto& kv : robotPaths) {
      if(kv.first->GetMultiBody()->IsPassive())
        continue;

      const auto& path = kv.second;

      if(t >= path.size())
        continue;

      auto cfg1 = path[t-1];
      auto cfg2 = path[t];
      
      if(abs(cfg1[1] - cfg2[1]) >= .00001) {
        if(switches.empty() or switches.back() != t-1)
          switches.push_back(t-1);
        switches.push_back(t);
        std::cout << "Found switch at " << t-1 << std::endl;
        break;
      }
    }
  }

  const size_t numCopies = 10;

  for(size_t i = 0; i < switches.size(); i++) {
    size_t t = switches[i] + numCopies*i;
    for(auto& kv : robotPaths) {
      auto& path = kv.second;

      if(t >= path.size())
        continue;

      auto cfg = path[t];

      path.insert(path.begin()+t,numCopies,cfg);
    }
  }

  for(auto kv : robotPaths) {
    if(m_debug) {
      std::cout << "PATH FOR: " << kv.first->GetLabel() << std::endl;
      std::cout << "PATH LENGTH: " << kv.second.size() << std::endl;
    }

    const std::string filename = this->GetMPProblem()->GetBaseFilename() 
                               + "::FinalPath::" + kv.first->GetLabel();

    std::ofstream ofs(filename);

    for(size_t i = 0; i < kv.second.size(); i++) {
      auto cfg = kv.second[i];
      if(m_debug) {
        std::cout << "\t" << i << ": " << cfg.PrettyPrint() << std::endl;
      }
      if(m_savePaths) {
        ofs << cfg << "\n";
      }
    }
    ofs.close();
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
