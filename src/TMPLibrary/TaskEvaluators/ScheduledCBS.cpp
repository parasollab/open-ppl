#include "ScheduledCBS.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "TMPLibrary/StateGraphs/ModeGraph.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"
#include "TMPLibrary/TaskEvaluators/SubmodeQuery.h"

#include <map>
#include <set>


/*----------------------- Construction -----------------------*/

ScheduledCBS::
ScheduledCBS() {
  this->SetName("ScheduledCBS");
}

ScheduledCBS::
ScheduledCBS(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("ScheduledCBS");

  m_vcLabel = _node.Read("vcLabel",true,"",
         "Validity checker to use for multi-robot collision checking.");

  m_queryLabel = _node.Read("queryLabel",true,"",
         "Map Evaluator to use to query individual solutions.");

  m_queryStrategy = _node.Read("queryStrategy",true,"",
         "MPStrategy to sue to query individial paths.");

  m_bypass = _node.Read("bypass",false,m_bypass,
          "Flag to use bypass strategy.");

  m_sqLabel = _node.Read("sqLabel",true,"","SubmodeQuery label.");

  m_upperBound = std::numeric_limits<double>::infinity();

  m_buffer = _node.Read("buffer",false,m_buffer,size_t(0),SIZE_MAX,
        "Number of timesteps to buffer after a conflict.");

  m_alpha = _node.Read("alpha", false, double(0), double(0), std::numeric_limits<double>::max(), 
    "User-defined hyperparameter for the probability of quitting");

  m_X = _node.Read("X", false, double(1), double(0), std::numeric_limits<double>::max(),
    "User-defined hyperparameter for the probability of quitting");  

  m_writeSolution = _node.Read("writeSolution",false,m_writeSolution,
          "Flag to use bypass strategy.");

  m_workspace = _node.Read("workspace", false, m_workspace,
          "Bypass logic not needed for workspace dash");

}

ScheduledCBS::
~ScheduledCBS() {

}

/*------------------------ Overrides -------------------------*/
void
ScheduledCBS::
Initialize() {
 
  m_startTimes.clear();
  m_endTimes.clear();
 
  m_unsafeVertexIntervalMap.clear();
  m_unsafeEdgeIntervalMap.clear();

  m_vertexIntervals.clear();
  m_edgeIntervals.clear();

  m_scheduleAtomicDistances.clear();
}

void
ScheduledCBS::
SetUpperBound(double _upperBound) {
  m_upperBound = _upperBound;
}

bool
ScheduledCBS::
Run(Plan* _plan) {
  if(!_plan)
    _plan = this->GetPlan();

  m_quit = false;

  BuildScheduleGraph(_plan);

  // Configure CBS Functions
  CBSLowLevelPlanner<SemanticTask*,Constraint,GroupPathType*> lowLevel(
    [this](Node& _node, SemanticTask* _task) {
      return LowLevelPlanner(_node,_task);
    }
  );

  CBSValidationFunction<SemanticTask*,Constraint,GroupPathType*> validation(
    [this](Node& _node) {
      return this->ValidationFunction(_node);
    }
  );

  CBSCostFunction<SemanticTask*,Constraint,GroupPathType*> cost(
    [this](Node& _node) {
      return this->CostFunction(_node);
    }
  );

  CBSSplitNodeFunction<SemanticTask*,Constraint,GroupPathType*> splitNode(
    [this](Node& _node, std::vector<std::pair<SemanticTask*,Constraint>> _constraints,
           CBSLowLevelPlanner<SemanticTask*,Constraint,GroupPathType*>& _lowLevel,
           CBSCostFunction<SemanticTask*,Constraint,GroupPathType*>& _cost) {
      return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
    }
  );

  CBSInitialFunction<SemanticTask*,Constraint,GroupPathType*> initial(
    [this](std::vector<Node>& _root, std::vector<SemanticTask*> _task,
           CBSLowLevelPlanner<SemanticTask*,Constraint,GroupPathType*>& _lowLevel,
           CBSCostFunction<SemanticTask*,Constraint,GroupPathType*>& _cost) {
      return this->InitialSolutionFunction(_root,_task,_lowLevel,_cost);
    }
  );

  CBSEarlyTerminationFunction termination(
    [this](const size_t& _numNodes) {
      return this->m_quit or this->EarlyTermination(_numNodes);
    }
  );

  // Collect tasks
  auto decomp = _plan->GetDecomposition();
  auto tasks = decomp->GetGroupMotionTasks();

  // Call CBS
  Node solution = CBS(tasks,validation,splitNode,lowLevel,cost,initial,termination);

  // Check if solution was found
  if(solution.cost == std::numeric_limits<double>::infinity()) {
    _plan->SetCost(std::numeric_limits<double>::infinity());
    return false;
  }

  ConvertToPlan(solution,_plan);
  return true;
}
/*----------------------- CBS Functors -----------------------*/

bool
ScheduledCBS::
LowLevelPlanner(Node& _node, SemanticTask* _task) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::LowLevelPlanner");

  // Initialize maps
  std::map<SemanticTask*,size_t> startTimes;
  std::map<SemanticTask*,size_t> endTimes;
  std::set<SemanticTask*> solved;

  // Collect start and end times for tasks starting at 0
  for(auto kv : _node.solutionMap) {
    auto task = kv.first;
    auto path = kv.second;
    startTimes[task] = 0;

    if(task->GetDependencies().empty() and task != _task) {
      if(path) {
        auto timesteps = path->TimeSteps();
        if(timesteps > 0) {
          endTimes[task] = timesteps-1;
        }
        else {
          endTimes[task] = 0;
        }
        solved.insert(task);
      }
    }
  }

  // Add all tasks with solutions to solved set
  auto size = solved.size();
  do {
    size = solved.size();

    // Iterate through set of tasks
    for(auto kv : _node.solutionMap) {
      auto task = kv.first;
      auto path = kv.second;

      if(solved.count(task))
        continue;

      size_t startTime = FindStartTime(task, solved, endTimes);
      if(startTime == MAX_UINT)
        continue;

      startTimes[task] = startTime;

      if(task == _task or !path)
        continue;

      solved.insert(task);

      size_t timesteps = path->TimeSteps();
      size_t offset = timesteps;// > 0 ? timesteps - 1 : 0;
      endTimes[task] = startTime + offset;
    }
  } while (solved.size() != size);

  // Initialize queue of tasks to replan
  std::priority_queue<std::pair<size_t,SemanticTask*>,
                      std::vector<std::pair<size_t,SemanticTask*>>,
                      std::greater<std::pair<double,SemanticTask*>>> pq;

  pq.push(std::make_pair(startTimes[_task],_task));

  std::set<SemanticTask*> unsolved;
  for(auto kv : _node.solutionMap) {
    auto task = kv.first;
    if(solved.count(task) or task == _task)
      continue;
    unsolved.insert(kv.first);
  }

  // Plan unsolved tasks
  while(!pq.empty()) {
    auto current = pq.top();
    pq.pop();
    SemanticTask* task = current.second;
    size_t startTime = current.first;

    if(m_debug) {
      std::cout << "Start time for "
                << task->GetLabel()
                << ": "
                << startTime
                << std::endl;
    }

    // Compute new path
    auto path = QueryPath(task,startTime,_node);

    // Check if path was found
    if(!path) {
      if(m_debug) {
        std::cout << "Failed to find a path for "
                  << task->GetLabel()
                  << std::endl;
      }
      return false;
    }

    // Save path to solution
    _node.solutionMap[task] = path;
    solved.insert(task);

    size_t timesteps = path->TimeSteps();
    size_t offset = timesteps;// > 0 ? timesteps - 1 : 0;
    endTimes[task] = startTime + offset;

    if(m_debug) {
      std::cout << "Found path for "
                << task->GetLabel()
                << " from "
                << startTime
                << " to "
                << startTime + offset
                << std::endl;
    }

    if(startTime > endTimes[task]) {
      std::cout << path->VIDs() << std::endl;
      std::cout << path->GetWaitTimes() << std::endl;
      std::cout << startTime << ", " <<  timesteps << ", " << (startTime + timesteps) << std::endl;
      throw RunTimeException(WHERE) << " BAD STUFF";
    }

    // Check if new tasks are available to plan
    std::vector<SemanticTask*> toRemove;
    for(auto t : unsolved) {

      size_t st = FindStartTime(t, solved, endTimes);
      if(st == MAX_UINT)
        continue;

      // If the task is ready, move it to the pq

      startTimes[t] = st;
      toRemove.push_back(t);
      pq.push(std::make_pair(st,t));
    }

    // Remove newly available tasks from unsolved queue
    for(auto t : toRemove) {
      unsolved.erase(t);
    }
  }

  if(solved.size() != _node.solutionMap.size())
    throw RunTimeException(WHERE) << "Did not solve all the tasks.";

  return true;
}

std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>>
ScheduledCBS::
ValidationFunction(Node& _node) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

  if(!_node.cachedNextConstraintSet.empty())
    return _node.cachedNextConstraintSet;

  auto constraintSets = FindConflicts(_node,m_bypass);

  _node.conflicts = constraintSets.size();

  if(constraintSets.empty())
    return {};

  return constraintSets[0];
}

std::vector<ScheduledCBS::Node>
ScheduledCBS::
SplitNodeFunction(Node& _node,
          std::vector<std::pair<SemanticTask*,Constraint>> _constraints,
          CBSLowLevelPlanner<SemanticTask*,Constraint,GroupPathType*>& _lowLevel,
          CBSCostFunction<SemanticTask*,Constraint,GroupPathType*>& _cost) {

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

    if(child.cost < _node.cost) {

      for(auto kv : _node.solutionMap) {
        auto st = kv.first;
        auto pp = kv.second;
        auto pc = pp->TimeSteps();
        auto cp = child.solutionMap[st];
        auto cc = cp->TimeSteps();
        std::cout << st->GetLabel() << " parent: " << pc << ", child: " << cc << std::endl;
        if(pc > cc) {
          std::cout << "CHILD COSTS LESS THAN PARENT" << std::endl;
          std::cout << "Parent" << std::endl;
          std::cout << pp->VIDs() << std::endl;
          std::cout << pp->GetWaitTimes() << std::endl;
          std::cout << "Child" << std::endl;
          std::cout << cp->VIDs() << std::endl;
          std::cout << cp->GetWaitTimes() << std::endl;
        }
      }

      throw RunTimeException(WHERE) << "Child cost less than parent.";
    }

    if(child.cost > m_upperBound)
      continue;

    if(m_bypass and child.cost == _node.cost) {

      auto newConstraints = FindConflicts(child,true);
      child.conflicts = newConstraints.size();
      if(!newConstraints.empty())
        child.cachedNextConstraintSet = newConstraints[0];

      if(child.conflicts < _node.conflicts) {
        child.constraintMap = _node.constraintMap;
        return {child};
      }

    }

    newNodes.push_back(child);
  }

  if(newNodes.empty()) {
    std::vector<SemanticTask*> tasks;
    for(auto pair : _constraints) {
      tasks.push_back(pair.first);
    }

    m_quit = HandleFailure(tasks);
  }

  return newNodes;
}

double
ScheduledCBS::
CostFunction(Node& _node) {

  double cost = 0;
  for(auto kv : _node.solutionMap) {
    auto path = kv.second;
    cost = std::max(cost,double(m_endTimes[path]));
  }

  return cost;
}

void
ScheduledCBS::
InitialSolutionFunction(std::vector<Node>& _root, std::vector<SemanticTask*> _tasks,
                        CBSLowLevelPlanner<SemanticTask*,Constraint,GroupPathType*>& _lowLevel,
                        CBSCostFunction<SemanticTask*,Constraint,GroupPathType*>& _cost) {

  Node node;

  SemanticTask* initTask = nullptr;
  for(auto task : _tasks) {
    node.solutionMap[task] = nullptr;
    node.constraintMap[task] = {};

    if(!initTask and task->GetDependencies().empty()) {
      initTask = task;
    }
  }

  // Plan tasks
  if(!_lowLevel(node,initTask))
    throw RunTimeException(WHERE) << "No initial plan.";

  // Set node cost
  node.cost = _cost(node);

  _root.push_back(node);
}

bool
ScheduledCBS::
EarlyTermination(const size_t& _numNodes) {
  
  double prob = 1 - pow(m_X, m_alpha * _numNodes / (m_quitTimes+1));
  double rand = DRand();
  if(rand < prob) {
    m_quitTimes++;

    if(this->m_debug) {
      std::cout << "Quitting CBS search with "
                << _numNodes
                << " nodes with "
                << prob << " probability."
                << std::endl;
    }

    return true;
  }
  else {
    return false;
  }
}

/*--------------------- Helper Functions ---------------------*/

ScheduledCBS::GroupPathType*
ScheduledCBS::
QueryPath(SemanticTask* _task, const size_t _startTime,
                         const Node& _node) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::QueryPath");

  // auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  // auto solution = mg->GetMPSolution();
  auto lib = this->GetMPLibrary();
  auto solution = lib->GetMPSolution();
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

  // Get last timestep of constraints
  const auto& constraints = _node.constraintMap.at(_task);
  size_t lastTimestep = 0;

  if(m_debug) {
    std::cout << "Constraints" << std::endl;
  }

  for(auto c : constraints) {
    //lastTimestep = std::max(c.second.max,lastTimestep);
    lastTimestep = std::max(c.second.min,lastTimestep);
    if(m_debug) {
      std::cout << c << std::endl;
    }
  }

  if(m_debug) {
    std::cout << "Min End Time: " << lastTimestep << std::endl;
  }

  // Compute Intervals
  ComputeIntervals(_task,_node);

  //const double timeRes = this->GetMPProblem()->GetEnvironment()->GetTimeRes();
  auto q = dynamic_cast<SIPPMethod<MPTraits<Cfg>>*>(
    this->GetMPLibrary()->GetMapEvaluator(m_queryLabel)
  );
  q->SetMinEndTime(lastTimestep);
  q->SetStartTime(_startTime);
  q->SetVertexIntervals(m_vertexIntervals);
  q->SetEdgeIntervals(m_edgeIntervals);

  // Solve the task
  lib->Solve(problem,_task->GetGroupMotionTask().get(),solution,m_queryStrategy,
             LRand(),this->GetNameAndLabel()+"::"+_task->GetLabel());

  q->SetMinEndTime(0);
  q->SetStartTime(0);
  q->SetVertexIntervals({});
  q->SetEdgeIntervals({});

  auto path = solution->GetGroupPath(group);
  if(path->VIDs().size() == 0)
    return nullptr;

  // Make path copy
  auto newPath = new GroupPathType(solution->GetGroupRoadmap(group));
  *newPath = *path;

  size_t timesteps = newPath->TimeSteps();
  size_t offset = timesteps;// > 0 ? timesteps - 1 : 0;
  m_startTimes[newPath] = _startTime;
  m_endTimes[newPath] = _startTime + offset;

  return newPath;
}

std::vector<std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>>>
ScheduledCBS::
FindConflicts(Node& _node, bool _getAll) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();

  std::vector<std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>>> constraintSets;

  auto vc = static_cast<CollisionDetectionValidityMethod<MPTraits<Cfg>>*>(
              this->GetMPLibrary()->GetValidityChecker(m_vcLabel));

  // Sort tasks based on slack
  auto slack = ComputeScheduleSlack(_node);
  std::vector<std::pair<size_t,SemanticTask*>> slackOrderedTasks;

  for(auto kv : slack) {
    auto task = m_scheduleGraph->GetVertex(kv.first);
    if(!task)
      continue;

    auto pair = std::make_pair(size_t(kv.second),task);
    slackOrderedTasks.push_back(pair);
  }

  std::sort(slackOrderedTasks.begin(),slackOrderedTasks.end(),
            [this,_node](const std::pair<size_t,SemanticTask*> _elem1,
                   const std::pair<size_t,SemanticTask*> _elem2) {

    if(_elem1.first != _elem2.first)
      return _elem1.first < _elem2.first;

    auto path1 = _node.solutionMap.at(_elem1.second);
    auto path2 = _node.solutionMap.at(_elem2.second);

    return this->m_startTimes[path1] < this->m_startTimes[path2];
  });

  // Find max timestep
  size_t maxTimestep = 0;
  for(auto kv : _node.solutionMap) {
    auto path = kv.second;
    maxTimestep = std::max(maxTimestep,m_endTimes[path]);
  }

  // Collect cfgs
  auto lib = this->GetMPLibrary();
  std::map<SemanticTask*,std::vector<GroupCfgType>> cfgPaths;
  for(auto kv : _node.solutionMap) {
    auto task = kv.first;
    auto path = kv.second;
    cfgPaths[task] = path->FullCfgsWithWait(lib);
  }

  //TODO::Make sure collision checking happens until start of next task,
  //      not just end of current path.

  //for(size_t t = 0; t <= maxTimestep; t++) {
    //for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end(); iter1++) {

  for(size_t i = 0; i < slackOrderedTasks.size(); i++) {
    auto task1 = slackOrderedTasks[i].second;
    auto slack1 = slackOrderedTasks[i].first;
    auto path1 = _node.solutionMap.at(task1);

    const size_t start1 = m_startTimes[path1];

    // Compute final timestep for path after waiting
    size_t end1 = m_endTimes[path1];
    auto vid1 = m_scheduleGraph->GetVID(task1);
    for(auto dep : m_scheduleGraph->GetPredecessors(vid1)) {
      auto depTask = m_scheduleGraph->GetVertex(dep);
      if(depTask) {
        end1 = std::min(end1,m_startTimes[_node.solutionMap.at(depTask)]);
      }
      else {
        end1 = maxTimestep;
      }
    }

    const auto& cfgs1 = cfgPaths[task1];

    std::set<SemanticTask*> collidingTasks;

    for(size_t t = start1; t < end1; t++) {
      const size_t index1 = std::min(t - start1,cfgs1.size()-1);
      const auto gcfg1 = cfgs1[index1];
      gcfg1.ConfigureRobot();
      auto group1 = gcfg1.GetGroupRoadmap()->GetGroup();

      bool collision = false;

      //auto iter2 = iter1;
      //iter2++;
      //for(; iter2 != cfgPaths.end(); iter2++) {
      for(size_t j = i+1; j < slackOrderedTasks.size(); j++) {
        auto task2 = slackOrderedTasks[j].second;

        if(collidingTasks.count(task2))
          continue;

        auto slack2 = slackOrderedTasks[j].first;
        auto path2 = _node.solutionMap.at(task2);

        const size_t start2 = m_startTimes[path2];
        if(t < start2)
          continue;

        // Compute final timestep for path after waiting
        size_t end2 = m_endTimes[path2];
        auto vid2 = m_scheduleGraph->GetVID(task2);

        // Check if this task has dependencies or is terminal
        auto depTasks = m_scheduleGraph->GetPredecessors(vid2);
        bool terminal = true;
        if(!depTasks.empty()) {
          // If it has dependencies, account for waiting time
          for(auto dep : depTasks) {
            auto depTask = m_scheduleGraph->GetVertex(dep);
            if(!depTask) 
              continue;
              
            end2 = std::min(end2,m_startTimes[_node.solutionMap.at(depTask)]);
            auto gt1 = depTask->GetGroupMotionTask();
            auto grp1 = gt1->GetRobotGroup();
            auto gt2 = task2->GetGroupMotionTask();
            auto grp2 = gt2->GetRobotGroup();
            bool overlap = false;
            for(auto r1 : grp1->GetRobots()) {
              for(auto r2 : grp2->GetRobots()) {
                if(r1 == r2) {
                  overlap = true;
                  break;
                }
              }
              if(overlap)
                break;
            }

            // If there is overlap in the robots, then task1 is not terminal
            if(overlap) {
              terminal = false;
            }
          }
        }
        
        if(terminal) {
          // Otherwise, it is terminal, and needs to be collision checked through
          // the end of path1.
          end2 = end1;
        }

        if(t > end2)
          continue;

        const auto& cfgs2 = cfgPaths[task2];
        const size_t index2 = std::min(t - start2,cfgs2.size()-1);
        const auto gcfg2 = cfgs2[index2];
        gcfg2.ConfigureRobot();
        auto group2 = gcfg2.GetGroupRoadmap()->GetGroup();

        // Check for collision
        for(auto robot1 : group1->GetRobots()) {
          for(auto robot2 : group2->GetRobots()) {

            if(robot1 == robot2)
              continue;

            auto mb1 = robot1->GetMultiBody();
            auto mb2 = robot2->GetMultiBody();

            CDInfo cdInfo;
            collision = collision or vc->IsMultiBodyCollision(cdInfo,
              mb1,mb2,this->GetNameAndLabel());

            if(collision) {
              if(m_debug) {
                std::cout << "Collision found between "
                          << robot1->GetLabel()
                          << " and "
                          << robot2->GetLabel()
                          << ", at timestep "
                          << t
                          << " and positions\n\t1: "
                          << gcfg1.GetRobotCfg(robot1).PrettyPrint()
                          << "\n\t2: "
                          << gcfg2.GetRobotCfg(robot2).PrettyPrint()
                          << " during tasks "
                          << task1->GetLabel()
                          << " and "
                          << task2->GetLabel()
                          << std::endl;

                std::cout << "Task starts: " << start1
                          << ", " << start2 << std::endl;

                std::cout << "Task1:" << std::endl
                          << path1->VIDs() << std::endl
                          << path1->GetWaitTimes() << std::endl << std::endl;

                std::cout << "Task2:" << std::endl
                          << path2->VIDs() << std::endl
                          << path2->GetWaitTimes() << std::endl << std::endl;

              }

              std::cout << "COLLISION BETWEEN SLACKS OF "
                        << slack1 << " AND " << slack2 << std::endl;

              collidingTasks.insert(task2);

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
                if(terminal) {
                  endT = SIZE_MAX;
                }
                else {
                  endT = end1;
                }
              }
              else if(group2Passive) {
                if(terminal) {
                  endT = SIZE_MAX;
                }
                else {
                  endT = end2;
                }
              }

              stats->IncStat(this->GetNameAndLabel()+"::CollisionFound");

              auto edge1 = path1->GetEdgeAtTimestep(index1);
              auto edge2 = path2->GetEdgeAtTimestep(index2);

              if(m_debug) {
                std::cout << "Edge 1: " << edge1 << std::endl;
                std::cout << "Edge 2: " << edge2 << std::endl;
              }

              size_t duration1 = 0;
              size_t duration2 = 0;

              if(edge1.first != edge1.second) {
                duration1 = path1->GetRoadmap()->GetEdge(
                  edge1.first,edge1.second).GetTimeSteps();
              }

              if(edge2.first != edge2.second) {
                duration2 = path2->GetRoadmap()->GetEdge(
                  edge2.first,edge2.second).GetTimeSteps();
              }

              size_t zero = 0;
              Range<size_t> interval1(t < duration1 ? zero : t-duration1,endT);
              Range<size_t> interval2(t < duration2 ? zero : t-duration2,endT);

              std::vector<std::pair<SemanticTask*,Constraint>> constraints;
              constraints.push_back(std::make_pair(task1,
                                    std::make_pair(edge1,interval1)));
              constraints.push_back(std::make_pair(task2,
                                    std::make_pair(edge2,interval2)));

              for(auto constraint : constraints) {
                for(auto c : _node.constraintMap[constraint.first]) {
                  if(c == constraint.second)
                    throw RunTimeException(WHERE) << "Adding constraint that already exists.";
                }
              }

              constraintSets.push_back(constraints);

              if(!_getAll)
                return constraintSets;
            }

            if(collision)
              break;
          }
          if(collision)
            break;
        }
        if(collision)
          break;
      }
    }
  }

  std::sort(constraintSets.begin(), constraintSets.end(), [slack,this](
              const std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>> _elem1,
              const std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>> _elem2) {
    for(size_t i = 0; i < _elem1.size() and i < _elem2.size(); i++) {
      if(i >= _elem2.size())
        return true;

      auto task1 = _elem1[i].first;
      auto task2 = _elem2[i].first;

      auto vid1 = this->m_scheduleGraph->GetVID(task1);
      auto vid2 = this->m_scheduleGraph->GetVID(task2);

      auto slack1 = slack.at(vid1);
      auto slack2 = slack.at(vid2);

      if(slack1 == slack2)
        continue;

      return slack1 < slack2;
    }

    return true;
  });

  return constraintSets;
}

bool
ScheduledCBS::
HandleFailure(std::vector<SemanticTask*> _tasks) {
  auto sq = dynamic_cast<SubmodeQuery*>(this->GetTaskEvaluator(m_sqLabel).get());

  // TODO::Remove assumption of two tasks
  auto task1 = _tasks[0];
  auto task2 = _tasks[1];

  sq->AddSchedulingConstraint(task1,task2);
  sq->AddSchedulingConstraint(task2,task1);

  return true;
}

void
ScheduledCBS::
ConvertToPlan(const Node& _node, Plan* _plan) {
  _plan->SetCost(_node.cost);

  if(!m_writeSolution)
    return;
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

  std::unordered_map<SemanticTask*,std::vector<GroupCfgType>> cfgPaths;

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
        endTimes[task] = startTimes[task];
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

  if(!m_workspace) {
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
      // if(m_debug) {
      //   std::cout << "\t" << i << ": " << cfg.PrettyPrint() << std::endl;
      // }
      // if(m_savePaths) {
        ofs << cfg << "\n";
      // }
    }
    ofs.close();
    // ::WritePath("hypergraph-"+kv.first->GetLabel()+".rdmp.path",kv.second);
  }

  if(!m_workspace) {
    // Naive way to create paths - will lose synchronization
    // Initialize a decomposition
    auto top = std::shared_ptr<SemanticTask>(new SemanticTask());
    Decomposition* decomp = new Decomposition(top);
    plan->SetDecomposition(decomp);

    for(auto kv : robotPaths) {
      std::cout << "Setting plan for " << kv.first->GetLabel() << std::endl;
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
      std::cout << "set plan" << std::endl;
    }
    plan->Print();
  }
}

size_t
ScheduledCBS::
FindStartTime(SemanticTask* _task, std::set<SemanticTask*> _solved,
              std::map<SemanticTask*,size_t> _endTimes) {

  size_t startTime = 0;

  // Check if all dependencies have been solved
  bool missingDep = false;
  for(auto dep : _task->GetDependencies()) {
    for(auto t : dep.second) {
      // Check if dependency has been solved
      if(!_solved.count(t)) {
        // If not, break
        missingDep = true;
        break;
      }
      // Otherwise, update start time
      startTime = std::max(startTime,_endTimes[t]);
    }

    if(missingDep)
      break;
  }

  // Check that task and all dependencies have been solved
  if(missingDep)
    return MAX_UINT;

  return startTime;
}

void
ScheduledCBS::
ComputeIntervals(SemanticTask* _task, const Node& _node) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ComputeIntervals");

  // auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  // auto solution = mg->GetMPSolution();
  auto solution = this->GetMPLibrary()->GetMPSolution();
  auto group = _task->GetGroupMotionTask()->GetRobotGroup();
  auto grm = solution->GetGroupRoadmap(group);

  m_vertexIntervals.clear();
  m_edgeIntervals.clear();

  const auto& constraints = _node.constraintMap.at(_task);

  UnsafeVertexIntervals vertexUnsafeIntervals;
  UnsafeEdgeIntervals edgeUnsafeIntervals;

  for(auto c : constraints) {
    auto edge = c.first;
    if(edge.first == edge.second) {
      vertexUnsafeIntervals[edge.first].push_back(c.second);
    }
    else {
      edgeUnsafeIntervals[edge].push_back(c.second);
    }
  }

  for(auto vit = grm->begin(); vit != grm->end(); vit++) {
    m_vertexIntervals[vit->descriptor()] = ConstructSafeIntervals(vertexUnsafeIntervals[vit->descriptor()]);

    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      m_edgeIntervals[std::make_pair(eit->source(),eit->target())] = ConstructSafeIntervals(
        edgeUnsafeIntervals[std::make_pair(eit->source(),eit->target())]);
    }
  }
}

std::vector<Range<size_t>>
ScheduledCBS::
ConstructSafeIntervals(std::vector<Range<size_t>>& _unsafeIntervals) {
  
  //const size_t m_buffer = 2;

  // Return infinite interval if there are no unsafe intervals
  if(_unsafeIntervals.empty())
    return {Range<size_t>(0,MAX_UINT)};

  // Merge unsafe intervals
  std::vector<Range<size_t>> unsafeIntervals = _unsafeIntervals;

  bool firstTime = true;
  do {

    std::set<size_t> merged;

    std::vector<Range<size_t>> copyIntervals = unsafeIntervals;
    const size_t size = copyIntervals.size();

    unsafeIntervals.clear();

    for(size_t i = 0; i < size; i++) {
      if(merged.count(i))
        continue;

      const auto& interval1 = copyIntervals[i];

      size_t zero = 0;
      size_t min = std::max(zero,std::min(interval1.min,interval1.min - (firstTime ? m_buffer : 0)));
      size_t max = std::max(interval1.max,interval1.max + (firstTime ? m_buffer : 0));


      for(size_t j = i+1; j < copyIntervals.size(); j++) {

        const auto& interval2 = copyIntervals[j];

        // Check if there is no overlap
        if(interval2.min > max + m_buffer or min > interval2.max + m_buffer)
          continue;

        
        size_t intervalMin = std::min(interval2.min,interval2.min-m_buffer);
        size_t intervalMax = std::max(interval2.max,interval2.max+m_buffer);

        // If there is, merge the intervals
        min = std::min(min,intervalMin);
        max = std::max(max,intervalMax);

        if(min > max)
          throw RunTimeException(WHERE) << "Upside down interval.";

        merged.insert(j);
      }

      Range<size_t> interval(min,max);
      unsafeIntervals.push_back(interval);
    }

    firstTime = false;

    if(copyIntervals.size() == unsafeIntervals.size())
      break;

  } while(true);

  struct less_than {
    inline bool operator()(const Range<size_t>& _r1, const Range<size_t>& _r2) {
      return _r1.min < _r2.min;
    }
  };

  std::sort(unsafeIntervals.begin(), unsafeIntervals.end(), less_than());

  for(size_t i = 1; i < unsafeIntervals.size(); i++) {
    if(unsafeIntervals[i-1].max > unsafeIntervals[i].max
      or unsafeIntervals[i-1].max > unsafeIntervals[i].min)
      throw RunTimeException(WHERE) << "THIS IS VERY BAD AND WILL RESULT IN AN INFINITE LOOP WITH NO ANSWERS.";
  }

  // Construct set of intervals
  std::vector<Range<size_t>> intervals;

  size_t min = 0;
  size_t max = std::numeric_limits<size_t>::infinity();

  auto iter = unsafeIntervals.begin();
  while(iter != unsafeIntervals.end()) {
    max = std::min(iter->min - 1,iter->min);
    if(min < max)
      intervals.emplace_back(min,max);

    min = std::max(iter->max + 1,iter->max);
    iter++;
  }

  if(min == SIZE_MAX) {
    if(m_debug) {
      for(auto inter : intervals) {
        std::cout << inter << std::endl;
      }
    }
    return intervals;
  }

  max = MAX_UINT;
  intervals.push_back(Range<size_t>(min,max));

  if(m_debug) {
    for(auto inter : intervals) {
      std::cout << inter << std::endl;
    }
  }

  return intervals;
}

/*------------------ Critical Path Functions -----------------*/

void
ScheduledCBS::
BuildScheduleGraph(Plan* _plan) {

  // Clear old schedule
  m_scheduleGraph.reset(new ScheduleGraph(_plan->GetCoordinator()->GetRobot()));

  // Add root node
  auto root = m_scheduleGraph->AddVertex(nullptr);

  // Collect tasks with dependents
  std::set<size_t> parentTasks;

  // Add all tasks to graph
  for(auto task : _plan->GetDecomposition()->GetGroupMotionTasks()) {
    auto vid = m_scheduleGraph->AddVertex(task);

    for(auto dep : task->GetDependencies()) {
      for(auto t : dep.second) {
        auto depVID = m_scheduleGraph->AddVertex(t);
        parentTasks.insert(depVID);

        m_scheduleGraph->AddEdge(vid,depVID,size_t(1));
      }
    }
  }

  // Add edges from root to tasks with no dependencies
  for(auto vit = m_scheduleGraph->begin(); vit != m_scheduleGraph->end(); vit++) {
    auto vid = vit->descriptor();
    if(parentTasks.count(vid) or vid == root)
      continue;

    m_scheduleGraph->AddEdge(root,vid,size_t(1));
  }

  // Compute atomic distances
  ComputeScheduleAtomicDistances();

  if(m_debug) {
    std::cout << "Built Schedule Graph" << std::endl;

    for(auto vit = m_scheduleGraph->begin(); vit != m_scheduleGraph->end(); vit++) {

      std::cout << vit->descriptor()
                << ":"
                << m_scheduleAtomicDistances[vit->descriptor()]
                << "\tname: "
                << (vit->property() != nullptr ? vit->property()->GetLabel() : "root")
                << std::endl
                << "\t";

      for(auto eit = vit->begin(); eit != vit->end(); eit++) {
        std::cout << eit->target() << ", ";
      }

      std::cout << std::endl;
    }
  }
}

void
ScheduledCBS::
ComputeScheduleAtomicDistances() {

  m_scheduleAtomicDistances.clear();

  SSSPPathWeightFunction<ScheduleGraph> weight = [this](
      typename ScheduleGraph::adj_edge_iterator& _ei,
      const double _sourceDistance,
      const double _targetDistance) {
    return _sourceDistance + 1.;
  };

  auto sssp = DijkstraSSSP(m_scheduleGraph.get(),{0},weight);

  for(auto kv : sssp.distance) {
    m_scheduleAtomicDistances[kv.first] = kv.second;
  }
}

std::vector<std::vector<size_t>>
ScheduledCBS::
ComputeCriticalPaths(const Node& _node) {

  auto slack = ComputeScheduleSlack(_node);

  if(m_debug) {
    std::cout << "Slack values" << std::endl;
    for(auto kv : slack) {
      auto task = m_scheduleGraph->GetVertex(kv.first);
      auto slack = kv.second;
      std::cout << (task ? task->GetLabel() : "root")
                << " : "
                << slack
                << std::endl;
    }
  }

  return {};
}

std::unordered_map<size_t,double>
ScheduledCBS::
ComputeScheduleSlack(const Node& _node) {

  SSSPPathWeightFunction<ScheduleGraph> weight = [this,_node](
      typename ScheduleGraph::adj_edge_iterator& _ei,
      const double _sourceDistance,
      const double _targetDistance) {

    auto source = this->m_scheduleGraph->GetVertex(_ei->source());
    auto target = this->m_scheduleGraph->GetVertex(_ei->target());

    // Check if source is root vertex
    if(!source)
      return 0.;

    // Otherwise compute slack between parent task (target) and child task (source)
    auto child = _node.solutionMap.at(source);
    auto parent = _node.solutionMap.at(target);

    auto start = m_startTimes[child];
    auto end = m_startTimes[parent] + parent->TimeSteps();

    auto additionalSlack = start - end;

    return double(additionalSlack) + _sourceDistance;
  };

  auto sssp = DijkstraSSSP(m_scheduleGraph.get(),{0},weight);
  return sssp.distance;
}

/*------------------------------------------------------------*/
