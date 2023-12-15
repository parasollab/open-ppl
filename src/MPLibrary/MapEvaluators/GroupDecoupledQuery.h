#ifndef PMPL_GROUP_DECOUPLED_QUERY_H_
#define PMPL_GROUP_DECOUPLED_QUERY_H_

#include "MapEvaluatorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Calls an individual query for each robot in the group to realize cooperative
/// A*. After each plan is extracted, that robot is treated as a dynamic
/// obstacle for the remaining robots.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupDecoupledQuery : public MapEvaluatorMethod<MPTraits> {



  public:

    ///@name Motion Planning Types
    ///@{

    //typedef typename MPTraits::RoadmapType   RoadmapType;
    //typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::Path          Path;
    //typedef typename RoadmapType::VID        VID;
    //typedef typename RoadmapType::EdgeID     EdgeID;

    ///}
  private:
    ///@name Internal Types
    ///@{

    // Edge <Source, Target>, Time Interval <Start, End>
    typedef std::pair<std::pair<size_t,size_t>,Range<size_t>> Constraint;
    //typedef CBSNode<Robot,Constraint,Path>    Node;

    typedef std::map<size_t,std::vector<Range<size_t>>> VertexIntervals;
    typedef std::map<std::pair<size_t,size_t>,std::vector<Range<size_t>>> EdgeIntervals;

    ///@}

  public:

    ///@name Construction
    ///@{

    GroupDecoupledQuery();

    GroupDecoupledQuery(XMLNode& _node);

    virtual ~GroupDecoupledQuery() = default;

    ///@}
    ///@name MapEvaluator Overrides
    ///@{

    virtual bool operator()() override;

    ///@}

  protected:

    /// Generate a path for a robot given a set of constraints
    Path* SolveIndividualTask(Robot* const _robot);
    bool ValidationFunction(Robot* _robot);

    ///@name Internal State
    ///@{

    std::string m_queryLabel;  ///< Label for an individual query method.
    std::string m_vcLabel;  ///< Label for validty checker method.

    bool m_ignoreOtherRobots{false};

    std::unordered_map<Robot*, MPTask*> m_taskMap; ///< The task for each robot
    /// Map of task to its constraints
    //std::unordered_map<IndividualTask*,std::unordered_set<ConstraintType>> constraintMap;
    std::map<Robot*,std::set<Constraint>> m_constraintMap;

    std::vector<std::map<Robot*,VertexIntervals>> m_unsafeVertexIntervals;
    std::vector<std::map<Robot*,EdgeIntervals>> m_unsafeEdgeIntervals;

    VertexIntervals m_vertexIntervals;
    EdgeIntervals m_edgeIntervals;

    // increment by m_buffer when checking for collisions instead of
    // just 1
    size_t m_buffer{1};

    // whether or not this is to be used as local strategy
    bool m_local{false};

    // if decoupled is to be used as local strategy we need to set
    // a maxinum number of iterations before quitting and expanding
    size_t m_maxIter{size_t(MAX_INT)};

    // Whether or not to shuffle the priority order for each query
    bool m_shuffleOrder{true};

    ///@}
    ///@name Helpers
    ///@{

    void ComputeIntervals(Robot* _robot);

    std::vector<Range<size_t>> ConstructSafeIntervals(std::vector<Range<size_t>>& _unsafeIntervals);

    bool OverlappingIntervals(Range<size_t> _existingInterval, Range<size_t> _newInterval);

    Range<double> MergeIntervals(Range<double> _interval1, Range<double> _interval2);

    virtual std::string GetQueryMethod() override;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GroupDecoupledQuery<MPTraits>::
GroupDecoupledQuery() {
  this->SetName("GroupDecoupledQuery");
}


template <typename MPTraits>
GroupDecoupledQuery<MPTraits>::
GroupDecoupledQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("GroupDecoupledQuery");

  m_queryLabel = _node.Read("queryLabel", true, "",
      "The individual query method.");

  m_ignoreOtherRobots = _node.Read("ignoreOtherRobots", false, m_ignoreOtherRobots,
      "Flag to ignore collisions with other robots.");

  m_vcLabel = _node.Read("vcLabel", true, "",
      "The validity checker for conflict detection. Must be a CD type.");

  m_buffer = _node.Read("buffer",false, m_buffer,size_t(0), size_t(MAX_INT),
      "Buffer to fit around time intervals.");  

  m_local = _node.Read("ifLocal", false, false, 
      "Whether or not this is to be used as local strategy.");

  m_maxIter = _node.Read("maxIter", false, m_maxIter, size_t(0), size_t(MAX_INT),
      "Maximum number of iterations before quitting when used as local strategy");        

  m_shuffleOrder = _node.Read("shuffleOrder",false,m_shuffleOrder,
      "Flag to shuffle the priority order for each query.");

  // m_earlyTermination = _node.Read("")    
}

/*-------------------------- MapEvaluator Overrides --------------------------*/

template <typename MPTraits>
bool
GroupDecoupledQuery<MPTraits>::
operator()() {
  // For each individual task t in group task:
  // - Set individual task
  // - Run query
  // - create dynamic obstacle from path
  // Clear individual task
  // Clear dynamic obstacles
  auto groupTask = this->GetGroupTask();
  auto group = groupTask->GetRobotGroup();
  if(this->m_debug)
    std::cout << "Running decoupled query for robot group '"
              << group->GetLabel() << "'."
              << std::endl;

  // Resetting constraint map
  m_constraintMap.clear();

  bool success = true;
  //auto query = this->GetMapEvaluator(m_queryLabel);

  // Unset the group task.
  this->GetMPLibrary()->SetGroupTask(nullptr);

  // Shuffle the order in which tasks are planned

  size_t index = 0;
  std::set<Robot*> solved;
  while(solved.size() < group->Size()) {
  //for(auto& task : *groupTask) {
    //auto robot = task.GetRobot();

    if(m_shuffleOrder) {
      index = LRand() % group->Size();
    }
    else {
      index++;
    }
    auto iter = groupTask->begin();
    iter += index;

    auto robot = iter->GetRobot();
    if(solved.count(robot))
      continue;
    solved.insert(robot);
    auto& task = *iter;


    m_taskMap[robot]=&task;
    if(this->m_debug)
      std::cout << "\tQuerying path for robot '" << robot->GetLabel()
        << "', task '" << task.GetLabel() << "'."
        << std::endl;

    // Evaluate this task.
    this->GetMPLibrary()->SetTask(&task);
    //bool moreThanOneIter = false;
    size_t count = 0;
    /*
       while(true) {
       if(count == m_maxIter) {
       success = false;
       break;
       }        
       if(ValidationFunction(robot) and moreThanOneIter)
       break;   
       auto path = this->SolveIndividualTask(robot);
       if(!path or path->VIDs().empty())
       success = false;
       moreThanOneIter = true;
       count++;
       }
     */

    do {
      if(count == m_maxIter) {
        success = false;
        break;
      }

      auto path = this->SolveIndividualTask(robot);
      count++;

      if(!path or path->VIDs().empty()) {
        success = false;
        break;
      }

      if(ValidationFunction(robot)) {
        break;
      }

    } while(true);

    //ValidationFunction(robot);
    //auto path = this->SolveIndividualTask(robot);
    //if(!path or path->VIDs().empty())
    //     success = false;
    //success &= (*query)();
    if(!success)
      break;

    // Success: add this robot/path as a dynamic obstacle for the remaining
    // robots.

    if(m_ignoreOtherRobots)
      continue;

    this->GetMPProblem()->AddDynamicObstacle(
        DynamicObstacle(robot, this->GetPath(robot)->FullCfgsWithWait(this->GetMPLibrary()))
        );

  }

  if(success)
    this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::FoundPath", 1);

  if(this->m_debug)
    std::cout << "\tDone." << std::endl;


  // Restore the group task.
  this->GetMPLibrary()->SetTask(nullptr);
  this->GetMPLibrary()->SetGroupTask(groupTask);
  this->GetMPProblem()->ClearDynamicObstacles();

  return success;
}


template <typename MPTraits>
typename MPTraits::Path*
GroupDecoupledQuery<MPTraits>::
SolveIndividualTask(Robot* const _robot) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "SolveIndividualTask");

  MPTask* const task = m_taskMap.at(_robot);

  if(this->m_debug) {
    std::cout << "Solving task '" << task->GetLabel() << "' (" << task
              << ") for robot '" << _robot->GetLabel() << "' (" << _robot
              << ")."
              << std::endl;
  }

  size_t minEndTime = 0;

  for(auto c : m_constraintMap[_robot]) {
    minEndTime = std::max(minEndTime,c.second.max);
  }

  ComputeIntervals(_robot);

  // Generate a path for this robot individually while avoiding the conflicts.
  auto groupTask = this->GetGroupTask();
  this->GetMPLibrary()->SetGroupTask(nullptr);
  this->GetMPLibrary()->SetTask(task);
  auto query = dynamic_cast<SIPPMethod<MPTraits>*>(this->GetMapEvaluator(m_queryLabel));
  query->SetEdgeIntervals(m_edgeIntervals);
  query->SetVertexIntervals(m_vertexIntervals);
  query->SetMinEndTime(minEndTime);
  const bool success = (*query)();
  this->GetMPLibrary()->SetTask(nullptr);
  this->GetMPLibrary()->SetGroupTask(groupTask);

  if(this->m_debug)
    std::cout << "Path for robot " << _robot->GetLabel() << " was "
              << (success ? "" : "not ") << "found."
              << std::endl;

  // If we failed to find a path, return an empty pointer.
  if(!success) {

    // // if we are using shared roadmap
    // if(_robot->GetRepresentative())
    //   this->GetGoalTracker()->Clear();
    return nullptr;
  }
    

  // Otherwise, return a copy of the robot's path from the solution object.
  Path* path = this->GetPath(_robot);

  Path* pathCopy = new Path(path->GetRoadmap());
  *pathCopy = *path;

  return pathCopy;
}

template <typename MPTraits>
void
GroupDecoupledQuery<MPTraits>::
ComputeIntervals(Robot* _robot) {
  MethodTimer mt(this->GetStatClass(),
    this->GetNameAndLabel() + "::ComputeIntervals");

  auto solution = this->GetMPSolution();
  auto rm = solution->GetRoadmap(_robot);

  m_vertexIntervals.clear();
  m_edgeIntervals.clear();

  const auto& constraints = m_constraintMap.at(_robot);

  VertexIntervals vertexUnsafeIntervals;
  EdgeIntervals edgeUnsafeIntervals;

  for(auto c : constraints) {
    auto edge = c.first;
    if(edge.first == edge.second) {
      vertexUnsafeIntervals[edge.first].push_back(c.second);
    }
    else {
      edgeUnsafeIntervals[edge].push_back(c.second);
    }
  }

  for(auto vit = rm->begin(); vit != rm->end(); vit++) {
    m_vertexIntervals[vit->descriptor()] = ConstructSafeIntervals(vertexUnsafeIntervals[vit->descriptor()]);

    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      m_edgeIntervals[std::make_pair(eit->source(),eit->target())] = ConstructSafeIntervals(
        edgeUnsafeIntervals[std::make_pair(eit->source(),eit->target())]);
    }
  }
}

template <typename MPTraits>
std::vector<Range<size_t>>
GroupDecoupledQuery<MPTraits>::
ConstructSafeIntervals(std::vector<Range<size_t>>& _unsafeIntervals) {
  const size_t buffer = 2;

  // Return infinite interval if there are no unsafe intervals
  if(_unsafeIntervals.empty())
    return {Range<size_t>(0,MAX_INT)};

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
      size_t min = std::max(zero,std::min(interval1.min,interval1.min - (firstTime ? buffer : 0)));
      size_t max = std::max(interval1.max,interval1.max + (firstTime ? buffer : 0));

      for(size_t j = i+1; j < copyIntervals.size(); j++) {

        const auto& interval2 = copyIntervals[j];

        // Check if there is no overlap

        auto min1 = min;
        auto max1 = std::max(max,max+buffer);
        auto min2 = interval2.min;
        auto max2 = std::max(interval2.max + buffer,interval2.max);

        //if(interval2.min > max + buffer or min > interval2.max + buffer)
        if(min2 > max1 or min1 > max2)
          continue;

        // If there is, merge the intervals
        //min = std::min(min,interval2.min-buffer);
        //max = std::max(max,interval2.max+buffer);
        min = std::min(min,std::min(interval2.min-buffer,interval2.min));
        max = std::max(max,max2);

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

  max = MAX_INT;
  intervals.push_back(Range<size_t>(min,max));

  return intervals;
}

/*---------------------------- Helper Functions ---------------------*/

template <typename MPTraits>
bool
GroupDecoupledQuery<MPTraits>::
OverlappingIntervals(Range<size_t> _existingInterval, Range<size_t> _newInterval) {
  if(_existingInterval.max <= _newInterval.min)
    return false;

  if(_existingInterval.min >= _newInterval.max)
    return false;

  return true;
}

template <typename MPTraits>
Range<double>
GroupDecoupledQuery<MPTraits>::
MergeIntervals(Range<double> _interval1, Range<double> _interval2) {
 return Range<double>(std::max(_interval1.min,_interval2.min),
          std::min(_interval1.max, _interval2.max));
}

template <typename MPTraits>
bool
GroupDecoupledQuery<MPTraits>::
ValidationFunction(Robot* _robot) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::ValidationFunction");

  auto vc = static_cast<CollisionDetectionValidityMethod<MPTraits>*>(
              this->GetMPLibrary()->GetValidityChecker(m_vcLabel));

  bool conflictFree = true;

  // First we collect the cfgs of the robot's path
  auto lib = this->GetMPLibrary();
  auto robotPath = this->GetPath(_robot);
  auto cfgRobotPath = robotPath->FullCfgsWithWait(lib);

  // Check this configuration against each dynamic obstacle.
  const auto& obstacles = this->GetMPProblem()->GetDynamicObstacles();
  // We quit if no dynamic obstacles
  if(obstacles.empty())
    return true;
  // Find max timestep
  size_t maxTimestep = 0;
  for(const auto& obstacle : obstacles) {
    const auto& obstaclePath = obstacle.GetPath();
    maxTimestep = std::max(maxTimestep,obstaclePath.size()-1);      
  }
  // caveat: when a decoupled/hybrid strategy is used as local strategy we
  // actually want the minimum timestep   
  if(m_local) {
    maxTimestep = std::min(maxTimestep,cfgRobotPath.size() - 1);
  }
  else {
    maxTimestep = std::max(maxTimestep,cfgRobotPath.size() - 1);
  }
  // Collect obstacles' paths cfgs.
  std::map<Robot*,std::vector<Cfg>> cfgObstaclePaths;
  for(const auto& obstacle : obstacles) {
    const auto& obstaclePath = obstacle.GetPath();
    //auto cfgObstaclePath = obstaclePath->FullCfgsWithWait(lib);
    cfgObstaclePaths[obstacle.GetRobot()] = obstaclePath;
  }




  for(size_t t = 0; t <= maxTimestep; t++) {
    for(auto iter = cfgObstaclePaths.begin(); iter != cfgObstaclePaths.end(); iter++) {

      
      auto obstacleRobot = iter->first;
      //auto obstacleCfgPath = iter->second;
      const auto& obstacleCfgPath = iter->second;

      // If local, consider this obstacle to have 'disappeared' at the end of its path
      if(m_local and obstacleCfgPath.size() <= t)
        continue;

      const size_t index1 = std::min(t,obstacleCfgPath.size()-1);
      const auto cfg1 = obstacleCfgPath[index1];
      cfg1.ConfigureRobot();
      auto omb = obstacleRobot->GetMultiBody();

      // robot is _robot, its path is cfgRobotPath
      const size_t index2 = std::min(t,cfgRobotPath.size()-1);
      const auto cfg2 = cfgRobotPath[index2];
      cfg2.ConfigureRobot();
      auto mb = _robot->GetMultiBody();

      // Check for collision
      bool collision = false;


      CDInfo cdInfo;
      collision = collision or vc->IsMultiBodyCollision(cdInfo,
                      omb,mb,this->GetNameAndLabel());

      if(collision) {
        conflictFree = false;
        if(this->m_debug) {
          std::cout << "Collision found between dynamic obstacle "
                    << obstacleRobot->GetLabel()
                    << " and robot "
                    << _robot->GetLabel()
                    << ", at timestep "
                    << t
                    << " and positions\n\t1: "
                    << cfg1.PrettyPrint()
                    << "\n\t2: "
                    << cfg2.PrettyPrint()
                    << "."
                    << std::endl;

          std::cout << "Task:" << std::endl
                    << robotPath->VIDs() << std::endl
                    << robotPath->GetWaitTimes() << std::endl << std::endl;
        }

        auto endT = t;

        stats->IncStat(this->GetNameAndLabel()+"::CollisionFound");

        auto edge = robotPath->GetEdgeAtTimestep(t).first;
        //auto edge2 = path2->GetEdgeAtTimestep(t).first;

        if(this->m_debug) {
          std::cout << "Edge: " << edge << std::endl;
          //std::cout << "Edge 2: " << edge2 << std::endl;
        }

        size_t duration = 0;
        //size_t duration2 = 0;

        if(edge.first != edge.second) {
          duration = robotPath->GetRoadmap()->GetEdge(
          edge.first,edge.second).GetTimeSteps();
        }

        //if(edge2.first != edge2.second) {
        //  duration2 = path2->GetRoadmap()->GetEdge(
        //  edge2.first,edge2.second).GetTimeSteps();
        //}

        size_t zero = 0;
        Range<size_t> interval(t < duration ? zero : t-duration,
                               t > index1 ? SIZE_MAX : endT+m_buffer);
        //Range<size_t> interval2(t < duration2 ? zero : t-duration2,endT);

        //std::vector<std::pair<Robot*,Constraint>> constraints;
        auto iter = m_constraintMap.find(_robot);
        if(iter == m_constraintMap.end()) {
          std::set<Constraint> constraintSet;
          constraintSet.insert(std::make_pair(edge,interval));
          m_constraintMap[_robot] = constraintSet;
        } else {
          iter->second.insert(std::make_pair(edge,interval));
        }
        //constraints.push_back(std::make_pair(_robot,
        //                      std::make_pair(edge,interval)));
        //constraints.push_back(std::make_pair(robot2,
        //                      std::make_pair(edge2,interval2)));

        //for(auto constraint : constraints) {
        //  for(auto c : m_constraintMap[constraint.first]) {
        //    if(c == constraint.second) {
        //      throw RunTimeException(WHERE) << "Adding constraint that already exists.";
        //      // std::cout << "Adding constraint that already exists." << std::endl;
        //    }
        //  }
        //}

        //return constraints;
        return conflictFree;
      }
    }
  }
  return conflictFree;
  //return {};
}


template <typename MPTraits>
std::string
GroupDecoupledQuery<MPTraits>::
GetQueryMethod() {
  return m_queryLabel;
}

/*----------------------------------------------------------------------------*/

#endif
