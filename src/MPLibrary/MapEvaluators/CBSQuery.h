#ifndef PPL_CBS_QUERY_H_
#define PPL_CBS_QUERY_H_

#include "MapEvaluatorMethod.h"
#include "SIPPMethod.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "Utilities/CBS.h"

#include "ConfigurationSpace/Cfg.h"

#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
/// Generates paths for each robot individually and then finds and resolves
/// conflicts by setting constraints on each robot's path and replanning.
///
/// Reference:
///   Guni Sharon, Roni Stern, Ariel Felner, and Nathan Sturtevant. "Conflict-
///   Based Search For Optimal Multi-Agent Path Finding". AAAI 2012.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits>
class CBSQuery : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType   RoadmapType;
    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::Path          Path;
    typedef typename RoadmapType::VID        VID;
    typedef typename RoadmapType::EdgeID     EdgeID;

    ///}

  private:

    ///@name Internal Types
    ///@{

    // Edge <Source, Target>, Time Interval <Start, End> 
    typedef std::pair<std::pair<size_t,size_t>,Range<size_t>> Constraint;
    typedef CBSNode<Robot,Constraint,Path>    Node;

    typedef std::map<size_t,std::vector<Range<size_t>>> VertexIntervals;
    typedef std::map<std::pair<size_t,size_t>,std::vector<Range<size_t>>> EdgeIntervals;

    ///@}

  public:

    ///@name Construction
    ///@{

    CBSQuery();

    CBSQuery(XMLNode& _node);

    virtual ~CBSQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluator Overrides
    ///@{

    virtual bool operator()() override;

    ///@}

  protected:

    ///@name CBS Functors
    ///@{

    /// Generate a path for a robot given a set of constraints.
    /// @param _robot The robot to solve the task for.
    /// @param _node The CBS node to store the new path in.
    /// @return Path pointer to the resulting path. A nullptr indicates failure.
    Path* SolveIndividualTask(Robot* const _robot, Node& _node);

    /// Validate that the set of paths in the node do not contain conflicts.
    /// @param _node The node to validate
    /// @return The set of new robot-constraint pairs to add to the node generated
    ///         from discovered conflicts.
    std::vector<std::pair<Robot*,Constraint>> ValidationFunction(Node& _node);

    /// Create child nodes corresponding to the additional constraints.
    /// @param _node The parent CBS node to spawn children for.
    /// @param _constraints The set of new constraints to spawn new child nodes for.
    /// @param _lowLevel The functor to plan new paths for robots with new constraints.
    /// @param _cost The functor to compute the cost of new child nodes.
    /// @return The set of new child nodes.
    std::vector<Node> SplitNodeFunction(Node& _node, 
          std::vector<std::pair<Robot*, Constraint>> _constraints,
          CBSLowLevelPlanner<Robot, Constraint, typename MPTraits::Path>& _lowlevel,
          CBSCostFunction<Robot, Constraint, typename MPTraits::Path>& _cost);

    /// Compute the cost of a CBS node. Can either be makespace or sum-of-cost.
    /// @param _node Node to compute cost for.
    /// @return The computed cost.
    double CostFunction(Node& _node);

    ///@}
    ///@name Helpers
    ///@{

    /// Compute the safe intervals for a robot given its constraint set in the node.
    /// @param _robot The robot to compute safe intervals for.
    /// @param _node The node containing the constraint set.
    void ComputeIntervals(Robot* _robot, const Node& _node);

    /// Construct the safe intervals that complement the unsafe intervals.
    /// @param _unsafeIntervals The unsafe intervals that define the complement.
    /// @return The set of safe intervals.
    std::vector<Range<size_t>> ConstructSafeIntervals(std::vector<Range<size_t>>& _unsafeIntervals);

    /// Check if two intervals are overlapping.
    /// @param _interval1 First interval to check.
    /// @param _interval2 Second interval to check.
    /// @return Boolean indicating if they are overlapping.
    bool OverlappingIntervals(Range<size_t> _interval1, Range<size_t> _interval2);

    /// Merge a pair of intervals into a new interval.
    /// @param _interval1 First interval to check.
    /// @param _interval2 Second interval to check.
    /// @return New merged interval.
    Range<double> MergeIntervals(Range<double> _interval1, Range<double> _interval2);

    ///@}
    ///@name Internal State
    ///@{

    std::vector<Robot*> m_robots; ///< The robots in the group

    std::string m_queryLabel;  ///< Query method for making individual plans.
    std::string m_vcLabel;     ///< Validity checker for conflict detection.
    std::string m_costLabel = "SOC"; ///< The label of the cost function

    size_t m_nodeLimit{std::numeric_limits<size_t>::max()}; ///< The maximum number of nodes

    std::unordered_map<Robot*, MPTask*> m_taskMap; ///< The task for each robot

    VertexIntervals m_vertexIntervals;  ///< The current set of safe vertex intervals.
    EdgeIntervals m_edgeIntervals;      ///< The current set of safe edge intervals.

    ///@}

};

template <typename MPTraits>
CBSQuery<MPTraits>::
CBSQuery() {
  this->SetName("CBSQuery");
}


template <typename MPTraits>
CBSQuery<MPTraits>::
CBSQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("CBSQuery");

  m_queryLabel = _node.Read("queryLabel", true, "",
      "The individual query method. Must be derived from QueryMethod and "
      "should not be used by any other object.");

  m_nodeLimit = _node.Read("nodeLimit", false, m_nodeLimit,
      size_t(1), std::numeric_limits<size_t>::max(),
      "Maximum number of CBS nodes to expand before declaring failure.");

  m_vcLabel = _node.Read("vcLabel", true, "",
      "The validity checker for conflict detection. Must be a CD type.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
CBSQuery<MPTraits>::
Initialize() {
  // Assert that the validity checker is an instance of collision detection
  // validity.
  auto vc = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(
      this->GetValidityChecker(m_vcLabel)
  );
  if(!vc)
    throw RunTimeException(WHERE) << "Validity checker " << m_vcLabel
                                  << " is not of type "
                                  << "CollisionDetectionValidityMethod.";

  // Assert that the query evaluator is an instance of SIPP method.
  auto query = dynamic_cast<SIPPMethod<MPTraits>*>(
      this->GetMapEvaluator(m_queryLabel)
  );
  if(!query)
    throw RunTimeException(WHERE) << "Query method " << m_queryLabel
                                  << " is not of type SIPPMethod."
                                  << std::endl;
}

template <typename MPTraits>
bool
CBSQuery<MPTraits>::
operator()() {
  auto stats = this->GetStatClass();
  const std::string clockName = this->GetNameAndLabel() + "::Query";
  MethodTimer mt(stats, clockName);

  m_robots = this->GetGroupTask()->GetRobotGroup()->GetRobots();
  for (auto& task : *(this->GetGroupTask())){
    auto robot = task.GetRobot();
    if (m_taskMap.count(robot) && m_taskMap[robot] != &task)
      throw RunTimeException(WHERE) << "This class can't handle group tasks "
                                    << "with more than one individual task "
                                    << "for a given robot (robot "
                                    << robot->GetLabel() << " has multiple "
                                    << "tasks)." << std::endl;

    m_taskMap[robot]=&task;
  }


  //CBSLowLevelPlanner<MPTask, Constraint, std::shared_ptr<typename MPTraits::Path>>(
  CBSLowLevelPlanner<Robot, Constraint, typename MPTraits::Path> lowlevel(
      [this](Node& _node, Robot* _robot) {
        auto path = this->SolveIndividualTask(_robot, _node);
        if(!path or path->VIDs().empty())
          return false;

        return true;
      });

  CBSValidationFunction<Robot, Constraint, typename MPTraits::Path> validation(
      [this](Node& _node) {
        return this->ValidationFunction(_node);
      });

  CBSSplitNodeFunction<Robot, Constraint, typename MPTraits::Path> split(
      [this](Node& _node, std::vector<std::pair<Robot*, Constraint>> _constraints,
             CBSLowLevelPlanner<Robot, Constraint, typename MPTraits::Path>& _lowlevel,
             CBSCostFunction<Robot, Constraint, typename MPTraits::Path>& _cost) {
        return this->SplitNodeFunction(_node,_constraints,_lowlevel,_cost);
      });

  CBSCostFunction<Robot, Constraint, typename MPTraits::Path> cost(
      [this](Node& _node) {
        return this->CostFunction(_node);
      });

  auto solutionNode =  CBS(m_robots, validation, split, lowlevel, cost);
  auto solution = this->GetMPSolution();

  if(solutionNode.solutionMap.empty())
    return false;

  for(auto rp : solutionNode.solutionMap) {
    solution->SetPath(rp.first, rp.second);
  }

  return true;
}

template <typename MPTraits>
typename MPTraits::Path*
CBSQuery<MPTraits>::
SolveIndividualTask(Robot* const _robot, Node& _node) {
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
  
  for(auto c : _node.constraintMap[_robot]) {
    minEndTime = std::max(minEndTime,c.second.max);
  }

  ComputeIntervals(_robot,_node);

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
  if(!success)
    return nullptr;

  // Otherwise, return a copy of the robot's path from the solution object.
  Path* path = this->GetPath(_robot);

  Path* pathCopy = new Path(path->GetRoadmap());
  *pathCopy = *path;
  _node.solutionMap[_robot] = pathCopy;

  return pathCopy;
}

template <typename MPTraits>
void
CBSQuery<MPTraits>::
ComputeIntervals(Robot* _robot, const Node& _node) {
  MethodTimer mt(this->GetStatClass(),
    this->GetNameAndLabel() + "::ComputeIntervals");

  auto solution = this->GetMPSolution();
  auto rm = solution->GetRoadmap(_robot);

  m_vertexIntervals.clear();
  m_edgeIntervals.clear();

  const auto& constraints = _node.constraintMap.at(_robot);

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
CBSQuery<MPTraits>::
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
        if(interval2.min > max + buffer or min > interval2.max + buffer)
          continue;

        // If there is, merge the intervals
        min = std::min(min,interval2.min-buffer);
        max = std::max(max,interval2.max+buffer);

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

/*---------------------------- CBS Functor Methods ---------------------------*/

template <typename MPTraits>
std::vector<std::pair<Robot*,typename CBSQuery<MPTraits>::Constraint>>
CBSQuery<MPTraits>::
ValidationFunction(Node& _node) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::ValidationFunction");

  auto vc = static_cast<CollisionDetectionValidityMethod<MPTraits>*>(
              this->GetMPLibrary()->GetValidityChecker(m_vcLabel));

  // Find max timestep
  size_t maxTimestep = 0;
  for(auto kv : _node.solutionMap) {
    auto path = kv.second;
    maxTimestep = std::max(maxTimestep,path->TimeSteps());
  }

  // Collect cfgs
  auto lib = this->GetMPLibrary();
  std::map<Robot*,std::vector<Cfg>> cfgPaths;
  for(auto kv : _node.solutionMap) {
    auto robot = kv.first;
    auto path = kv.second;
    cfgPaths[robot] = path->FullCfgsWithWait(lib);
  }

  for(size_t t = 0; t <= maxTimestep; t++) {
    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end(); iter1++) {

      auto robot1 = iter1->first;
      auto path1 = _node.solutionMap[robot1];
      
      const auto& cfgs1 = iter1->second;
      const size_t index1 = std::min(t,cfgs1.size()-1);
      const auto cfg1 = cfgs1[index1];
      cfg1.ConfigureRobot();
      auto mb1 = robot1->GetMultiBody();

      auto iter2 = iter1;
      iter2++;
      for(; iter2 != cfgPaths.end(); iter2++) {
        auto robot2 = iter2->first;
        auto path2 = _node.solutionMap[robot2];
        
        const auto& cfgs2 = iter2->second;
        const size_t index2 = std::min(t,cfgs2.size()-1);
        const auto cfg2 = cfgs2[index2];
        cfg2.ConfigureRobot();

        // Check for collision
        bool collision = false;

        auto mb2 = robot2->GetMultiBody();

        CDInfo cdInfo;
        collision = collision or vc->IsMultiBodyCollision(cdInfo,
                        mb1,mb2,this->GetNameAndLabel());

        if(collision) {
          if(this->m_debug) {
            std::cout << "Collision found between "
                      << robot1->GetLabel()
                      << " and "
                      << robot2->GetLabel()
                      << ", at timestep "
                      << t
                      << " and positions\n\t1: "
                      << cfg1.PrettyPrint()
                      << "\n\t2: "
                      << cfg2.PrettyPrint()
                      << "."
                      << std::endl;

            std::cout << "Task1:" << std::endl
                      << path1->VIDs() << std::endl
                      << path1->GetWaitTimes() << std::endl << std::endl;

            std::cout << "Task2:" << std::endl
                      << path2->VIDs() << std::endl
                      << path2->GetWaitTimes() << std::endl << std::endl;
          }

          auto endT = t;

          stats->IncStat(this->GetNameAndLabel()+"::CollisionFound");

          auto edge1 = path1->GetEdgeAtTimestep(t).first;
          auto edge2 = path2->GetEdgeAtTimestep(t).first;

          if(this->m_debug) {
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

          std::vector<std::pair<Robot*,Constraint>> constraints;
          constraints.push_back(std::make_pair(robot1,
                                std::make_pair(edge1,interval1)));
          constraints.push_back(std::make_pair(robot2,
                                std::make_pair(edge2,interval2)));

          for(auto constraint : constraints) {
            for(auto c : _node.constraintMap[constraint.first]) {
              if(c == constraint.second) {
                throw RunTimeException(WHERE) << "Adding constraint that already exists.";
                // std::cout << "Adding constraint that already exists." << std::endl;
              }
            }
          }

          return constraints;
        }
      }
    }
  }

  return {};
}

template <typename MPTraits>
std::vector<typename CBSQuery<MPTraits>::Node>
CBSQuery<MPTraits>::
SplitNodeFunction(Node& _node, std::vector<std::pair<Robot*, Constraint>> _constraints,
                  CBSLowLevelPlanner<Robot, Constraint, typename MPTraits::Path>& _lowlevel,
                  CBSCostFunction<Robot, Constraint, typename MPTraits::Path>& _cost) {
  MethodTimer mt(this->GetStatClass(),
    this->GetNameAndLabel() + "::SplitNodes");

  std::vector<Node> children;


  for(auto constraintPair : _constraints) {
    auto robot = constraintPair.first;
    auto constraint = constraintPair.second;
    Node child = _node;

    child.constraintMap[robot].insert(constraint);

    if(!_lowlevel(child, robot))
      continue;

    child.cost = _cost(child);
    children.push_back(child);

    if(this->m_debug)
      std::cout << "\t\t\tChild node created." << std::endl;
  }

  return children;
}

template <typename MPTraits>
double
CBSQuery<MPTraits>::
CostFunction(Node& _node) {

  size_t cost = 0;

  if(this->m_costLabel == "SOC") {
    for(const auto& ts : _node.solutionMap) {
      cost += ts.second->TimeSteps();
    }
  }
  else {
    for(const auto& ts : _node.solutionMap) {
      cost = std::max(cost,ts.second->TimeSteps());
    }
  }

  return double(cost);
}

/*---------------------------- Helper Functions ---------------------*/

template <typename MPTraits>
bool
CBSQuery<MPTraits>::
OverlappingIntervals(Range<size_t> _interval1, Range<size_t> _interval2) {
  if(_interval1.max <= _interval2.min)
    return false;

  if(_interval1.min >= _interval2.max)
    return false;

  return true;
}

template <typename MPTraits>
Range<double>
CBSQuery<MPTraits>::
MergeIntervals(Range<double> _interval1, Range<double> _interval2) {
 return Range<double>(std::max(_interval1.min,_interval2.min),
          std::min(_interval1.max, _interval2.max));
}

#endif
