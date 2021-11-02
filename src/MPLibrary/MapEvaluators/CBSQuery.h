#ifndef PMPL_CBS_QUERY_H_
#define PMPL_CBS_QUERY_H_

#include "MapEvaluatorMethod.h"
#include "QueryMethod.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "Utilities/CBS.h"

#include "ConfigurationSpace/Cfg.h"

#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>

template<typename MPTraits>
class CBSQuery : public MapEvaluatorMethod<MPTraits> {

  public:

    typedef typename MPTraits::RoadmapType   RoadmapType;
    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::Path          Path;
    typedef typename RoadmapType::VID        VID;
    typedef typename RoadmapType::EdgeID     EdgeID;


  private:

    struct Conflict {
      CfgType cfg1;     ///< The first robot's configuration.
      CfgType cfg2;     ///< The second robot's configuration.
      size_t  timestep; ///< The timestep when the collision occurred.

      /// @todo This is to support old gcc v4.x, replace with
      ///       default-constructed class members after we upgrade.
      Conflict(const CfgType& _cfg1 = CfgType(nullptr),
          const CfgType& _cfg2 = CfgType(nullptr),
          const size_t _timestep = 0) :
          cfg1(_cfg1),
          cfg2(_cfg2),
          timestep(_timestep)
      {}

      /// @return True if this is an empty conflict.
      bool Empty() const noexcept {
        return !cfg1.GetRobot();
      }
    };

    typedef std::pair<size_t, CfgType> Constraint;

    typedef std::set<Constraint> ConstraintSet;

    typedef std::map<Robot*, ConstraintSet> ConstraintMap;

    typedef std::unordered_map<Robot*, Path*> SolutionMap;

    typedef CBSNode<Robot, Constraint, Path> CBSNodeType;

    typedef std::unordered_map<size_t,
                std::unordered_map<size_t,
                    std::vector<Range<double>>>> EdgeIntervals;

    typedef std::unordered_map<Robot*,
                std::unordered_map<size_t,
                    EdgeIntervals>> EdgeIntervalsMap;

    typedef std::multimap<size_t,
                std::pair<CfgType,size_t>> SingleConflictsCache;

    typedef std::unordered_map<Robot*,SingleConflictsCache> GroupConflictsCache;

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

    ///@name Helpers
    ///@{

    Path* SolveIndividualTask(Robot* const _robot,
        const ConstraintMap& _constraintMap = {});

    std::pair<std::pair<Robot*, Robot*>, Conflict> FindConflict(const SolutionMap& _solution);

    double MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceTimestep, const double _bestTimestep) const;

    bool IsEdgeSafe(const VID _source, const VID _target,
        const CfgType& _conflictCfg) const;

    EdgeIntervals ComputeIntervals(Robot* _robot);

    EdgeIntervals JoinEdgeIntervals(Robot* _robot, std::vector<EdgeIntervals> _edgeIntervals);

    std::vector<Range<double>> JoinIntervals(std::vector<std::vector<Range<double>>> _allIntervals);

    std::vector<Range<double>> InsertIntervals(std::vector<Range<double>> _jointIntervals, std::vector<Range<double>> _newIntervals);

    bool OverlapingIntervals(Range<double> _existingInterval, Range<double> _newInterval);

    Range<double> MergeIntervals(Range<double> _interval1, Range<double> _interval2);

    std::set<std::pair<size_t, Cfg>>::iterator LowerBound(size_t bound) const;
    std::set<std::pair<size_t, Cfg>>::iterator UpperBound(size_t bound) const;

    std::vector<Robot*> m_robots;

    std::string m_queryLabel;  ///< Query method for making individual plans.
    std::string m_vcLabel;     ///< Validity checker for conflict detection.
    std::string m_safeIntervalLabel; // The Safe Intarval Tool label
    std::string m_costLabel = "SOC";
    size_t m_nodeLimit{std::numeric_limits<size_t>::max()};

    const ConstraintSet* m_currentConstraints{nullptr};
    std::set<ConstraintMap> m_constraintCache;
    std::unordered_map<Robot*, MPTask*> m_taskMap;

    GroupConflictsCache m_groupConflictsCache;

    EdgeIntervalsMap m_edgeIntervalsMap;

    size_t m_cacheIndex{0};

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

  m_safeIntervalLabel = _node.Read("safeIntervalToolLabel", true, "",
      "The Safe Interval Tool Label");

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
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(
      this->GetValidityChecker(m_vcLabel)
  );
  if(!vc)
    throw RunTimeException(WHERE) << "Validity checker " << m_vcLabel
                                  << " is not of type "
                                  << "CollisionDetectionValidity.";

  // Assert that the query evaluator is an instance of query method.
  auto query = dynamic_cast<QueryMethod<MPTraits>*>(
      this->GetMapEvaluator(m_queryLabel)
  );
  if(!query)
    throw RunTimeException(WHERE) << "Query method " << m_queryLabel
                                  << " is not of type QueryMethod."
                                  << std::endl;
}

template <typename MPTraits>
bool
CBSQuery<MPTraits>::
operator()() {

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
      [this](CBSNodeType& _node, Robot* _robot) {
        auto path = this->SolveIndividualTask(_robot, _node.constraintMap);
        if(!path or path->Cfgs().empty())
          return false;

        Path* pathCopy = new Path(path->GetRoadmap());
        *pathCopy = *path;
        _node.solutionMap[_robot] = pathCopy;
        return true;
      });

  CBSValidationFunction<Robot, Constraint, typename MPTraits::Path> validation(
      [this](CBSNodeType& _node) {
        auto robotConflict = this->FindConflict(_node.solutionMap);
        std::vector<std::pair<Robot*, Constraint>> constraints;
        if (robotConflict.first.first != nullptr) {
          auto c = robotConflict.second;
          constraints = {std::make_pair(robotConflict.first.first, std::make_pair(c.timestep, c.cfg2)), std::make_pair(robotConflict.first.second, std::make_pair(c.timestep, c.cfg1))};
        }
        return constraints;
      });

  CBSSplitNodeFunction<Robot, Constraint, typename MPTraits::Path> split(
        [this](CBSNodeType& _node, std::vector<std::pair<Robot*, Constraint>> _constraints,
          CBSLowLevelPlanner<Robot, Constraint, typename MPTraits::Path>& _lowlevel,
          CBSCostFunction<Robot, Constraint, typename MPTraits::Path>& _cost) {

            std::vector<CBSNodeType> children;
            for (auto constraintPair : _constraints) {
              auto robot = constraintPair.first;
              auto constraint = constraintPair.second;
              CBSNodeType child = _node;
              if(child.constraintMap[robot].find(constraint) != child.constraintMap[robot].end()){
                if(this->m_debug)
                  std::cout << "\t\t\tConflict double-assigned to robot "
                            << robot->GetLabel() << ", skipping."
                            << std::endl;
                continue;
              }
              child.constraintMap[robot].insert(constraint);

              if(m_constraintCache.count(child.constraintMap)) {
                if (this->m_debug)
                  std::cout << "\t\t\tThis conflict set was already attempted, skipping."
                            << std::endl;
                continue;
              }
              m_constraintCache.insert(child.constraintMap);
              if (!_lowlevel(child, robot))
                continue;

              child.cost = _cost(child);
              children.push_back(child);

              if(this->m_debug)
                std::cout << "\t\t\tChild node created." << std::endl;
            }

            return children;
      });
  CBSCostFunction<Robot, Constraint, typename MPTraits::Path> cost(
      [this](CBSNodeType& _node) {
        double cost = 0;
        if (this->m_costLabel == "SOC") {
          for (const auto& ts : _node.solutionMap) {
            cost += ts.second->Length();
          }
        } else {
          for (const auto& ts : _node.solutionMap) {
            if (ts.second->Length() > cost)
              cost = ts.second->Length();
          }
        }
        return cost;
      });

  auto solution_node =  CBS(m_robots, validation, split, lowlevel, cost);
  auto solution = this->GetMPSolution();
  if (solution_node.solutionMap.size() > 0) {
    for(auto rp : solution_node.solutionMap) {
      solution->SetPath(rp.first, rp.second);
    }
    return true;
  }
  return false;
}

template <typename MPTraits>
typename MPTraits::Path*
CBSQuery<MPTraits>::
SolveIndividualTask(Robot* const _robot, const ConstraintMap& _constraintMap) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "SolveIndividualTask");

  MPTask* const task = m_taskMap.at(_robot);

  if(this->m_debug)
    std::cout << (_constraintMap.empty() ? "\t" : "\t\t\t")
              << "Solving task '" << task->GetLabel() << "' (" << task
              << ") for robot '" << _robot->GetLabel() << "' (" << _robot
              << ")."
              << std::endl;

  // Set the conflicts to avoid.
  if (!_constraintMap.empty())
    m_currentConstraints = &_constraintMap.at(_robot);

  // New stuff starts here:
  this->GetMPProblem()->ClearDynamicObstacles();

  std::vector<EdgeIntervals> edgeIntervalsSet;

  EdgeIntervals jointEdgeIntervals;


  if(m_currentConstraints) {
    for(auto c : *m_currentConstraints) {
      bool conflictCached = false;
      // First, we check if _robot has a SafeInterval's that have been cached before (at least once).
      if(m_groupConflictsCache.count(_robot)){
        auto it = m_groupConflictsCache.find(_robot);
        // If so, we check if the current constraint has been cached (we first
        // check the timestep).
        std::cout << "\t\t\tm_currentConstraints[_robot] has size  set has a size of " << m_currentConstraints->size() << std::endl;
        std::cout << "\t\t\tm_groupConflictsCache[_robot] has size  set has a size of " << it->second.size() << std::endl;
        if(it->second.count(c.first)){
          //auto it2 = it->second.find(c.first);
          auto eq = it->second.equal_range(c.first);
          // Now we iterate through all the constraints that have the same
          // timestep, when we match the same constraint cfg, we can now get the
          // safe intervals.
          for(auto it2 = eq.first; it2 != eq.second ; ++it2) {
            if(it2->second.first == c.second){
              std::cout << "cfg: " << it2->second.first.PrettyPrint() << ", index: " << it2->second.second
                << ", timestep: " << c.first << std::endl;
              auto index = it2->second.second;
              auto edgeIntervals = m_edgeIntervalsMap[_robot][index];
              edgeIntervalsSet.push_back(edgeIntervals);
              conflictCached = true;
              //break;
            }
          }
        }
      }
      // If we got here it means that the current constraint has not been
      // cached, then we have to compute its safe intervals by turning it into a
      // dynamic obstacle.
      if(!conflictCached) {
        std::cout << "\t\t\tEdgeIntervals has not been cached we have to compute it  manually." << std::endl;
        std::vector<CfgType> path = {c.second,c.second,c.second};
        Robot* constraintRobot = c.second.GetRobot();
        DynamicObstacle dyOb(constraintRobot, path);
        dyOb.SetStartTime(c.first-1);
        this->GetMPProblem()->AddDynamicObstacle(std::move(dyOb));
        auto edgeIntervals = ComputeIntervals(_robot);
        // This is not ok since we will print the number of robots
        auto newIndex = m_cacheIndex;
        ++m_cacheIndex;
        m_edgeIntervalsMap[_robot][newIndex] = edgeIntervals;
        m_groupConflictsCache[_robot].emplace(c.first,std::make_pair(c.second,newIndex));
        std::cout << "\t\t\t\tEMPLACING NEW CONFLICT "
        << "cfg: " << c.second.PrettyPrint() << ", index: " << newIndex << ", timestep: " << c.first << std::endl;
        edgeIntervalsSet.push_back(edgeIntervals);
        this->GetMPProblem()->ClearDynamicObstacles();
      }
    }
    std::cout << "\t\t\tEdgeIntervals set has a size of " << edgeIntervalsSet.size() << std::endl;
    jointEdgeIntervals = JoinEdgeIntervals(_robot,edgeIntervalsSet);
  }
  edgeIntervalsSet.clear();

  size_t minEndtime = 0;
  if(m_currentConstraints) {
    for(auto c : *m_currentConstraints) {
      minEndtime = std::max(c.first, minEndtime);
    }
  }

  // Generate a path for this robot individually while avoiding the conflicts.
  auto groupTask = this->GetGroupTask();
  this->GetMPLibrary()->SetGroupTask(nullptr);
  this->GetMPLibrary()->SetTask(task);
  auto query = dynamic_cast<QueryMethod<MPTraits>*>(
      this->GetMapEvaluator(m_queryLabel)
  );
  query->SetPathWeightFunction(
      [this](typename RoadmapType::adj_edge_iterator& _ei,
             const double _sourceDistance,
             const double _targetDistance) {
        return this->MultiRobotPathWeight(_ei, _sourceDistance, _targetDistance);
      }
  );
  query->SetEdgeIntervals(jointEdgeIntervals);
  query->SetMinEndtime(minEndtime);
  const bool success = (*query)();
  this->GetMPLibrary()->SetTask(nullptr);
  this->GetMPLibrary()->SetGroupTask(groupTask);
  query->ClearPathWeightFunction();


  // Generate a path for this robot individually while avoiding the conflicts.
  // auto groupTask = this->GetMPLibrary()->GetGroupTask();
  // this->GetMPLibrary()->SetGroupTask(nullptr);
  // this->GetMPLibrary()->SetTask(task);
  // auto query = this->GetMapEvaluator(m_queryLabel);
  // const bool success = (*query)();
  // this->GetMPLibrary()->SetTask(nullptr);
  // this->GetMPLibrary()->SetGroupTask(groupTask);

  // Clear the conflicts.
  m_currentConstraints = nullptr;

  if(this->m_debug)
    std::cout << (_constraintMap.empty() ? "\t\t" : "\t\t\t\t")
              << "Path for robot " << _robot->GetLabel() << " was "
              << (success ? "" : "not ") << "found."
              << std::endl;

  // If we failed to find a path, return an empty pointer.
  if(!success)
    return nullptr;

  // Otherwise, return a copy of the robot's path from the solution object.
  Path* path = this->GetPath(_robot);
  //auto out = new Path(std::move(*path));
  //path->Clear();

  return path;
}

template <typename MPTraits>
std::pair<std::pair<Robot*, Robot*>, typename CBSQuery<MPTraits>::Conflict>
CBSQuery<MPTraits>::
FindConflict(const SolutionMap& _solution) {
  // Recreate each path at resolution level.
  std::map<Robot*, std::vector<CfgType>> cfgPaths;
  for(const auto& pair : _solution) {
    Robot* const robot = pair.first;
    const auto& path   = pair.second;
    cfgPaths[robot] = path->FullCfgs(this->GetMPLibrary());
  }

  // Find the latest timestep in which a robot is still moving.
  size_t lastTimestep = 0;
  for(auto& path : cfgPaths)
    lastTimestep = std::max(lastTimestep, path.second.size());

  auto vc = static_cast<CollisionDetectionValidity<MPTraits>*>(
      this->GetValidityChecker(m_vcLabel)
  );

  // Step through each timestep.
  for(size_t t = 0; t < lastTimestep; ++t) {
    // Collision check each robot path against all others at this timestep.
    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end();) {
      // Configure the first robot at the approriate configuration.
      auto robot1         = iter1->first;
      const auto& path1  = iter1->second;
      const size_t step1 = std::min(t, path1.size() - 1);
      const auto& cfg1   = path1[step1];
      auto multibody1    = robot1->GetMultiBody();
      multibody1->Configure(cfg1);

      // Compare to all remaining robots.
      for(auto iter2 = ++iter1; iter2 != cfgPaths.end(); ++iter2) {
        // Configure the second robot at the appropriate configuration.
        auto robot2         = iter2->first;
        const auto& path2  = iter2->second;
        const size_t step2 = std::min(t, path2.size() - 1);
        const auto& cfg2   = path2[step2];
        auto multibody2    = robot2->GetMultiBody();
        multibody2->Configure(cfg2);

        // Check for collision. If none, move on.
        CDInfo cdInfo;
        const bool collision = vc->IsMultiBodyCollision(cdInfo,
            multibody1, multibody2, this->GetNameAndLabel());
        if(!collision)
          continue;

        if(this->m_debug)
          std::cout << "\t\tConflict detected at timestep " << t
                    << " (time " << this->GetEnvironment()->GetTimeRes() * t
                    << ")."
                    << "\n\t\t\tRobot " << robot1->GetLabel() << ": "
                    << cfg1.PrettyPrint()
                    << "\n\t\t\tRobot " << robot2->GetLabel() << ": "
                    << cfg2.PrettyPrint()
                    << std::endl;

				//Old block pre-merge with master, remove if compiles
				//TODO::Was getting some weird complication bug about explicit construction
        Conflict newConflict(cfg1,cfg2,t);
        std::pair<Robot*, Robot*> robotPair = std::make_pair(robot1, robot2);
        //newConflict.cfg1     = cfg1;
        //newConflict.cfg2     = cfg2;
        //newConflict.timestep = t;

        //Conflict newConflict{cfg1, cfg2, t};

        return std::make_pair(robotPair, newConflict);
      }
    }
  }

  if(this->m_debug)
    std::cout << "\t\tNo conflict detected." << std::endl;

  // We didn't find a conflict, return an empty one.
  // Again weird compilation - I'll figure these out later
  Conflict c;
  return std::make_pair(std::make_pair(nullptr, nullptr), c);
}


template <typename MPTraits>
double
CBSQuery<MPTraits>::
MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _startTime, const double _bestEndTime) const {
  // Compute the time when we will end this edge.
  const size_t startTime = static_cast<size_t>(std::llround(_startTime)),
               endTime   = startTime + _ei->property().GetTimeSteps();

  // If this end time isn't better than the current best, we won't use it. Return
  // without checking conflicts to save computation.
  if(endTime >= static_cast<size_t>(std::llround(_bestEndTime)))
    return endTime;

  // If there are no current conflicts, there is nothing to check.
  if(!m_currentConstraints)
    return endTime;

  // There is at least one conflict. Find the set which occurs between this
  // edge's start and end time.
  auto lower = LowerBound(startTime);
  auto upper = UpperBound(endTime);

  // If all of the conflicts happen before or after now, there is nothing to
  // check.
  const bool beforeNow = lower == m_currentConstraints->end();
  if(beforeNow)
    return endTime;

  const bool afterNow = upper == m_currentConstraints->begin();
  if(afterNow)
    return endTime;

  // Check the conflict set to see if this edge hits any of them.
  for(auto iter = lower; iter != upper; ++iter) {
    // Unpack the conflict data.
    const size_t timestep = iter->first;
    const CfgType& cfg    = iter->second;

    // Assert that the conflict occurs during this edge transition (remove this
    // later once we're sure it works right).
    const bool rightTime = startTime <= timestep and timestep <= endTime;
    if(!rightTime)
      throw RunTimeException(WHERE) << "The conflict set should only include "
                                    << "conflicts that occur during this range.";

    // Check if the conflict cfg hits this edge.
    const bool hitsEdge = !IsEdgeSafe(_ei->source(), _ei->target(), cfg);
    if(!hitsEdge)
      continue;

    if(this->m_debug)
      std::cout << "\t\t\t\t\tEdge (" << _ei->source() << ","
                << _ei->target() << ") collides against robot "
                << cfg.GetRobot()->GetLabel()
                << " at " << cfg.PrettyPrint()
                << std::endl;

    // The conflict blocks this edge.
    return std::numeric_limits<double>::infinity();
  }

  // There is no conflict and the end time is better!
  return endTime;
}

template<typename MPTraits>
std::set<std::pair<size_t, Cfg>>::iterator
CBSQuery<MPTraits>::
LowerBound(size_t bound) const {
    //const ConstraintSet* m_currentConstraints{nullptr};
    //typedef std::pair<size_t, CfgType> Constraint;
  auto bound_it = m_currentConstraints->end();
  for (auto it = m_currentConstraints->begin(); it != m_currentConstraints->end(); it++) {
    if (it->first >= bound){
      bound_it = it;
      break;
    }
  }
  return bound_it;
}

template<typename MPTraits>
std::set<std::pair<size_t, Cfg>>::iterator
CBSQuery<MPTraits>::
UpperBound(size_t bound) const {
    //const ConstraintSet* m_currentConstraints{nullptr};
    //typedef std::pair<size_t, CfgType> Constraint;
  auto bound_it = m_currentConstraints->end();
  for (auto it = m_currentConstraints->begin(); it != m_currentConstraints->end(); it++) {
    if (it->first > bound){
      bound_it = it;
      break;
    }
  }
  return bound_it;
}


template <typename MPTraits>
bool
CBSQuery<MPTraits>::
IsEdgeSafe(const VID _source, const VID _target, const CfgType& _conflictCfg)
    const {
  auto robot = this->GetTask()->GetRobot();
  auto roadmap = this->GetRoadmap(robot);

  // Reconstruct the edge path at resolution-level.
  std::vector<CfgType> path;
  path.push_back(roadmap->GetVertex(_source));
  std::vector<CfgType> edge = this->GetMPLibrary()->ReconstructEdge(
      roadmap, _source, _target);
  path.insert(path.end(), edge.begin(), edge.end());
  path.push_back(roadmap->GetVertex(_target));

  // Get the valididty checker and make sure it has type
  // CollisionDetectionValidity.
  /// @TODO Figure out how to avoid needing this downcast so that we can
  ///       leverage more efficient compose checks (like checking the bounding
  ///       spheres first).
  auto basevc = this->GetValidityChecker(m_vcLabel);
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc);

  // Configure the other robot at _conflictCfg.
  auto otherMultiBody = _conflictCfg.GetRobot()->GetMultiBody();
  otherMultiBody->Configure(_conflictCfg);

  // Check each configuration in the resolution-level path for collision with
  // _conflictCfg.
  CDInfo cdInfo;
  auto thisMultiBody = robot->GetMultiBody();
  for(const CfgType& cfg : path) {
    thisMultiBody->Configure(cfg);
    if(vc->IsMultiBodyCollision(cdInfo, thisMultiBody, otherMultiBody,
        this->GetNameAndLabel()))
      return false;
  }

  // If we haven't detected a collision, the edge is safe.
  return true;
}


template <typename MPTraits>
typename CBSQuery<MPTraits>::EdgeIntervals
CBSQuery<MPTraits>::
ComputeIntervals(Robot* _robot) {
  MethodTimer mt(this->GetStatClass(),
    this->GetNameAndLabel() + "::ComputeIntervals");
  auto si = this->GetMPTools()->GetSafeIntervalTool(m_safeIntervalLabel);
  //std::cout << "Using SI Tool: "
  //          << si->GetLabel()
  //          << std::endl;
  //std::cout << "Computing Intervals" << std::endl;
  auto roadmap = this->GetRoadmap(_robot);
  typename CBSQuery<MPTraits>::EdgeIntervals edgeIntervals;
  for(auto vi = roadmap->begin(); vi != roadmap->end(); ++vi) {
    //auto vid = roadmap->GetVID(vi);
    //auto cfg = roadmap->GetVertex(vi);
    //auto vertexIntervals = si->ComputeIntervals(cfg);
    //m_roadmap_intervals[vid] = vertexIntervals;
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {

      //if(ei->source() == 602 and ei->target() == 534)
      //  std::cout << "HERE" << std::endl;
      MethodTimer mt(this->GetStatClass(),
        this->GetNameAndLabel() + "::EdgeIntervals");
      auto singleEdgeIntervals = si->ComputeIntervals(ei->property(), ei->source(),
          ei->target(), roadmap);
      edgeIntervals[ei->source()][ei->target()] = singleEdgeIntervals;
    }
  }
  return edgeIntervals;
}


template <typename MPTraits>
typename CBSQuery<MPTraits>::EdgeIntervals
CBSQuery<MPTraits>::
JoinEdgeIntervals(Robot* _robot, std::vector<typename CBSQuery<MPTraits>::EdgeIntervals> _edgeIntervals) {

  auto roadmap = this->GetRoadmap(_robot);
  typename CBSQuery<MPTraits>::EdgeIntervals jointEdgeIntervals;
  for(auto vi = roadmap->begin(); vi != roadmap->end(); ++vi) {
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      std::vector<std::vector<Range<double>>> allIntervals;
      for(size_t i = 0 ; i < _edgeIntervals.size() ; ++i ) {
        auto intervals = _edgeIntervals[i][ei->source()][ei->target()];
        //std::cout << "edge(" << ei->source() << "," << ei->target() << ")[" << i << "]: " << intervals << std::endl;
        allIntervals.push_back(intervals);
        //MethodTimer mt(this->GetStatClass(),
        //  this->GetNameAndLabel() + "::EdgeIntervals");
      }
      auto jointIntervals = JoinIntervals(allIntervals);
      jointEdgeIntervals[ei->source()][ei->target()] = jointIntervals;
      //std::cout << "edge(" << ei->source() << "," << ei->target() << ")[joint]: " << jointIntervals << std::endl;
    }
  }
  return jointEdgeIntervals;
}


template <typename MPTraits>
std::vector<Range<double>>
CBSQuery<MPTraits>::
JoinIntervals(std::vector<std::vector<Range<double>>> _allIntervals) {
  std::vector<Range<double>> jointIntervals;
  for(auto intervals : _allIntervals)
    jointIntervals = InsertIntervals(jointIntervals, intervals);
  return jointIntervals;
}


template <typename MPTraits>
std::vector<Range<double>>
CBSQuery<MPTraits>::
InsertIntervals(std::vector<Range<double>> _jointIntervals, std::vector<Range<double>> _newIntervals) {

  std::vector<Range<double>> newJointIntervals;
  if(_newIntervals.empty()) {
    newJointIntervals = _jointIntervals;
    return newJointIntervals;
  } else if(_jointIntervals.empty()) {
    newJointIntervals = _newIntervals;
  return newJointIntervals;
  } else {
    for(size_t i = 0 ; i < _jointIntervals.size() ; ++i) {
      for(size_t j = 0 ; j < _newIntervals.size() ; ++j) {
        if(OverlapingIntervals(_jointIntervals[i], _newIntervals[j])) {
          auto newInterval = MergeIntervals(_jointIntervals[i],_newIntervals[j]);
          newJointIntervals.push_back(newInterval);
        }
      }
    }
  }
  return newJointIntervals;
  //std::vector<Range<double>> alternativeJointIntervals;
  //alternativeJointIntervals.push_back(newJointIntervals[0]);
  //alternativeJointIntervals.push_back(newJointIntervals[newJointIntervals.size()-1]);
  //return alternativeJointIntervals;
}


template <typename MPTraits>
bool
CBSQuery<MPTraits>::
OverlapingIntervals(Range<double> _existingInterval, Range<double> _newInterval) {
  if(_existingInterval.max <= _newInterval.min)
    return false;

  if(_existingInterval.min >= _newInterval.max)
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
