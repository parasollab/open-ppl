#ifndef PMPL_GROUP_CBS_QUERY_H_
#define PMPL_GROUP_CBS_QUERY_H_

#include "MapEvaluatorMethod.h"
#include "QueryMethod.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"

#include "ConfigurationSpace/Cfg.h"

#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// This method is a migration of a Discrete MAPF algorithm called
/// CBS (Conflict-based Search). It extracts all the individual plans of the
/// robots, then relying on the data structure "Conflict Tree", the individual
/// plans are checked for conflicts and then the conflicts are fixed using
/// a low-level search algorithm. Currently Dijkstra's algorithm is used.
///
/// Reference:
///  Guni Sharon et. al.. "Conflict-based search for optimal multi-agent
///  pathfinding". Artificial Intelligence (Elsevier journal) 2015.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupCBSQuery : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType   RoadmapType;
    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::Path          Path;
    typedef typename RoadmapType::VID        VID;
    typedef typename RoadmapType::EdgeID     EdgeID;

    ///@}

  private:

    ///@name Internal Types
    ///@{

    /// A conflict decribes an inter-robot collision between two different
    /// robots at a given time.
    struct Conflict {
      CfgType cfg1;      ///< The first robot's configuration.
      CfgType cfg2;      ///< The second robot's configuration.
      double  time{-1};  ///< The time when the collision occurred.
    };

    /// A set of conflicts for a single robot, keyed on time.
    typedef std::multimap<double, CfgType> ConflictSet;

    /// A mapping from robot to conflict set.
    typedef std::map<Robot*, ConflictSet> ConflictMap;

    /// A solution to the query is a path for each robot. These are implemented
    /// with shared_ptr because we may have many copies of a particular path at
    /// once.
    typedef std::map<Robot*, std::shared_ptr<Path>> Solution;

    /// A node in a conflict-based search tree. It describes a set of conflicts
    /// which must be avoided in addition to the normal validity checks.
    struct CBSNode {

      double      cost{0};     ///< The total cost of the team's path.
      ConflictMap conflicts;   ///< The conflicts in this node.
      Solution    solution;    ///< The constructed solution.

      /// Order by cost.
      bool operator>(const CBSNode& _other) const noexcept {
        return cost > _other.cost;
      }

    };

    /// A priority queue for finding the next cheapest CBS node to expand.
    using CBSTree = std::priority_queue<CBSNode,
                                        std::vector<CBSNode>,
                                        std::greater<CBSNode>>;

    ///@}

  public:

    ///@name Construction
    ///@{

    GroupCBSQuery();

    GroupCBSQuery(XMLNode& _node);

    virtual ~GroupCBSQuery() = default;

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

    /// Construct an initial solution to initialize the CBS tree.
    /// @param _tree The CBS tree.
    void ConstructInitialSolution(CBSTree& _tree);

    /// Try to add new child nodes to the CBS tree after detecting a conflict
    /// with the parent's solution.
    /// @param _conflict The conflict cfg and time.
    /// @param _parent   The parent node from.
    /// @param _tree     The conflict tree.
    void CreateChildNodes(Conflict&& _conflict, const CBSNode& _parent,
        CBSTree& _tree);

    /// Try to create a new child node which resolves a single conflict in the
    /// parent solution.
    /// @param _robot  The robot which is avoiding the conflict.
    /// @param _cfg    The cfg that _robot should avoid.
    /// @param _time   The time when _cfg must be avoided.
    /// @param _parent The parent CBS node (its conflicts will be copied).
    /// @param _tree   The CBS tree to add new nodes.
    void CreateChildNode(Robot* const _robot, CfgType&& _cfg,
        const double _time, const CBSNode& _parent, CBSTree& _tree);

    /// Compute the total cost of the paths in a solution.
    /// @return The total cost of the solution.
    double ComputeCost(const Solution& _solution);

    /// Compute a path for an individual robot which avoids a set of conflicts.
    /// @param _robot     The individual robot.
    /// @param _conflicts The conflicts to avoid.
    /// @return A path if found, or empty pointer if not.
    std::shared_ptr<Path> SolveIndividualTask(Robot* const _robot,
        const ConflictMap& _conflicts = {});

    /// Store a valid solution in the MPLibrary's solution object.
    /// @param _solution The solution paths to store.
    void SetSolution(Solution&& _solution);

    /// Check for inter-robot collisions in a solution.
    /// @param _solution The solution to check.
    /// @return A discovered conflict, or an empty one if none was found.
    Conflict FindConflict(const Solution& _solution);

    /// Define a function for computing path weights w.r.t. multi-robot
    /// problems. Here the metric is the number of time steps, and we return
    /// infinity if taking an edge would result in a inter-robot collision.
    /// @param _ei An iterator to the edge we are checking.
    /// @param _sourceDistance The shortest time to the source node.
    /// @param _targetDistance The best known time to the target node.
    /// @return The time to the target node via this edge, or infinity if taking
    ///         this edge would result in a collision with dynamic obstacles.
    double MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    /// Check if an edge is collision-free with respect to another robot's cfg.
    /// @param _source The cfg source of the edge.
    /// @param _target The cfg target of the edge.
    /// @param _conflictCfg The other robot's cfg to check.
    /// @return True if there is no collision between this robot traveling the
    ///         edge from _source to _target and the other robot at _conflictCfg.
    bool IsEdgeSafe(const VID _source, const VID _target,
        const CfgType& _conflictCfg) const;

    ///@}
    ///@name Internal State
    ///@{

    GroupTask* m_groupTask;    ///< The group task we're working on.

    std::string m_queryLabel;  ///< Query method for making individual plans.
    std::string m_vcLabel;     ///< Validity checker for conflict detection.

    /// The maximum number of CBS tree nodes that will be expanded.
    size_t m_nodeLimit{std::numeric_limits<size_t>::max()};

    std::multimap<EdgeID, double> m_edgeConflicts; ///< Set of edge conflicts.

    /// The current set of conflicts to avoid.
    const ConflictSet* m_currentConflicts{nullptr};

    /// The set of conflicts we've already tried to resolve.
    std::set<ConflictMap> m_conflictCache;

    /// A map from robot to task.
    std::unordered_map<Robot*, MPTask*> m_taskMap;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GroupCBSQuery<MPTraits>::
GroupCBSQuery() {
  this->SetName("GroupCBSQuery");
}


template <typename MPTraits>
GroupCBSQuery<MPTraits>::
GroupCBSQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("GroupCBSQuery");

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
GroupCBSQuery<MPTraits>::
Initialize() {
  // Assert that the validity checker is an instance of collision detection
  // validity.
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(
      this->GetValidityChecker(m_vcLabel).get()
  );
  if(!vc)
    throw RunTimeException(WHERE) << "Validity checker " << m_vcLabel
                                  << " is not of type "
                                  << "CollisionDetectionValidity.";

  // Assert that the query evaluator is an instance of query method.
  auto query = dynamic_cast<QueryMethod<MPTraits>*>(
      this->GetMapEvaluator(m_queryLabel).get()
  );
  if(!query)
    throw RunTimeException(WHERE) << "Query method " << m_queryLabel
                                  << " is not of type QueryMethod."
                                  << std::endl;

  // Set the query method's path weight function.
  query->SetPathWeightFunction(
      [this](typename RoadmapType::adj_edge_iterator& _ei,
             const double _sourceDistance,
             const double _targetDistance) {
        return this->MultiRobotPathWeight(_ei, _sourceDistance, _targetDistance);
      }
  );
}

/*-------------------------- MapEvaluator Overrides --------------------------*/

template <typename MPTraits>
bool
GroupCBSQuery<MPTraits>::
operator()() {
  // Set the task we're working on.
  m_groupTask = this->GetGroupTask();

  if(this->m_debug)
    std::cout << "Running CBS decoupled query for robot group '"
              << m_groupTask->GetRobotGroup()->GetLabel() << "' ("
              << m_groupTask->GetRobotGroup() << "), "
              << "task '" << m_groupTask->GetLabel() << "' ("
              << m_groupTask << ")."
              << std::endl;

  // Clear the group task for creating the individual plans.
  this->GetMPLibrary()->SetGroupTask(nullptr);

  // Intialize the CBS tree.
  CBSTree tree;
  ConstructInitialSolution(tree);

  size_t counter = 0;
  bool success = false;

  // Test the next best solution for conflicts until we find a path that is
  // simultaneously valid for all robots.
  while(!tree.empty() and m_nodeLimit > counter) {
    // Increment iteration count.
    ++counter;
    if(this->m_debug)
      std::cout << "\tStarting iteration " << counter << "."
                << std::endl;

    // Get the next best CBS node.
    CBSNode currentCBSNode = tree.top();
    tree.pop();

    // Check the solution for conflicts.
    Conflict conflict = FindConflict(currentCBSNode.solution);

    // If we detected a conflict, this solution isn't valid. Add new child nodes
    // to the tree and move on to the next node.
    const bool conflictDetected = conflict.time != -1;
    if(conflictDetected) {
      CreateChildNodes(std::move(conflict), currentCBSNode, tree);
      continue;
    }

    // This is a valid solution with minimal cost.
    SetSolution(std::move(currentCBSNode.solution));
    success = true;
    if(this->m_debug) {
      std::cout << "\tSolution found with cost " << currentCBSNode.cost << "."
                << std::endl;
      for(Robot* const robot : *m_groupTask->GetRobotGroup())
        std::cout << "\t\tVID Path for robot " << robot->GetLabel()
                  << ": " << this->GetPath(robot)->VIDs()
                  << std::endl;
    }
    break;
  }

  if(this->m_debug and !success)
    std::cout << "\tNo solution found." << std::endl;

  // Restore the group task.
  this->GetMPLibrary()->SetGroupTask(m_groupTask);

  // Clear the conflict cache.
  m_conflictCache.clear();

  return success;
}

/*--------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
ConstructInitialSolution(CBSTree& _tree) {
  m_taskMap.clear();

  // Construct an initial solution with all robots ignoring each other.
  Solution initialSolution;
  for(auto& task : *m_groupTask) {
    // Ensure a maximum of one task per robot.
    auto robot = task.GetRobot();
    if(m_taskMap.count(robot))
      throw RunTimeException(WHERE) << "This class can't handle group tasks "
                                    << "with more than one individual task "
                                    << "for a given robot (robot "
                                    << robot->GetLabel() << " has multiple "
                                    << "tasks)."
                                    << std::endl;
    m_taskMap[robot] = &task;

    // Construct a path for this robot. If none exists, we can't solve the query
    // yet.
    auto path = SolveIndividualTask(robot);
    if(!path)
      return;
    initialSolution[robot] = path;
  }

  // Push our initial solution into the tree.
  CBSNode root;
  root.solution = initialSolution;
  root.cost = ComputeCost(initialSolution);
  _tree.push(root);
}


template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
CreateChildNodes(Conflict&& _conflict, const CBSNode& _parent, CBSTree& _tree) {
  // Find the two robots involved in the conflict.
  auto robot1 = _conflict.cfg1.GetRobot(),
       robot2 = _conflict.cfg2.GetRobot();

  // Assign the conflict to each robot.
  CreateChildNode(robot1, std::move(_conflict.cfg2), _conflict.time, _parent,
      _tree);
  CreateChildNode(robot2, std::move(_conflict.cfg1), _conflict.time, _parent,
      _tree);
}


template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
CreateChildNode(Robot* const _robot, CfgType&& _cfg, const double _time,
    const CBSNode& _parent, CBSTree& _tree) {
  if(this->m_debug)
    std::cout << "\t\tAttempting to create CBS node with conflict on robot "
              << _robot->GetLabel() << " at time "
              << _time << "."
              << std::endl;

  // Initialize the child node by copying the parent.
  CBSNode child = _parent;
  child.conflicts[_robot].emplace(_time, std::move(_cfg));

  // If we've already seen this set of conflicts, don't check them again.
  if(m_conflictCache.count(child.conflicts)) {
    if(this->m_debug)
      std::cout << "\t\t\tThis conflict set was already attempted, skipping."
                << std::endl;
    return;
  }
  m_conflictCache.insert(child.conflicts);

  // Find a path for this robot. If it fails, discard the new node.
  auto path = SolveIndividualTask(_robot, child.conflicts);
  if(!path)
    return;

  child.solution[_robot] = path;
  child.cost = ComputeCost(child.solution);
  _tree.push(child);

  if(this->m_debug)
    std::cout << "\t\t\tChild node created." << std::endl;
}


template <typename MPTraits>
double
GroupCBSQuery<MPTraits>::
ComputeCost(const Solution& _solution) {
  double cost = 0;
  for(const auto& pair : _solution)
    cost += pair.second->Length();
  return cost;
}


template <typename MPTraits>
std::shared_ptr<typename MPTraits::Path>
GroupCBSQuery<MPTraits>::
SolveIndividualTask(Robot* const _robot, const ConflictMap& _conflicts) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "SolveIndividualTask");

  MPTask* const task = m_taskMap.at(_robot);
  if(this->m_debug)
    std::cout << (_conflicts.empty() ? "\t" : "\t\t\t")
              << "Solving task '" << task->GetLabel() << "' (" << task
              << ") for robot '" << _robot->GetLabel() << "' (" << _robot
              << ")."
              << std::endl;

  // Set the conflicts to avoid.
  if(!_conflicts.empty())
    m_currentConflicts = &_conflicts.at(_robot);

  // Generate a path for this robot individually while avoiding the conflicts.
  this->GetMPLibrary()->SetTask(task);
  auto query = this->GetMapEvaluator(m_queryLabel);
  const bool success = (*query)();
  this->GetMPLibrary()->SetTask(nullptr);

  // Clear the conflicts.
  m_currentConflicts = nullptr;

  if(this->m_debug)
    std::cout << (_conflicts.empty() ? "\t\t" : "\t\t\t\t")
              << "Path for robot " << _robot->GetLabel() << " was "
              << (success ? "" : "not ") << "found."
              << std::endl;

  // If we failed to find a path, return an empty pointer.
  if(!success)
    return {};

  // Otherwise, return a copy of the robot's path from the solution object.
  Path* path = this->GetPath(_robot);
  auto out = std::shared_ptr<Path>(new Path(std::move(*path)));
  path->Clear();

  return out;
}


template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
SetSolution(Solution&& _solution) {
  for(auto& pair : _solution) {
    auto robot = pair.first;
    auto& path = pair.second;

    // Move the solution path into the MPSolution object.
    auto currentPath = this->GetPath(robot);
    *currentPath = std::move(*path);
  }
}


template <typename MPTraits>
typename GroupCBSQuery<MPTraits>::Conflict
GroupCBSQuery<MPTraits>::
FindConflict(const Solution& _solution) {
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
      this->GetValidityChecker(m_vcLabel).get()
  );
  const double timeRes = this->GetEnvironment()->GetTimeRes();

  // Step through each timestep.
  for(size_t t = 0; t < lastTimestep; ++t) {
    // Compute the time value from the time step and resolution.
    const double time = t * timeRes;

    // Collision check each robot path against all others at this timestep.
    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end();) {
      // Configure the first robot at the approriate configuration.
      auto robot1        = iter1->first;
      const auto& path1  = iter1->second;
      const size_t step1 = std::min(t, path1.size() - 1);
      const auto& cfg1   = path1[step1];
      auto multibody1    = robot1->GetMultiBody();
      multibody1->Configure(cfg1);

      // Compare to all remaining robots.
      for(auto iter2 = ++iter1; iter2 != cfgPaths.end(); ++iter2) {
        // Configure the second robot at the appropriate configuration.
        auto robot2        = iter2->first;
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
          std::cout << "\t\tConflict detected at timestemp " << t
                    << " (time " << time << ")."
                    << "\n\t\t\tRobot " << robot1->GetLabel() << ": "
                    << cfg1.PrettyPrint()
                    << "\n\t\t\tRobot " << robot2->GetLabel() << ": "
                    << cfg2.PrettyPrint()
                    << std::endl;

        Conflict newConflict;
        newConflict.cfg1 = cfg1;
        newConflict.cfg2 = cfg2;
        newConflict.time = time;

        return newConflict;
      }
    }
  }

  if(this->m_debug)
    std::cout << "\t\tNo conflict detected." << std::endl;

  // We didn't find a conflict, return an empty one.
  return {};
}


template <typename MPTraits>
double
GroupCBSQuery<MPTraits>::
MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _startTime, const double _bestEndTime) const {
  // Compute the time when we will end this edge.
  const double endTime = _startTime + _ei->property().GetTimeSteps();

  // If this end time isn't better than the current best, we won't use it. Return
  // without checking conflicts to save computation.
  if(endTime >= _bestEndTime)
    return endTime;

  // If there are no current conflicts, there is nothing to check.
  if(!m_currentConflicts)
    return endTime;

  // There is at least one conflict. Find the set which occurs between this
  // edge's start and end time.
  auto lower = m_currentConflicts->lower_bound(_startTime),
       upper = m_currentConflicts->upper_bound(endTime);

  // If all of the conflicts happen before or after now, there is nothing to
  // check.
  const bool beforeNow = lower == m_currentConflicts->end();
  if(beforeNow)
    return endTime;

  const bool afterNow = upper == m_currentConflicts->begin();
  if(afterNow)
    return endTime;

  // Check the conflict set to see if this edge hits any of them.
  for(auto iter = lower; iter != upper; ++iter) {
    // Unpack the conflict data.
    const double& time = iter->first;
    const CfgType& cfg = iter->second;

    // Assert that the conflict occurs during this edge transition (remove this
    // later once we're sure it works right).
    const bool rightTime = _startTime < time and time < endTime;
    if(!rightTime)
      throw RunTimeException(WHERE) << "The conflict set should only include "
                                    << "conflicts that occur during this range.";

    // Check if the conflict cfg hits this edge.
    const bool hitsEdge = !IsEdgeSafe(_ei->source(), _ei->target(), cfg);
    if(!hitsEdge)
      continue;

    // The conflict blocks this edge.
    return std::numeric_limits<double>::infinity();
  }

  // There is no conflict and the end time is better!
  return endTime;
}


template <typename MPTraits>
bool
GroupCBSQuery<MPTraits>::
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
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc.get());

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

/*----------------------------------------------------------------------------*/

#endif
