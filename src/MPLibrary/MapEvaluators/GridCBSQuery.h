#ifndef PMPL_GRID_CBS_QUERY_H_
#define PMPL_GRID_CBS_QUERY_H_

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
/// This method is is the preliminar implementation of a similar algorithm to 
/// CBS-MP. 
///
/// Reference:
///  Liron Cohen et. al.. "Optimal and Bounded-Suboptimal
///  Multi-Agent Motion Planning". 
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GridCBSQuery : public MapEvaluatorMethod<MPTraits> {

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

    /// A set of conflicts for a single robot, keyed on timestep.
    typedef std::multimap<size_t, CfgType> ConflictSet;

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

    GridCBSQuery();

    GridCBSQuery(XMLNode& _node);

    virtual ~GridCBSQuery() = default;

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
    /// @param _robot    The robot which is avoiding the conflict.
    /// @param _cfg      The cfg that _robot should avoid.
    /// @param _timestep The timestep when _cfg must be avoided.
    /// @param _parent   The parent CBS node (its conflicts will be copied).
    /// @param _tree     The CBS tree to add new nodes.
    void CreateChildNode(Robot* const _robot, CfgType&& _cfg,
        const size_t _timestep, const CBSNode& _parent, CBSTree& _tree);

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
    /// problems. Here the metric is the number of timesteps, and we return
    /// infinity if taking an edge would result in a inter-robot collision.
    /// @param _ei             An iterator to the edge we are checking.
    /// @param _sourceTimestep The shortest time to the source node.
    /// @param _bestTimestep   The best known time to the target node.
    /// @return The time to the target node via this edge, or infinity if taking
    ///         this edge would result in a collision with dynamic obstacles.
    double MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceTimestep, const double _bestTimestep) const;

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

    /// The current set of conflicts to avoid.
    const ConflictSet* m_currentConflicts{nullptr};

    /// The set of conflicts we've already tried to resolve.
    std::set<ConflictMap> m_conflictCache;

    /// A map from robot to task.
    std::unordered_map<Robot*, MPTask*> m_taskMap;

    std::string m_gridIntervalMapLabel; ///< The grid interval map tool label/

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GridCBSQuery<MPTraits>::
GridCBSQuery() {
  this->SetName("GridCBSQuery");
}


template <typename MPTraits>
GridCBSQuery<MPTraits>::
GridCBSQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("GridCBSQuery");

  m_queryLabel = _node.Read("queryLabel", true, "",
      "The individual query method. Must be derived from QueryMethod and "
      "should not be used by any other object.");

  m_nodeLimit = _node.Read("nodeLimit", false, m_nodeLimit,
      size_t(1), std::numeric_limits<size_t>::max(),
      "Maximum number of CBS nodes to expand before declaring failure.");

  m_vcLabel = _node.Read("vcLabel", true, "",
      "The validity checker for conflict detection. Must be a CD type.");
    // Parse the other options.
  m_gridIntervalMapLabel = _node.Read("gridIntervalMapLabel", false, "",
      "Label of the GridIntervalMap");

}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
GridCBSQuery<MPTraits>::
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
GridCBSQuery<MPTraits>::
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
    const bool conflictDetected = !conflict.Empty();
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
GridCBSQuery<MPTraits>::
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
GridCBSQuery<MPTraits>::
CreateChildNodes(Conflict&& _conflict, const CBSNode& _parent, CBSTree& _tree) {
  // Find the two robots involved in the conflict.
  auto robot1 = _conflict.cfg1.GetRobot(),
       robot2 = _conflict.cfg2.GetRobot();

  // Assign the conflict to each robot.
  CreateChildNode(robot1, std::move(_conflict.cfg2), _conflict.timestep,
      _parent, _tree);
  CreateChildNode(robot2, std::move(_conflict.cfg1), _conflict.timestep,
      _parent, _tree);
}


template <typename MPTraits>
void
GridCBSQuery<MPTraits>::
CreateChildNode(Robot* const _robot, CfgType&& _cfg, const size_t _timestep,
    const CBSNode& _parent, CBSTree& _tree) {
  if(this->m_debug)
    std::cout << "\t\tAttempting to create CBS node with conflict on robot "
              << _robot->GetLabel() << " at timestep "
              << _timestep << " colliding against robot "
              << _cfg.GetRobot()->GetLabel()
              << std::endl;

  // Initialize the child node by copying the parent.
  CBSNode child = _parent;

  // Assert that we aren't adding a duplicate conflict, which should be
  // impossible with a correct implementation.
  auto bounds = child.conflicts[_robot].equal_range(_timestep);
  for(auto iter = bounds.first; iter != bounds.second; ++iter)
    if(iter->second == _cfg) {
      if(this->m_debug)
        std::cout << "\t\t\tConflict double-assigned to robot "
                  << _robot->GetLabel() << ", skipping."
                  << std::endl;
      return;
    }

  child.conflicts[_robot].emplace(_timestep, std::move(_cfg));

#if 0
  for(auto conflictSet : child.conflicts) {
    for(auto conflict : conflictSet.second) {
      std::cout << "\t\t\t\t\tConflict on robot " << conflictSet.first->GetLabel()
        << " at time " << conflict.first << " against robot "
        << conflict.second.GetRobot()->GetLabel() << " at cfg "
        << conflict.second.PrettyPrint() << std::endl;
    }
  }
#endif

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
GridCBSQuery<MPTraits>::
ComputeCost(const Solution& _solution) {
  double cost = 0;
  for(const auto& pair : _solution)
    cost += pair.second->Length();
  return cost;
}


template <typename MPTraits>
std::shared_ptr<typename MPTraits::Path>
GridCBSQuery<MPTraits>::
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
GridCBSQuery<MPTraits>::
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
typename GridCBSQuery<MPTraits>::Conflict
GridCBSQuery<MPTraits>::
FindConflict(const Solution& _solution) {
  auto giMap = this->GetMPTools()->GetGridIntervalMap(m_gridIntervalMapLabel);
  // Recreate each path at resolution level.
  std::map<Robot*, std::vector<CfgType>> cfgPaths;
  for(const auto& pair : _solution) {
    Robot* const robot = pair.first;
    const auto& path   = pair.second;
    cfgPaths[robot] = path->FullCfgs(this->GetMPLibrary());
    giMap->UpdateReservationTable(path.get());
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
                    << " (time " << this->GetEnvironment()->GetTimeRes() * t
                    << ")."
                    << "\n\t\t\tRobot " << robot1->GetLabel() << ": "
                    << cfg1.PrettyPrint()
                    << "\n\t\t\tRobot " << robot2->GetLabel() << ": "
                    << cfg2.PrettyPrint()
                    << std::endl;

        Conflict newConflict{cfg1, cfg2, t};

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
GridCBSQuery<MPTraits>::
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
  if(!m_currentConflicts)
    return endTime;

  // There is at least one conflict. Find the set which occurs between this
  // edge's start and end time.
  auto lower = m_currentConflicts->lower_bound(startTime),
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


template <typename MPTraits>
bool
GridCBSQuery<MPTraits>::
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

/*----------------------------------------------------------------------------*/

#endif
