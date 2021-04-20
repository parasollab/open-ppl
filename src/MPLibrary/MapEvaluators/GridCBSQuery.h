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

    typedef typename MPTraits::RoadmapType          RoadmapType;
    typedef typename MPTraits::CfgType              CfgType;
    typedef typename MPTraits::Path                 Path;
    typedef typename RoadmapType::VID               VID;
    typedef typename RoadmapType::EdgeID            EdgeID;
    typedef typename GridIntervalMap<MPTraits>::CellConflict  CellConflict;

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

      std::vector<Robot*> robots;     ///< The conflicting robots
      size_t cellIndex;     ///< The index cell.


      /// @todo This is to support old gcc v4.x, replace with
      ///       default-constructed class members after we upgrade.
      Conflict(const size_t _timestep = 0,
          const std::vector<Robot*> _robots = std::vector<Robot*>(),
          const size_t _cellIndex = 0 )
           :
          timestep(_timestep),
          robots(_robots),
          cellIndex(_cellIndex)
      {}

      /// @return True if this is an empty conflict.
      bool Empty() const noexcept {
        return robots.size() <= 1;
      }
    };

    /// A set of conflicts for a single robot, keyed on timestep.
    typedef std::multimap<size_t, CfgType> ConflictSet;

    /// A mapping from robot to conflict set.
    typedef std::map<Robot*, ConflictSet> ConflictMap;

    /// A set of conflicts for a single robot, keyed on timestep.
    typedef std::multimap<size_t, size_t> CellConflictSet;

    /// A mapping from robot to conflict set.
    typedef std::map<Robot*, CellConflictSet> CellConflictMap;

    /// A solution to the query is a path for each robot. These are implemented
    /// with shared_ptr because we may have many copies of a particular path at
    /// once.
    typedef std::map<Robot*, std::shared_ptr<Path>> Solution;

    /// A node in a conflict-based search tree. It describes a set of conflicts
    /// which must be avoided in addition to the normal validity checks.
    struct CBSNode {

      double      cost{0};     ///< The total cost of the team's path.
      //ConflictMap conflicts;   ///< The conflicts in this node.
      CellConflictMap cellConflicts; 
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
    void CreateChildNode(Robot* const _robot, Robot* const _robot2, const size_t _cellIndex,
        const size_t _timestep, const CBSNode& _parent, CBSTree& _tree);

    /// Compute the total cost of the paths in a solution.
    /// @return The total cost of the solution.
    double ComputeCost(const Solution& _solution);

    /// Compute a path for an individual robot which avoids a set of conflicts.
    /// @param _robot     The individual robot.
    /// @param _conflicts The conflicts to avoid.
    /// @return A path if found, or empty pointer if not.
    std::shared_ptr<Path> SolveIndividualTask(Robot* const _robot, const CellConflictMap& _cellConflicts = {});

    /// Store a valid solution in the MPLibrary's solution object.
    /// @param _solution The solution paths to store.
    void SetSolution(Solution&& _solution);

    /// Check for inter-robot collisions in a solution.
    /// @param _solution The solution to check.
    /// @return A discovered conflict, or an empty one if none was found.
    Conflict FindConflict(const Solution& _solution);

    Conflict FindConflict2(const Solution& _solution);

    /// Define a function for computing path weights w.r.t. multi-robot
    /// problems. Here the metric is the number of timesteps, and we return
    /// infinity if taking an edge would result in a inter-robot collision.
    /// @param _ei             An iterator to the edge we are checking.
    /// @param _sourceTimestep The shortest time to the source node.
    /// @param _bestTimestep   The best known time to the target node.
    /// @return The time to the target node via this edge, or infinity if taking
    ///         this edge would result in a collision with dynamic obstacles.
    double MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceTimestep, const double _bestTimestep);

    // Check if an edge is collision-free with respect to a given workspace grid-cell
    bool IsEdgeSafe(const VID _source, const VID _target, const size_t _conflictCell);

    // Maps edges and vertices are to workspace grid cells, it uses the below functions
    void RoadmapsCellMapping();

    // Maps edges to workspace cells, an edge is represented as a vector of set of cells 
    // for storing accurately the sets of cells that are swept during each intermediate 
    // cfg of the edge. 
    void EdgeCellMapping(Robot* _robot, size_t _source, size_t _target);

    // Maps vertices to a set of cells.
    void VertexCellMapping(Robot* _robot, size_t _vid);

    // Updates the Reservation Table (whis is stored in the Grid Interval Map tool).
    // When a path is given, it is updated into the reservation table by updating vertices
    // and edges by retrieving their corresponding sets of cells that they swept from the
    // cellsEdge and cellsVertex caches. 
    void UpdatePathReservationTable(Path* _path, size_t _lastTimestep);

    //void VertexCellMapping(Robot* _robot, size_t _vid);

    ///@}
    ///@name Internal State
    ///@{

    GroupTask* m_groupTask;    ///< The group task we're working on.

    std::string m_queryLabel;  ///< Query method for making individual plans.
    std::string m_vcLabel;     ///< Validity checker for conflict detection.

    /// The maximum number of CBS tree nodes that will be expanded.
    size_t m_nodeLimit{std::numeric_limits<size_t>::max()};

    /// The current set of conflicts to avoid.
    //const ConflictSet* m_currentConflicts{nullptr};

    /// The set of conflicts we've already tried to resolve.
    //std::set<ConflictMap> m_conflictCache;

    /// A map from robot to task.
    std::unordered_map<Robot*, MPTask*> m_taskMap;

    std::string m_gridIntervalMapLabel; ///< The grid interval map tool label/


    const CellConflictSet* m_currentCellConflicts{nullptr}; // The current set of cellconflicts

    std::set<CellConflictMap> m_cellConflictCache; 

    bool m_alternativeFindConflict{false}; // A flag for utilizing a alternative FindConflict function

    std::unordered_map<Robot*,
      std::unordered_map<size_t,
        std::unordered_map<size_t, 
          std::vector<std::unordered_set<size_t>>>>> m_cellsEdgeCache; // The sets of cells that the existing edges swept.

    std::unordered_map<Robot*,
        std::unordered_map<size_t, 
          std::unordered_set<size_t>>> m_cellsVertexCache; // The sets of cells that the exisiting vertices swept.

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

  m_alternativeFindConflict = _node.Read("alternativeFindConflict", false, m_alternativeFindConflict,
      "Boolean flag to choose alternative FindConflict function");

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

  {
  MethodTimer mt(this->GetStatClass(),
    this->GetNameAndLabel() + "::PreProccesing::RoadmapsCellMapping"); 
  RoadmapsCellMapping();
  }

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
    this->GetStatClass()->IncStat("CBSNodes");
    if(this->m_debug)
      std::cout << "\tStarting iteration " << counter << "."
                << std::endl;

    // Get the next best CBS node.
    CBSNode currentCBSNode = tree.top();
    tree.pop();

    // Check the solution for conflicts.

    Conflict conflict;

    if(!m_alternativeFindConflict) 
      conflict = FindConflict(currentCBSNode.solution);
    else 
      conflict = FindConflict2(currentCBSNode.solution);

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
  m_cellConflictCache.clear();

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

  auto robot1 = _conflict.robots[0],
        robot2 = _conflict.robots[1];

  // Assign the conflict to each robot.
  CreateChildNode(robot1, robot2, _conflict.cellIndex, _conflict.timestep,
      _parent, _tree);
  CreateChildNode(robot2, robot1, _conflict.cellIndex, _conflict.timestep,
      _parent, _tree);
}


template <typename MPTraits>
void
GridCBSQuery<MPTraits>::
CreateChildNode(Robot* const _robot, Robot* const _robot2, const size_t _cellIndex, 
  const size_t _timestep, const CBSNode& _parent, CBSTree& _tree) {
  auto giMap = this->GetMPTools()->GetGridIntervalMap(m_gridIntervalMapLabel);
  
  if(this->m_debug)
    std::cout << "\t\tAttempting to create CBS node with conflict on robot "
              << _robot->GetLabel() << " at timestep "
              << _timestep << ", at cell " << _cellIndex << " with center at " 
              << giMap->CellCenter(_cellIndex) << " colliding against robot "
              << _robot2->GetLabel() << std::endl;

  // Initialize the child node by copying the parent.
  CBSNode child = _parent;

  // Assert that we aren't adding a duplicate conflict, which should be
  // impossible with a correct implementation.
  auto cellBounds = child.cellConflicts[_robot].equal_range(_timestep);
  for(auto iter = cellBounds.first; iter != cellBounds.second; ++iter)
    if(iter->second == _cellIndex) {
      if(this->m_debug)
        std::cout << "\t\t\tConflict double-assigned to robot "
                  << _robot->GetLabel() << ", skipping."
                  << std::endl;
      return;
    }


  child.cellConflicts[_robot].emplace(_timestep, _cellIndex);

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
  if(m_cellConflictCache.count(child.cellConflicts)) {
    if(this->m_debug)
      std::cout << "\t\t\tThis conflict set was already attempted, skipping."
                << std::endl;
    return;
  }

  m_cellConflictCache.insert(child.cellConflicts);



  // Find a path for this robot. If it fails, discard the new node.
  auto path = SolveIndividualTask(_robot,  child.cellConflicts);
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
SolveIndividualTask(Robot* const _robot, const CellConflictMap& _cellConflicts) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::ConflictResolution");

  MPTask* const task = m_taskMap.at(_robot);
  if(this->m_debug)
    std::cout << (_cellConflicts.empty() ? "\t" : "\t\t\t")
              << "Solving task '" << task->GetLabel() << "' (" << task
              << ") for robot '" << _robot->GetLabel() << "' (" << _robot
              << ")."
              << std::endl;

  // Set the conflicts to avoid.
  if(!_cellConflicts.empty())
    m_currentCellConflicts = &_cellConflicts.at(_robot);

  // Generate a path for this robot individually while avoiding the conflicts.
  this->GetMPLibrary()->SetTask(task);
  auto query = this->GetMapEvaluator(m_queryLabel);
  const bool success = (*query)();
  this->GetMPLibrary()->SetTask(nullptr);


    // Clear the conflicts.
  m_currentCellConflicts = nullptr;

  if(this->m_debug)
    std::cout << (_cellConflicts.empty() ? "\t\t" : "\t\t\t\t")
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


// This is the FindConflict() original function.
template <typename MPTraits>
typename GridCBSQuery<MPTraits>::Conflict
GridCBSQuery<MPTraits>::
FindConflict(const Solution& _solution) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::ConflictDetection");
  auto giMap = this->GetMPTools()->GetGridIntervalMap(m_gridIntervalMapLabel);
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

  // Since resrvation table stores safe intervals, we need to know the
  // last timestep of all paths. Then we need to go through all edges
  // of all paths and then updating the resetvation table passing just 
  // the set of cells and the time interval.  

  // Updating reservation tables of each cell
  std::vector<Path*> paths;
  for(const auto& pair : _solution) {
    const auto& path   = pair.second;
    paths.push_back(path.get());
    UpdatePathReservationTable(path.get(), lastTimestep);
  }
  auto cellConflict = giMap->FindEarliestConflict();
  giMap->ClearConflicts();

  if(!cellConflict.Empty()) {

    auto robot1 = cellConflict.robots[0];
    auto robot2 = cellConflict.robots[1];

    if(this->m_debug)
            std::cout << "\t\tConflict detected at timestemp " << cellConflict.timestep
                      << " (time " << this->GetEnvironment()->GetTimeRes() * cellConflict.timestep
                      << ", cell " << cellConflict.cellIndex << ".)"
                      << "\n\t\t\tRobot " << robot1->GetLabel() << " and  "
                      << "\n\t\t\tRobot " << robot2->GetLabel() << ". "
                      << std::endl;

        Conflict newConflict{cellConflict.timestep, cellConflict.robots, cellConflict.cellIndex};

        return newConflict;
  }

  if(this->m_debug)
    std::cout << "\t\tNo conflict detected." << std::endl;

  // We didn't find a conflict, return an empty one.
  return {};
}


// This is the FindConflict() second function.
template <typename MPTraits>
typename GridCBSQuery<MPTraits>::Conflict
GridCBSQuery<MPTraits>::
FindConflict2(const Solution& _solution) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::ConflictDetection");
  auto giMap = this->GetMPTools()->GetGridIntervalMap(m_gridIntervalMapLabel);
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

  double cellsPerCfg = 0;
  double counter = 0;

  // Step through each timestep.
  for(size_t t = 0; t < lastTimestep; ++t) {
    // Collision check each robot path against all others at this timestep.
    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end();) {
      // Configure the first robot at the approriate configuration.
      auto robot1        = iter1->first;
      const auto& path1  = iter1->second;
      const size_t step1 = std::min(t, path1.size() - 1);
      const auto& cfg1   = path1[step1];
      auto cells1 = giMap->LocateCells(cfg1);

      cellsPerCfg += cells1.size();
      ++counter;

      // Compare to all remaining robots.
      for(auto iter2 = ++iter1; iter2 != cfgPaths.end(); ++iter2) {
        // Configure the second robot at the appropriate configuration.
        auto robot2        = iter2->first;
        const auto& path2  = iter2->second;
        const size_t step2 = std::min(t, path2.size() - 1);
        const auto& cfg2   = path2[step2];
        auto cells2 = giMap->LocateCells(cfg2);

        bool collision = false;
        size_t cellIndex = 0;

        for(auto cell : cells1) {
          if(cells2.count(cell)) {
            collision = true;
            cellIndex = cell;
            break;
          }
        }

        if(!collision)
          continue;

        this->GetStatClass()->SetStat("CellsPerCfg", cellsPerCfg/counter);

        std::vector<Robot*> robots{robot1,robot2};
        
        if(this->m_debug)
            std::cout << "\t\tConflict detected at timestemp " << t
                      << " (time " << this->GetEnvironment()->GetTimeRes() * t
                      << ", cell " << cellIndex << ".)"
                      << "\n\t\t\tRobot " << robot1->GetLabel() << " and  "
                      << "\n\t\t\tRobot " << robot2->GetLabel() << ". "
                      << std::endl;

        Conflict newConflict{t, robots, cellIndex};

        return newConflict;
      }
    }
  }

  this->GetStatClass()->SetStat("CellsPerCfg", cellsPerCfg/counter);
 
  if(this->m_debug)
    std::cout << "\t\tNo conflict detected." << std::endl;

  // We didn't find a conflict, return an empty one.
  return {};
}


template <typename MPTraits>
double
GridCBSQuery<MPTraits>::
MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _startTime, const double _bestEndTime) {
  // Compute the time when we will end this edge.
  const size_t startTime = static_cast<size_t>(std::llround(_startTime)),
               endTime   = startTime + _ei->property().GetTimeSteps();

  // If this end time isn't better than the current best, we won't use it. Return
  // without checking conflicts to save computation.
  if(endTime >= static_cast<size_t>(std::llround(_bestEndTime)))
    return endTime;

  // If there are no current conflicts, there is nothing to check.
  if(!m_currentCellConflicts)
    return endTime;

  // There is at least one conflict. Find the set which occurs between this
  // edge's start and end time.
  auto lower = m_currentCellConflicts->lower_bound(startTime),
       upper = m_currentCellConflicts->upper_bound(endTime);

  // If all of the conflicts happen before or after now, there is nothing to
  // check.
  const bool beforeNow = lower == m_currentCellConflicts->end();
  if(beforeNow)
    return endTime;

  const bool afterNow = upper == m_currentCellConflicts->begin();
  if(afterNow)
    return endTime;

  // Check the conflict set to see if this edge hits any of them.
  for(auto iter = lower; iter != upper; ++iter) {
    // Unpack the conflict data.
    const size_t timestep = iter->first;
    const size_t cell    = iter->second;

    // Assert that the conflict occurs during this edge transition (remove this
    // later once we're sure it works right).
    const bool rightTime = startTime <= timestep and timestep <= endTime;
    if(!rightTime)
      throw RunTimeException(WHERE) << "The conflict set should only include "
                                    << "conflicts that occur during this range.";

    // Check if the conflict cfg hits this edge.
    const bool hitsEdge = !IsEdgeSafe(_ei->source(), _ei->target(), cell);
    if(!hitsEdge) 
      continue;

    auto giMap = this->GetMPTools()->GetGridIntervalMap(m_gridIntervalMapLabel);

    if(this->m_debug)
      std::cout << "\t\t\t\t\tEdge (" << _ei->source() << ","
                << _ei->target() << ") collides against cell "
                << cell << " at " << giMap->CellCenter(cell)
                << std::endl;

    // The conflict blocks this edge.
    return std::numeric_limits<double>::infinity();
  }

  // There is no conflict and the end time is better!
  return endTime;
}


template<typename MPTraits>
bool
GridCBSQuery<MPTraits>::
IsEdgeSafe(const VID _source, const VID _target, const size_t _conflictCell) {

  auto robot = this->GetTask()->GetRobot();
  {
  MethodTimer mt(this->GetStatClass(),
    this->GetNameAndLabel() + "::IsEdgeSafe::AccessingCache"); 
  auto it = m_cellsEdgeCache.find(robot);
  if(it != m_cellsEdgeCache.end()) {
    // std::cout << "Robot exists" << std::endl;
    auto it2 = it->second.find(_source);
    if(it2 != it->second.end()) {
      // std::cout << "vid source exists" << std::endl;
      auto it3 = it2->second.find(_target);
      if(it3 != it2->second.end()) {
        // std::cout << "vid target exists" << std::endl;
        this->GetStatClass()->IncStat("ACCounter");
        auto cellsEdge = it3->second;
        for(size_t i = 0 ; i < cellsEdge.size() ; ++i)
          if(cellsEdge[i].count(_conflictCell)) {
            // std::cout << "Invalidating edge since conflictCell is contained in the edge" << std::endl;
            return false;
          }
      }
    }
  }
  return true;
  }  
}


template<typename MPTraits>
void
GridCBSQuery<MPTraits>::
RoadmapsCellMapping() {

  for(Robot* const robot : *m_groupTask->GetRobotGroup()) {
    auto roadmap = this->GetRoadmap(robot);
    auto numVertices = roadmap->get_num_vertices();
    for(size_t i = 0 ; i < numVertices; ++i) {
      VertexCellMapping(robot,i);
      for(size_t j = 0 ; j < numVertices; ++j ) {
        if(roadmap->IsEdge(i,j)) 
          EdgeCellMapping(robot,i,j);
      }  
    }
  }
}


template<typename MPTraits>
void
GridCBSQuery<MPTraits>::
EdgeCellMapping(Robot* _robot, size_t _source, size_t _target) {

  auto roadmap = this->GetRoadmap(_robot);

  // Reconstruct the edge path at resolution-level.
  std::vector<CfgType> path;
  std::vector<CfgType> edge = this->GetMPLibrary()->ReconstructEdge(
      roadmap, _source, _target);
  path.insert(path.end(), edge.begin(), edge.end());
  auto giMap = this->GetMPTools()->GetGridIntervalMap(m_gridIntervalMapLabel);
  std::vector<std::unordered_set<size_t>> cellsEdge;
  for(const CfgType& cfg : path) {
    auto cells = giMap->LocateCells(cfg);

    cellsEdge.push_back(cells);
  }

  m_cellsEdgeCache[_robot][_source][_target] = cellsEdge;

}


template<typename MPTraits>
void
GridCBSQuery<MPTraits>::
VertexCellMapping(Robot* _robot, size_t _vid) {

  auto roadmap = this->GetRoadmap(_robot);
  auto cfg = roadmap->GetVertex(_vid);
  auto giMap = this->GetMPTools()->GetGridIntervalMap(m_gridIntervalMapLabel);
  auto cellsVertex = giMap->LocateCells(cfg);
  m_cellsVertexCache[_robot][_vid] = cellsVertex;
}  


template<typename MPTraits>
void
GridCBSQuery<MPTraits>::
UpdatePathReservationTable(Path* _path, size_t _lastTimestep) {

  auto robot = _path->GetRobot();
  auto vidPath = _path->VIDs();
  size_t  minTime = 0, maxTime = 0;

  auto roadmap = this->GetRoadmap(robot);
  auto giMap = this->GetMPTools()->GetGridIntervalMap(m_gridIntervalMapLabel);

  // Retrieving the set of cells form the first vertex from cellVertex cache
  auto cellsVertex =  m_cellsVertexCache[robot][vidPath[0]];
  cellsVertex.clear();
  giMap->UpdateReservationTable(robot, cellsVertex, minTime, minTime);


  for(size_t i = 0 ; i < vidPath.size()-1 ; ++i) {
    // Retrieving the vector's of set of cells for each edge from the cellEdge cache
    auto edgeProperty = roadmap->GetEdge(vidPath[i],vidPath[i+1]);
    auto cellsEdge = m_cellsEdgeCache[robot][vidPath[i]][vidPath[i+1]];
    for(size_t j = 0 ; j < cellsEdge.size() ; ++j) {
      giMap->UpdateReservationTable(robot, cellsEdge[j], minTime + j, minTime + j);
      ++maxTime;
    }

    auto cellsVertex =  m_cellsVertexCache[robot][vidPath[0]];
    giMap->UpdateReservationTable(robot, cellsVertex, maxTime, maxTime);
    
    minTime = maxTime;

    cellsEdge.clear();
    cellsVertex.clear();
  }

  // Once we finish traversing the path, if there ais at least one robot that is still moving 
  // we add those timestep to the reservation table for not making the robot "disappear" once it gets
  // to the goal. 
  if(_lastTimestep > maxTime) {
    cellsVertex =  m_cellsVertexCache[robot][vidPath[vidPath.size()-1]];
    giMap->UpdateReservationTable(robot, cellsVertex, maxTime, _lastTimestep);
  }
}
 

/*----------------------------------------------------------------------------*/

#endif
