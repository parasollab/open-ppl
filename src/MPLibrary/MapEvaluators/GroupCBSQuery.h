#ifndef PMPL_GROUP_CBS_QUERY_H_
#define PMPL_GROUP_CBS_QUERY_H_

#include "MapEvaluatorMethod.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/CBSTree.h"
#include <limits>

////////////////////////////////////////////////////////////////////////////////
/// This method is a migration of a Discrete MAPF algorithm called
/// CBS (Conflict-based Search). It extracts all the individual plans of the
/// robots, then relying on the data structure "Conflict Tree", the individual
/// plans are checked for conflicts and then the conflicts are fixed using
/// a low-level search algorithm. Currently Dijkstra's algorithm is used.
///
/// Reference:
///  Guni Sharon et. al., "Conflict-based search for optimal multi-agent
///  pathfinding", Proceedings of the Twenty-Sixth AAAI Conference
///  on Artificial Intelligence
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupCBSQuery : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType                      RoadmapType;
    typedef typename MPTraits::CfgType                          CfgType;
    typedef typename MPTraits::Path                             Path;
    typedef typename RoadmapType::VID                           VID;
    typedef typename RoadmapType::EID::edge_id_type             EID;

    ///@}
    ///@name Local Types

    typedef typename CBSNode<MPTraits>::ConflictCfg             ConflictCfg;
    typedef typename CBSNode<MPTraits>::ConflictEdge            ConflictEdge;

    struct ConflictRobot {
      ConflictCfg conflictCfg;
      size_t numRobot;

    };

    typedef std::pair<ConflictRobot,ConflictRobot>              PairConflict;

    ///@}

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

    /// Extracting conflict info from CBSNode, for computing new
    /// individual paths.
    /// @param _currentCBSNode The current CBSNode.
    /// @param _numRobot An index for the current robot.
    /// @param _roadmap The current roadmap.
    void SetNewConflictInfo(CBSNode<MPTraits>& _currentCBSNode, size_t _numRobot,
      RoadmapType* _roadmap);

    /// Adding the new CBSNodes by extrating the conflict info of _pairCfg
    /// @param _cbsTree The conflict tree.
    /// @param _pairCfg The pair of conflicting cfgs within their timesteps.
    /// @param _currentNodeCost The total cost of the node.
    void AddingChildCBSNodes(CBSTree<MPTraits>& _cbsTree,
      const CBSNode<MPTraits>& _currentCBSNode, PairConflict _pairCfg);

    /// Computing the total cost of the paths in GroupTask.
    /// @param The group task.
    /// @return The total cost.
    double TotalCost(GroupTask* groupTask);

    /// Collecting the paths in GroupTask.
    /// @param The group task.
    /// @return A vector of paths.
    vector<Path*> CollectPaths(GroupTask* groupTask);

    ///  Printing the VID Paths in GroupTask
    /// @param The group task.
    void PrintPaths(GroupTask* groupTask);

    /// Computes individual paths for each robot, they are consistent with
    /// the current conflicts
    /// @param The group task.
    /// @param The current CBS node.
    /// @return True if all the paths were successfully computed
    bool MakeIndividualPlans(GroupTask* groupTask,
      CBSNode<MPTraits>& _currentCBSNode);

    /// Once the best solution is found, this function set it back to the
    /// group task.
    /// @param The group task.
    /// @param The current best set of paths.
    /// @param A boolean flag for set or not the paths.
    void SetBestGroupPlan(GroupTask* _groupTask, vector<Path>& _optimalPaths);

    /// When a better solution is found, this function updates the current
    /// best solution
    /// group task.
    /// @param The group task.
    /// @param The current best set of paths.
    /// @param The current best total cost.
    /// @param The current total cost of the current CBS node.
    /// @oaram The nubmer of feasible solutions.
    /// @param A boolean flag for update or not the paths.
    void UpdateBestGroupPlan(GroupTask* _groupTask, vector<Path>&
      _optimalPaths,double& _optimalCost, double _currentNodeCost);

    /// Validates a group plan, it determines if the plan is conflict free, if /// not it creates new CBS nodes for future iterations.
    /// @param The group task.
    /// @param _cbsTree The conflict tree.
    /// @param The current CBS node.
    /// @param The safe interval tool.
    /// @param A conflict on an edge, defined by the VIDs of the edge and the
    /// conflicting timestep.
    /// @param A conflict on a cfg, defined by the conflicting cfg if the
    /// other robot and the conflicting timestep.
    /// @param A boolean flag for the coming functions of the algorithm.
    /// @param The cost to update if the plan gets validated.
    /// @param A boolean flag to state when feasible a solution is found.
    bool ValidateGroupPlan(GroupTask* _groupTask, CBSTree<MPTraits>& _cbsTree,
      CBSNode<MPTraits>& _currentCBSNode, vector<Path*> _paths);

    /// Checking if a set of paths has inter-robot collisions.
    /// @param _paths The set f paths to check.
    PairConflict FindConflict(const vector<Path*>& _paths) ;

    ///@}

    ///@name Internal State
    ///@{

    std::string m_queryLabel;  ///< Label for an individual query method.

    std::string m_vcLabel; ///< The validity checker to use.

    size_t m_numIter; // The maximum iterations we will run in the CBS Tree

    bool m_firstSol{false}; // Return the first solution we found
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
      "The individual query method.");

  m_numIter = _node.Read("numIter", true,
          1, 0, MAX_INT, "Number of iterations");

  m_firstSol = _node.Read("firstSol", true, m_firstSol,
      "Return the first solution we found");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
Initialize() {
  if(this->m_debug)
    std::cout << "INITIALIZE '" << this->GetLabel() << "'" << std::endl;
  // Add maps to the goal tracker for the individual maps.
  auto goalTracker = this->GetGoalTracker();
  auto groupTask = this->GetGroupTask();
  for(auto& task : *groupTask) {
    auto roadmap = this->GetRoadmap(task.GetRobot());
    if(!goalTracker->IsMap(roadmap, &task))
      goalTracker->AddMap(roadmap, &task);
  }
}

/*-------------------------- MapEvaluator Overrides --------------------------*/

template <typename MPTraits>
bool
GroupCBSQuery<MPTraits>::
operator()() {
  auto groupTask = this->GetGroupTask();
  auto group = groupTask->GetRobotGroup();
  if(this->m_debug)
    std::cout << "\n\n\nRunning decoupled query for robot group '"
              << group->GetLabel() << "'."
              << std::endl;

  bool success = false;
  auto query = this->GetMapEvaluator(m_queryLabel);
  // Unset the group task.
  this->GetMPLibrary()->SetGroupTask(nullptr);
  /// In this part we will start to grow our CBSTree

  CBSTree<MPTraits> cbsTree;
  CBSNode<MPTraits> rootCBSNode;
  cbsTree.push(rootCBSNode);
  double optimalCost = std::numeric_limits<double>::infinity();
  std::vector<Path> optimalPaths;
  size_t counter = 0;
  size_t solutionCount = 0;
  do {
    if(this->m_debug)
      std::cout << "\n ITERATION: " << counter+1 << "\n" << std::endl;
    CBSNode<MPTraits> currentCBSNode = cbsTree.top();
    cbsTree.pop();
    ++counter;

    if(!MakeIndividualPlans(groupTask,currentCBSNode))
      continue;

    if(this->m_debug)
      PrintPaths(groupTask);
    /// Collecting the paths and getting their total cost
    //double currentNodeCost = 0;
    //currentNodeCost = TotalCost(groupTask);
    currentCBSNode.m_cost = TotalCost(groupTask);
    std::vector<Path*> paths;
    paths = CollectPaths(groupTask);
    // If in previous iterations we found a feasible solution, we will
    // just check the paths with lower cost, for means of optimality
    if(currentCBSNode.m_cost > optimalCost) {
      if(this->m_debug)
        std::cout << "Breaking, the current solution is cheaper." << std::endl;
      paths.clear();
      continue;
    }
    // If the current set of paths is cheaper, we will check it for conflicts,
    // if no conlict is found we continue, otherwise we analize the next node
    if(!ValidateGroupPlan(groupTask, cbsTree, currentCBSNode, paths))
      continue;

    success = true;

    // If the found solution is better than the current best
    // solution, we update it
    UpdateBestGroupPlan(groupTask, optimalPaths, optimalCost, currentCBSNode.m_cost);
    ++solutionCount;

    if(m_firstSol)
      break;
  } while (!cbsTree.empty() and m_numIter > counter );
  if(this->m_debug)
    std::cout << "\nCBSTree fully explored, Number of solutions: "
  		<< solutionCount << std::endl;
  //If there is a set of optimal paths, we set it back
  if(success)
    SetBestGroupPlan(groupTask, optimalPaths);

  if(this->m_debug)
    std::cout << "\tDone." << std::endl;
  // Restore the group task.
  this->GetMPLibrary()->SetTask(nullptr);
  this->GetMPLibrary()->SetGroupTask(groupTask);

  return success;
}

/*--------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
SetNewConflictInfo(CBSNode<MPTraits>& _currentCBSNode, size_t _numRobot,
  RoadmapType* _roadmap) {
  if(_currentCBSNode.m_cfgConflicts.empty() or
    _currentCBSNode.m_cfgConflicts[_numRobot].empty())
    return;
  for(size_t i = 0; i < _currentCBSNode.
    m_cfgConflicts[_numRobot].size() ; ++i) {
    _roadmap->SetConflictCfgAt(_currentCBSNode.
      m_cfgConflicts[_numRobot][i].conflictCfg, _currentCBSNode.
      m_cfgConflicts[_numRobot][i].timestep, true);
  }
}


template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
AddingChildCBSNodes(CBSTree<MPTraits>& _cbsTree,
  const CBSNode<MPTraits>& _currentCBSNode,typename GroupCBSQuery<MPTraits>::
    PairConflict _pairCfg) {
  // Creating CBS Node 1
  CBSNode<MPTraits> newCBSNode1;
  newCBSNode1.m_cfgConflicts = _currentCBSNode.m_cfgConflicts;
  newCBSNode1.m_edgeConflicts = _currentCBSNode.m_edgeConflicts;
  newCBSNode1.m_cfgConflicts[_pairCfg.first.numRobot].
    push_back(_pairCfg.first.conflictCfg);
  newCBSNode1.m_cost = _currentCBSNode.m_cost;
  _cbsTree.push(newCBSNode1);
  if(this->m_debug)
    std::cout << "Creating CBSnode with conflict on robot "
      << _pairCfg.first.numRobot << " at timestep "
      << _pairCfg.first.conflictCfg.timestep << std::endl;
  // Creating CBS Node 2
  CBSNode<MPTraits> newCBSNode2;
  newCBSNode2.m_cfgConflicts = _currentCBSNode.m_cfgConflicts;
  newCBSNode2.m_edgeConflicts = _currentCBSNode.m_edgeConflicts;
  newCBSNode2.m_cfgConflicts[_pairCfg.second.numRobot].
    push_back(_pairCfg.second.conflictCfg);
  newCBSNode2.m_cost = _currentCBSNode.m_cost;
  _cbsTree.push(newCBSNode2);
   if(this->m_debug)
    std::cout << "Creating CBSnode with conflict on robot "
      << _pairCfg.second.numRobot << " at timestep "
      << _pairCfg.first.conflictCfg.timestep << std::endl;
}


template <typename MPTraits>
double
GroupCBSQuery<MPTraits>::
TotalCost(GroupTask* groupTask) {
  double cost = 0;
  for(auto& task : *groupTask) {
    auto robot = task.GetRobot();
    auto path = this->GetPath(robot);
    cost += path->Length();
  }
  return cost;
}


template <typename MPTraits>
vector<typename MPTraits::Path*>
GroupCBSQuery<MPTraits>::
CollectPaths(GroupTask* groupTask) {
  vector<Path*> paths;
  for(auto& task : *groupTask) {
    auto robot = task.GetRobot();
    auto path = this->GetPath(robot);
    paths.push_back(path);
  }
  return paths;
}


template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
PrintPaths(GroupTask* groupTask) {
 for(auto& task : *groupTask) {
    auto robot = task.GetRobot();
    auto path = this->GetPath(robot);
    if(this->m_debug)
      std::cout << "\n\nVID Path for robot " << robot->GetLabel()
        << ": " << path->VIDs() << std::endl;
  }
}


template <typename MPTraits>
bool
GroupCBSQuery<MPTraits>::
MakeIndividualPlans(GroupTask* _groupTask, CBSNode<MPTraits>& _currentCBSNode
  ) {
  size_t numRobot = 0;
  bool feasiblePath = false;
  for(auto& task : *_groupTask) {
    // Evaluate this task.
    auto robot = task.GetRobot();
    auto roadmap = this->GetRoadmap(robot);
    auto query = this->GetMapEvaluator(m_queryLabel);

    /// Setting new conflict information in current node
    SetNewConflictInfo(_currentCBSNode, numRobot, roadmap);
    ++numRobot;
    //success = true;
    this->GetMPLibrary()->SetTask(&task);
    {
    MethodTimer mt(this->GetStatClass(),
      "GroupCBSQuery::Replanning indivual paths");
    feasiblePath = (*query)();
    }
    roadmap->ClearInvalidatedAt();
    roadmap->ClearConflictCfgsAt();
    // Running individual query for the robot in task
    if(!feasiblePath) {
      /// If we fail in finding one path, we quit the current CBS node
      if(this->m_debug)
        std::cout << "Breaking, path for robot " << robot->GetLabel()
        << " can't be computed." << std::endl;
      return false;
    }
  }
  return true;
}


template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
SetBestGroupPlan(GroupTask* _groupTask, vector<Path>& _optimalPaths) {
  for(auto& optimalPath : _optimalPaths) {
    auto robot = optimalPath.GetRobot();
    auto currentPath = this->GetPath(robot);
    *currentPath = optimalPath;
    this->GetMPSolution()->SetPath(robot,currentPath);
  } 
}


template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
UpdateBestGroupPlan(GroupTask* _groupTask, vector<Path>& _optimalPaths,
  double& _optimalCost, double _currentNodeCost) {
  _optimalPaths.clear();
  if(this->m_debug)
    std::cout << "Updating optimal paths" << std::endl;
  _optimalCost = _currentNodeCost;
  for(auto& task : *_groupTask) {
    auto robot = task.GetRobot();
    auto path = this->GetPath(robot);
    _optimalPaths.push_back(*path);
  }
}


template <typename MPTraits>
bool
GroupCBSQuery<MPTraits>::
ValidateGroupPlan(GroupTask* _groupTask, CBSTree<MPTraits>& _cbsTree,
  CBSNode<MPTraits>& _currentCBSNode, vector<Path*> _paths) {

  std::vector<typename CBSNode<MPTraits>::ConflictEdge> edgeConflicts;
  std::vector<typename CBSNode<MPTraits>::ConflictCfg> cfgConflicts;
  // Reserving the size for the containers of invalidated Cfgs,
  if(_currentCBSNode.m_cfgConflicts.empty()) {
    for(size_t i = 0 ; i <=  _groupTask->GetRobotGroup()->Size()  ; ++i) {
      _currentCBSNode.m_edgeConflicts.push_back(edgeConflicts);
      _currentCBSNode.m_cfgConflicts.push_back(cfgConflicts);
    }
  }
  //Here we check the paths for conflicts
  auto pairCfg = FindConflict(_paths);

  if( pairCfg.first.numRobot != INVALID_VID) {
    //Collision found adding two new cbsNodes
    AddingChildCBSNodes(_cbsTree, _currentCBSNode, pairCfg);
    _paths.clear();
    return false;
  }

  if(this->m_debug) {
    std::cout << "S  O  L  U  T  I  O  N    F  O  U  N  D\n" << std::endl;
    PrintPaths(_groupTask);
    std::cout << "CBSTree  has " << _cbsTree.size() << " nodes"
      << std::endl;
  }
  return true;
}


template <typename MPTraits>
typename GroupCBSQuery<MPTraits>::PairConflict
GroupCBSQuery<MPTraits>::
FindConflict(const std::vector<Path*>& _paths) {
  vector<vector<CfgType>> paths;
  for(const auto& path : _paths)
    //paths.push_back(FullPath(path));
    paths.push_back(path->FullCfgs(this->GetMPLibrary()));

  // Find the latest timestep in which a robot is still moving.
  size_t timeFinal = 0;
  for(auto& path : paths)
    if(path.size() > timeFinal)
      timeFinal = path.size();

  auto basevc = this->GetValidityChecker(m_vcLabel);
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc.get());

  for(size_t t = 0 ; t < timeFinal ; ++t) {
    const double timeRes = this->GetEnvironment()->GetTimeRes();
    for(size_t i = 0 ; i < paths.size() ; ++i ) {
      size_t t1 = t;
      if(t > paths[i].size()-1)
        t1 = paths[i].size()-1;
      auto robotMultiBody = paths[i][0].GetRobot()->GetMultiBody();
      robotMultiBody->Configure(paths[i][t1]);
      for(size_t j = 0 ; j < paths.size() ; ++j) {
        if ( i == j)
          continue;
        size_t t2 = t;
        if(t > paths[j].size()-1)
          t2 = paths[j].size()-1;
        auto obstacleMultiBody = paths[j][0].GetRobot()->GetMultiBody();
        obstacleMultiBody->Configure(paths[j][t2]);

        CDInfo cdInfo;
        if(vc->IsMultiBodyCollision(cdInfo, obstacleMultiBody, robotMultiBody,
            this->GetNameAndLabel())) {
          double ti = timeRes * static_cast<double>(t1);
          double tj = timeRes * static_cast<double>(t2);
          if(this->m_debug) {
            std::cout << "Conflict on robot " << i << " at timestep " << ti
            << " at cfg " << paths[i][t1].PrettyPrint() << "\nConflict on robot "
            << j << " at timestep "<< tj <<" at cfg " << paths[j][t2].PrettyPrint()
            << std::endl;
          }

          PairConflict pairConflict;
          pairConflict.first.conflictCfg.conflictCfg = paths[j][t2];
          pairConflict.first.conflictCfg.timestep = ti;
          pairConflict.first.numRobot = i;
          pairConflict.second.conflictCfg.conflictCfg = paths[i][t1];
          pairConflict.second.conflictCfg.timestep = tj;
          pairConflict.second.numRobot = j;


          return pairConflict;
        }
      }
    }
  }
  PairConflict nullPair;
  nullPair.first.numRobot = INVALID_VID;
  return nullPair;
}

/*----------------------------------------------------------------------------*/

#endif
