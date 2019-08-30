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

    typedef typename MPTraits::RoadmapType                        RoadmapType;
    typedef typename MPTraits::CfgType                                CfgType;
    typedef typename MPTraits::Path                                      Path;
    typedef typename RoadmapType::VID                                     VID;
    typedef typename RoadmapType::EID::edge_id_type                       EID;
    typedef typename CBSNode<MPTraits>::ConflictEdge             ConflictEdge;
    typedef typename CBSNode<MPTraits>::ConflictCfg               ConflictCfg;
    typedef typename SafeIntervalTool<MPTraits>::ConflictRobot  ConflictRobot;
    typedef typename SafeIntervalTool<MPTraits>::PairConflict    PairConflict;

    ///@}
        ///@name Local Types
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
    /// @param _num_robot An index for the current robot.
    /// @param _roadmap The current roadmap.
    void SetNewConflictInfo(CBSNode<MPTraits>& _currentCBSNode, size_t _num_robot, 
      RoadmapType*& _roadmap);
    
    /// Adding the new CBSNodes by extrating the conflict info of _pairCfg
    /// @param _cbsTree The conflict tree.
    /// @param _pairCfg The pair of conflicting cfgs within their timesteps.
    /// @param _cbsNodeCost The total cost of the node.
    void AddingChildCBSNodes(CBSTree<MPTraits>& _cbsTree, 
      CBSNode<MPTraits>& _currentCBSNode, PairConflict _pairCfg, 
      double _cbsNodeCost);

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
    bool MakeIndividualPlans(GroupTask*& groupTask, 
      CBSNode<MPTraits>*& _currentCBSNode);

    /// Once the best solution is found, this function set it back to the
    /// group task.
    /// @param The group task.
    /// @param The current best set of paths.
    /// @param A boolean flag for set or not the paths.
    void SetBestGroupPlan(GroupTask*& _groupTask, vector<Path>& _optimalPaths, 
      bool& _success);

    /// When a better solution is found, this function updates the current 
    /// best solution
    /// group task.
    /// @param The group task.
    /// @param The current best set of paths.
    /// @param The current best total cost.
    /// @param The current total cost of the current CBS node.
    /// @oaram The nubmer of feasible solutions.
    /// @param A boolean flag for update or not the paths.
    void UpdateBestGroupPlan(GroupTask*& _groupTask, vector<Path>& 
      _optimalPaths,double& _optimalCost, double _realCost, size_t& _num_sol,
       bool _success);

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
    bool ValidateGroupPlan(GroupTask*& _groupTask, CBSTree<MPTraits>& _cbsTree,
      CBSNode<MPTraits>& _currentCBSNode, SafeIntervalTool<MPTraits>* _siTool,
      vector<ConflictEdge>& _invalidEdgesAt, 
      vector<ConflictCfg>& _conflictCfgsAt, 
      vector<Path*> _paths, bool& _success, double _cbsNodeCost, 
      bool& _solutionFound);

    ///@}

    ///@name Internal State
    ///@{

    std::string m_queryLabel;  ///< Label for an individual query method.

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

  bool success = true;
  auto query = this->GetMapEvaluator(m_queryLabel);
  // Unset the group task.
  this->GetMPLibrary()->SetGroupTask(nullptr);
  /// In this part we will start to grow our CBSTree 

  CBSTree<MPTraits> cbsTree;
  std::vector<typename CBSNode<MPTraits>::ConflictEdge> invalidEdgesAt;
  std::vector<typename CBSNode<MPTraits>::ConflictCfg> conflictCfgsAt;

  CBSNode<MPTraits> rootCBSNode;
  cbsTree.push(rootCBSNode);
  double optimalCost = std::numeric_limits<double>::infinity();
  std::vector<Path> optimalPaths;
  bool solutionFound = false;
  size_t counter = 0;
  size_t num_sol = 0;
  do {
    std::cout << "\n ITERATION: " << counter+1 << "\n" << std::endl;
    CBSNode<MPTraits> currentCBSNode = cbsTree.top();
    cbsTree.pop();
    SafeIntervalTool<MPTraits>* siTool = this->GetMPTools()->
    	GetSafeIntervalTool("SI");
    std::vector<Path*> paths;
    double cbsNodeCost = 0;    
    size_t num_robot = 0;
    // Computing new individual paths that are consistent with the current 
    // conflicts
    for(auto& task : *groupTask) {
      auto robot = task.GetRobot();
      auto roadmap = this->GetRoadmap(robot);
      /// Setting new conflict information in current node
      SetNewConflictInfo(currentCBSNode, num_robot, roadmap);
      ++num_robot;
      success = true;
      this->GetMPLibrary()->SetTask(&task);
      {
      MethodTimer mt(this->GetStatClass(), 
      	"GroupCBSQuery::Replanning indivual paths");  
      // Running individual query for the robot in task
      success &= (*query)();
      }
      roadmap->ClearInvalidatedAt();
      roadmap->ClearConflictCfgsAt();

      if(!success) {
        /// If we fail in finding one path, we quit the current CBS node
        if(this->m_debug)
          std::cout << "Breaking, path for robot " << robot->GetLabel() 
          << " can't be computed." << std::endl;
        break;
      }      
    }
    ++counter;   
    if(!success) {
      continue;
     }
    if(this->m_debug)
      PrintPaths(groupTask);
    /// Collecting the paths and getting their total cost
    cbsNodeCost = TotalCost(groupTask);
    paths = CollectPaths(groupTask);
    // If in previous iterations we found a feasible solution, we will 
    // just check the paths with lower cost, for means of optimality
    if(cbsNodeCost > optimalCost and solutionFound) {
      if(this->m_debug)
        std::cout << "Breaking, the current solution is cheaper." << std::endl;
      paths.clear();
      continue;
    }    
    // If the current set of paths is cheaper, we will check it for conflicts,
    // if no conlict is found we continue, otherwise we analize the next node 
    if(!ValidateGroupPlan(groupTask, cbsTree, currentCBSNode, siTool,
     invalidEdgesAt,conflictCfgsAt, paths, success, cbsNodeCost, 
     solutionFound))
      continue;

    double realCost = cbsNodeCost;
    // If the found solution is better than the current best 
    // solution, we update it 
    UpdateBestGroupPlan(groupTask, optimalPaths, optimalCost, realCost, 
      num_sol, success);

    if(m_firstSol)
      break;
  } while (!cbsTree.empty() and m_numIter > counter );
  if(this->m_debug) 
    std::cout << "\nCBSTree fully explored, Number of solutions: " 
  		<< num_sol << std::endl; 
  //If there is a set of optimal paths, we set it back
  SetBestGroupPlan(groupTask, optimalPaths,success);

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
SetNewConflictInfo(CBSNode<MPTraits>& _currentCBSNode, size_t _num_robot, 
  RoadmapType*& _roadmap) {
  if(_currentCBSNode.m_conflictCfgsAt.empty() or 
    _currentCBSNode.m_conflictCfgsAt[_num_robot].empty())
    return;
  for(size_t i = 0; i < _currentCBSNode.
    m_conflictCfgsAt[_num_robot].size() ; ++i) {
    _roadmap->SetConflictCfgAt(_currentCBSNode.
      m_conflictCfgsAt[_num_robot][i].conflictCfg, _currentCBSNode.
      m_conflictCfgsAt[_num_robot][i].timestep, true);
  }
}


template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
AddingChildCBSNodes(CBSTree<MPTraits>& _cbsTree, 
  CBSNode<MPTraits>& _currentCBSNode,typename SafeIntervalTool<MPTraits>::
    PairConflict _pairCfg, double _cbsNodeCost) {
  // Creating CBS Node 1
  CBSNode<MPTraits> newCBSNode1;
  newCBSNode1.m_conflictCfgsAt = _currentCBSNode.m_conflictCfgsAt;
  newCBSNode1.m_invalidEdgesAt = _currentCBSNode.m_invalidEdgesAt;
  newCBSNode1.m_conflictCfgsAt[_pairCfg.conflict1.num_robot].
    push_back(_pairCfg.conflict1.conflictCfg);
  newCBSNode1.m_cost = _cbsNodeCost;
  _cbsTree.push(newCBSNode1);
  if(this->m_debug)
    std::cout << "Creating CBSnode with conflict on robot " 
      << _pairCfg.conflict1.num_robot << " at timestep " 
      << _pairCfg.conflict1.conflictCfg.timestep << std::endl;
  // Creating CBS Node 2
  CBSNode<MPTraits> newCBSNode2;
  newCBSNode2.m_conflictCfgsAt = _currentCBSNode.m_conflictCfgsAt;
  newCBSNode2.m_invalidEdgesAt = _currentCBSNode.m_invalidEdgesAt;
  newCBSNode2.m_conflictCfgsAt[_pairCfg.conflict2.num_robot].
    push_back(_pairCfg.conflict2.conflictCfg);
  newCBSNode2.m_cost = _cbsNodeCost;
  _cbsTree.push(newCBSNode2);
   if(this->m_debug)
    std::cout << "Creating CBSnode with conflict on robot " 
      << _pairCfg.conflict2.num_robot << " at timestep " 
      << _pairCfg.conflict1.conflictCfg.timestep << std::endl;
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
    std::cout << "\n\nVID Path for robot " << robot->GetLabel() 
      << ": " << path->VIDs() << std::endl;
  }
}


template <typename MPTraits>
bool
GroupCBSQuery<MPTraits>::
MakeIndividualPlans(GroupTask*& _groupTask, CBSNode<MPTraits>*& _currentCBSNode
  ) {
  size_t num_robot = 0;
  bool feasiblePath = true;
  for(auto& task : *_groupTask) {
    // Evaluate this task.
    auto robot = task.GetRobot();
    auto roadmap = this->GetRoadmap(robot);
    auto query = this->GetMapEvaluator(m_queryLabel);
    
    /// Setting new conflict information in current node
    SetNewConflictInfo(_currentCBSNode, num_robot, roadmap);
    ++num_robot;
    //success = true;
    this->GetMPLibrary()->SetTask(&task);
    {
    MethodTimer mt(this->GetStatClass(), 
      "GroupCBSQuery::Replanning indivual paths");  
    feasiblePath &= (*query)();
    }
    // roadmap->ClearInvalidatedAt();
    // roadmap->ClearConflictCfgsAt();
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
SetBestGroupPlan(GroupTask*& _groupTask, vector<Path>& _optimalPaths, 
  bool& _success) {
  if(!_optimalPaths.empty()) {
    _success = true; 
    size_t p = 0;
    for(auto& task : *_groupTask) {
      auto robot = task.GetRobot();
      auto path = this->GetPath(robot);
      *path =  _optimalPaths[p];
      this->SetPath(robot,path);
      ++p;
    }
  }
}


template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
UpdateBestGroupPlan(GroupTask*& _groupTask, vector<Path>& _optimalPaths,
  double& _optimalCost, double _realCost, size_t& _num_sol, bool _success) {
 if(_success) {
      ++_num_sol;
      if(_realCost < _optimalCost) {
        if(this->m_debug)      
          std::cout << "Updating optimal paths" << std::endl;
        _optimalCost = _realCost;
        for(auto& task : *_groupTask) {
          auto robot = task.GetRobot();
          auto path = this->GetPath(robot);
          _optimalPaths.push_back(*path);      
        }
      }
  }
}


template <typename MPTraits>
bool
GroupCBSQuery<MPTraits>::
ValidateGroupPlan(GroupTask*& _groupTask, CBSTree<MPTraits>& _cbsTree, 
  CBSNode<MPTraits>& _currentCBSNode, SafeIntervalTool<MPTraits>* _siTool, 
  vector<typename CBSNode<MPTraits>::ConflictEdge>& _invalidEdgesAt, 
  vector<typename CBSNode<MPTraits>::ConflictCfg>& _conflictCfgsAt, 
  vector<Path*> _paths, bool& _success, double _cbsNodeCost, 
  bool& _solutionFound) {
  // Reserving the size for the containers of invalidated Cfgs,
  if(_currentCBSNode.m_conflictCfgsAt.empty()) {
    for(size_t i = 0 ; i <=  _groupTask->GetRobotGroup()->Size()  ; ++i) {
      _invalidEdgesAt.clear();
      _currentCBSNode.m_invalidEdgesAt.push_back(_invalidEdgesAt);
      _conflictCfgsAt.clear();
      _currentCBSNode.m_conflictCfgsAt.push_back(_conflictCfgsAt);
    }
  }
  //Here we check the paths for conflicts
  auto pairCfg = _siTool->FindConflict(_paths);

  if( pairCfg.conflict1.num_robot != INVALID_VID) {
    //Collision found adding two new cbsNodes
    AddingChildCBSNodes(_cbsTree, _currentCBSNode, pairCfg, _cbsNodeCost);
    _success = false;
    _paths.clear();
    return false;
  }
  // If we got here it means we found a set of feasible paths
  _solutionFound = true;
  /// Getting total cost for feeding the m_cost of the CBS Nde
  _cbsNodeCost = TotalCost(_groupTask);
    _paths = CollectPaths(_groupTask);

  if(this->m_debug) {
    std::cout << "S  O  L  U  T  I  O  N    F  O  U  N  D\n" << std::endl;
    PrintPaths(_groupTask);
    std::cout << "CBSTree  has " << _cbsTree.size() << " nodes" 
      << std::endl;
  }
  return true;
}

/*----------------------------------------------------------------------------*/

#endif
