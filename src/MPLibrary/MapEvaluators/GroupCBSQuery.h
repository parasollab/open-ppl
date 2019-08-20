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

    typedef typename MPTraits::RoadmapType          RoadmapType;
    typedef typename MPTraits::CfgType              CfgType;
    typedef typename MPTraits::Path                 Path;
    typedef typename RoadmapType::EID::edge_id_type EID;

    ///@}
        ///@name Local Types
    ///@{

    typedef pair<pair<size_t,pair<CfgType,double>>,pair<size_t,pair<CfgType,
      double>>> PairConflict; ///< A pair of conflicts

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
    void SetNewConflictInfo(CBSNode<MPTraits>*& _currentCBSNode, size_t _num_robot, 
      RoadmapType*& _roadmap);
    
    /// Adding the new CBSNodes by extrating the conflict info of _pairCfg
    /// @param _cbsTree The conflict tree.
    /// @param _pairCfg The pair of conflicting cfgs within their timesteps.
    /// @param _cbsNodeCost The total cost of the node.
    void AddingChildCBSNodes(CBSTree<MPTraits>& _cbsTree, 
      CBSNode<MPTraits>*& _currentCBSNode, PairConflict _pairCfg, double _cbsNodeCost);

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

    ///@}

    ///@name Internal State
    ///@{

    std::string m_queryLabel;  ///< Label for an individual query method.

    size_t m_numIter; // The max number of iterations we will run in the CBS Tree

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
  std::vector<pair<pair<size_t,size_t>,double>> invalidEdgesAt;
  std::vector<std::pair<Cfg, double>> conflictCfgsAt;

  auto rootCBSNode = new CBSNode<MPTraits>;
  cbsTree.Insert(rootCBSNode);

  double optimalCost = std::numeric_limits<double>::infinity();
  std::vector<Path> optimalPaths;

  bool solutionFound = false;
  size_t counter = 0;
  size_t num_sol = 0;
  do {
    std::cout << "\n ITERATION: " << counter+1 << "\n" << std::endl;
    CBSNode<MPTraits>* currentCBSNode = cbsTree.GetMinNode();
    SafeIntervalTool<MPTraits>* siTool = this->GetMPTools()->
    	GetSafeIntervalTool("SI");
    std::vector<Path*> paths;
    double cbsNodeCost = 0;    
    // Collecting each path into a set of paths (all paths are already
    // computed, we are just getting them from each task object)
    size_t num_robot = 0;
    for(auto& task : *groupTask) {
      // Evaluate this task.
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
      success &= (*query)();
      }
      roadmap->ClearInvalidatedAt();
      roadmap->ClearConflictCfgsAt();
      // Running individual query for the robot in task
      if(!success) {
        /// If we fail in finding one path, we quit the current CBS node
        if(this->m_debug)
          std::cout << "Breaking, path for robot " << num_robot << " can't be computed." << std::endl;
        break;
      }      
    }
    ++counter;   
     if(!success) {
      continue;
     }
    /// Collecting the paths and getting their total cost
    cbsNodeCost = TotalCost(groupTask);
    paths = CollectPaths(groupTask);
    // If in previous iterations we found feasible solution, we will 
    // just check the paths with lower cost, for means of optimality
    if(cbsNodeCost > optimalCost and solutionFound) {
      if(this->m_debug)
        std::cout << "Breaking, the current solution is cheaper." << std::endl;
      paths.clear();
      continue;
    }    
    // Reserving the size for the containers of invalidated Cfgs,
    if(currentCBSNode->m_conflictCfgsAt.empty()) {
      for(size_t i = 0 ; i < num_robot ; ++i) {
        invalidEdgesAt.clear();
        currentCBSNode->m_invalidEdgesAt.push_back(invalidEdgesAt);
        conflictCfgsAt.clear();
        currentCBSNode->m_conflictCfgsAt.push_back(conflictCfgsAt);
      }
    }
    //Here we check the paths for conflicts
    auto pairCfg = siTool->FindConflict(paths);

    if( static_cast<int>(pairCfg.first.first) != -1) {
      //Collision found adding two new cbsNodes
      AddingChildCBSNodes(cbsTree, currentCBSNode, pairCfg, cbsNodeCost);
      success = false;
      paths.clear();
      continue;
    }
    // If we got here it means we found a set of feasible paths
    solutionFound = true;
    /// Getting total cost for feeding the m_cost of the CBS Nde
    cbsNodeCost = TotalCost(groupTask);
    paths = CollectPaths(groupTask);

    if(this->m_debug) {
      std::cout << "S  O  L  U  T  I  O  N    F  O  U  N  D\n" << std::endl;
      PrintPaths(groupTask);
      std::cout << "CBSTree  has " << cbsTree.Length() << " nodes" << std::endl;
    }

    double realCost = cbsNodeCost;
    if(success) {
      ++num_sol;
      if(realCost < optimalCost) {
        if(this->m_debug)      
          std::cout << "Updating optimal paths" << std::endl;
        optimalCost = realCost;
        for(auto& task : *groupTask) {
          auto robot = task.GetRobot();
          auto path = this->GetPath(robot);
          optimalPaths.push_back(*path);      
        }
      }
    }
    if(m_firstSol)
      break;
  } while (!cbsTree.Empty() and m_numIter > counter );
  if(this->m_debug) 
    std::cout << "\nCBSTree fully explored, Number of solutions: " 
  		<< num_sol << std::endl; 
  //If there is a set of optimal paths, we set it back
  if(!optimalPaths.empty()) {
    success = true; 
    size_t p = 0;
    for(auto& task : *groupTask) {
      auto robot = task.GetRobot();
      auto path = this->GetPath(robot);
      *path =  optimalPaths[p];
      this->SetPath(robot,path);
      ++p;
    }
  }
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
SetNewConflictInfo(CBSNode<MPTraits>*& _currentCBSNode, size_t _num_robot, 
  RoadmapType*& _roadmap) {
  /// Invalidating cfgs
  if(!_currentCBSNode->m_conflictCfgsAt.empty())
    if(!_currentCBSNode->m_conflictCfgsAt[_num_robot].empty())
      for(size_t i = 0; i < _currentCBSNode->
        m_conflictCfgsAt[_num_robot].size() ; ++i) {
        if(this->m_debug)
        _roadmap->SetConflictCfgAt(_currentCBSNode->
          m_conflictCfgsAt[_num_robot][i].first, _currentCBSNode->
          m_conflictCfgsAt[_num_robot][i].second, true);
      }
}


template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
AddingChildCBSNodes(CBSTree<MPTraits>& _cbsTree, CBSNode<MPTraits>*& _currentCBSNode, 
  PairConflict _pairCfg, double _cbsNodeCost) {
  // Creating CBS Node 1
  auto newCBSNode1 = new CBSNode<MPTraits>;
  newCBSNode1->m_conflictCfgsAt = _currentCBSNode->m_conflictCfgsAt;
  newCBSNode1->m_conflictCfgsAt[_pairCfg.first.first].
    push_back(make_pair(_pairCfg.first.second.first,
      _pairCfg.first.second.second));
  newCBSNode1->m_cost = _cbsNodeCost;
  _cbsTree.Insert(newCBSNode1);
  if(this->m_debug)
    std::cout << "Creating CBSnode with conflict on robot " 
      << _pairCfg.first.first << " at timestep " 
      << _pairCfg.first.second.second << std::endl;
  // Creating CBS Node 2
  auto newCBSNode2 = new CBSNode<MPTraits>;
  newCBSNode2->m_conflictCfgsAt = _currentCBSNode->m_conflictCfgsAt;
  newCBSNode2->m_invalidEdgesAt = _currentCBSNode->m_invalidEdgesAt;
  newCBSNode2->m_conflictCfgsAt[_pairCfg.second.first].
    push_back(make_pair(_pairCfg.second.second.first,
      _pairCfg.second.second.second));
  newCBSNode2->m_cost = _cbsNodeCost;
  _cbsTree.Insert(newCBSNode2);
   if(this->m_debug)
    std::cout << "Creating CBSnode with conflict on robot " 
      << _pairCfg.second.first << " at timestep " 
      << _pairCfg.second.second.second << std::endl;
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
/*----------------------------------------------------------------------------*/

#endif
