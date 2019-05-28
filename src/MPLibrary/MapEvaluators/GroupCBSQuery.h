#ifndef PMPL_GROUP_CBS_QUERY_H_
#define PMPL_GROUP_CBS_QUERY_H_

#include "MapEvaluatorMethod.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/CBSTree.h"

////////////////////////////////////////////////////////////////////////////////
/// Calls an individual query for each robot in the group to realize cooperative
/// A*. After each plan is extracted, that robot is treated as a dynamic
/// obstacle for the remaining robots.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupCBSQuery : public MapEvaluatorMethod<MPTraits> {

  public:

      ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType          RoadmapType;
    typedef typename MPTraits::Path                 Path;
    typedef typename RoadmapType::EID::edge_id_type EID;
    //typedef typename MPTraits::CBSNode CBSNode;

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

    ///@name Internal State
    ///@{

    std::string m_queryLabel;  ///< Label for an individual query method.

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
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
GroupCBSQuery<MPTraits>::
Initialize() {
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
  // For each individual task t in group task:
  // - Set individual task
  // - Run query
  // - create dynamic obstacle from path
  // Clear individual task
  // Clear dynamic obstacles
  auto groupTask = this->GetGroupTask();
  auto group = groupTask->GetRobotGroup();
  //if(this->m_debug)
  std::cout << "\n\n\nRunning decoupled query for robot group '"
              << group->GetLabel() << "'."
              << std::endl;

  bool success = true;
  auto query = this->GetMapEvaluator(m_queryLabel);



  // Unset the group task.
  this->GetMPLibrary()->SetGroupTask(nullptr);

  /// In this part we will start to grow our CBSTree ---------------------------------

  CBSTree<MPTraits> cbsTree;
  std::vector<size_t> invalidVertices;
  std::vector<pair<size_t,size_t>> invalidEdges;

  std::vector<pair<size_t,double>> invalidVerticesAt;
  std::vector<pair<pair<size_t,size_t>,double>> invalidEdgesAt;

  std::vector<std::pair<Cfg, double>> conflictCfgsAt;

  auto rootCBSNode = new CBSNode<MPTraits>;
  cbsTree.Insert(rootCBSNode);

  size_t counter = 0;
  do {
    std::cout << "\n ITERATION: " << counter+1 << "\n" << std::endl;
    std::cout << "---------------------------CBS conflict detection part--------------------" << std::endl;

    CBSNode<MPTraits>* currentCBSNode = cbsTree.GetMinNode();
    //groupTask = this->GetGroupTask();

    SafeIntervalTool<MPTraits>* siTool = this->GetMPTools()->GetSafeIntervalTool("SI");
    std::vector<Path*> paths;
    double cost = 0;
    // Collecting each path into a set of paths (all paths are already computed,
    // we are just getting them from each task object)
    // Here we defintely need to use the conflict information of our CBSNode
    size_t num_robot = 0;
    for(auto& task : *groupTask) {
      // if(!currentCBSNode->m_invalidVertices.empty() || !currentCBSNode->m_invalidEdges.empty()) {
      // Evaluate this task.
      auto robot = task.GetRobot();
      auto roadmap = this->GetRoadmap(robot);
      std::cout << "Roadmap " << num_robot << ", " << roadmap->get_num_vertices() << " vertices, " << roadmap->get_num_edges() << " edges." << std::endl;

      /// Invalitating vertices of robot "num_robot"
      if(!currentCBSNode->m_invalidVerticesAt.empty())
        if(!currentCBSNode->m_invalidVerticesAt[num_robot].empty())
          for(size_t i =0 ; i < currentCBSNode->m_invalidVerticesAt[num_robot].size() ; ++i) {
            std::cout << "Pre-Invalidating vertex " << currentCBSNode->m_invalidVerticesAt[num_robot][i].first << std::endl;
            std::cout  << "Size():  " << currentCBSNode->m_invalidVerticesAt[num_robot].size() << std::endl;
            //if(roadmap->IsVertex(currentCBSNode->m_invalidVerticesAt[num_robot][i].first))
            //  roadmap->DeleteVertex(currentCBSNode->m_invalidVerticesAt[num_robot][i].first);
            //currentCBSNode->m_invalidVerticesAt[num_robot].clear();
            //roadmap->SetVertexInvalidated(currentCBSNode->m_invalidVertices[num_robot][i],true);
            //roadmap->SetVertexInvalidatedAt(currentCBSNode->m_invalidVerticesAt[num_robot][i].first,currentCBSNode->m_invalidVerticesAt[num_robot][i].second,true);
          }
      /// Invalidating edges
      if(!currentCBSNode->m_invalidEdgesAt.empty())
        if(!currentCBSNode->m_invalidEdgesAt[num_robot].empty())
          for(size_t i =0 ; i < currentCBSNode->m_invalidEdgesAt[num_robot].size() ; ++i) {
            std::cout << "Pre-Invalidating edge (" << currentCBSNode->m_invalidEdgesAt[num_robot][i].first.first << "," << currentCBSNode->m_invalidEdgesAt[num_robot][i].first.second << ")" << std::endl;
            std::cout  << "Size():  " << currentCBSNode->m_invalidEdgesAt[num_robot].size() << std::endl;
            //roadmap->SetEdgeInvalidatedAt(currentCBSNode->m_invalidEdgesAt[num_robot][i].first.first, currentCBSNode->m_invalidEdgesAt[num_robot][i].first.second, currentCBSNode->m_invalidEdgesAt[num_robot][i].second,true);
            //roadmap->SetEdgeInvalidated(currentCBSNode->m_invalidEdges[num_robot][i].first, currentCBSNode->m_invalidEdges[num_robot][i].second,true);
            //if(roadmap->IsEdge(currentCBSNode->m_invalidEdgesAt[num_robot][i].first.first, currentCBSNode->m_invalidEdgesAt[num_robot][i].first.second))
            //  roadmap->DeleteEdge(currentCBSNode->m_invalidEdgesAt[num_robot][i].first.first, currentCBSNode->m_invalidEdgesAt[num_robot][i].first.second);
            //currentCBSNode->m_invalidEdgesAt[num_robot].clear();
          }

      if(!currentCBSNode->m_conflictCfgsAt.empty())
        if(!currentCBSNode->m_conflictCfgsAt[num_robot].empty())
          for(size_t i = 0; i < currentCBSNode->m_conflictCfgsAt[num_robot].size() ; ++i){
            std::cout << "++++++++++++++++++++++++++++++Pre-Invalidating a Roadmap Region " << std::endl;
            roadmap->SetConflictCfgAt(currentCBSNode->m_conflictCfgsAt[num_robot][i].first, currentCBSNode->m_conflictCfgsAt[num_robot][i].second, true);
          }

      ++num_robot;
      // this->GetMPLibrary()->SetTask(nullptr);
      success = true;
      this->GetMPLibrary()->SetTask(&task);
      {
      MethodTimer mt(this->GetStatClass(), "GroupCBSQuery::Replanning indivual paths");  
      success &= (*query)();
      }
      roadmap->ClearInvalidated();
      roadmap->ClearInvalidatedAt();
      roadmap->ClearConflictCfgsAt();
      // Running individual query for the robot in task
      if(success) {
        auto path = this->GetPath(robot);
        std::cout << "VID Path for robot " << robot->GetLabel() << ": " << path->VIDs() << std::endl;

      } else {
          std::cout << "???????????????? BREAKING THE FOR LOOP BECAUSE A PATH FAILED" <<  std::endl;
          break;
      }
      /// If we fail in finding one path, we stop analyzing the current CBS node
      // if(!success)
      //   break;
      // }

    }

    /// If the current CBS node failed, we break this iteration and analyze the next CBS node
    ++counter;
     if(!success) {
      std::cout << "!!!!!!!!!!!!! BREAKING THE LOOP BECAUSE THE CURRENT CBS NODE FAILED" <<  std::endl;
      continue;
     }
    /// Here we will reserve the size for the invalidVertices and invalidEdges
    if(currentCBSNode->m_invalidVerticesAt.empty() && currentCBSNode->m_invalidEdgesAt.empty()) {
      //std::cout << "^^^^^^^^^^^^^^^^^^^^^6 RESERVING SIZE" <<  std::endl;
      for(size_t i = 0 ; i < num_robot ; ++i) {
        invalidVertices.clear();
        invalidEdges.clear();
        currentCBSNode->m_invalidVertices.push_back(invalidVertices);
        currentCBSNode->m_invalidEdges.push_back(invalidEdges);

        invalidVerticesAt.clear();
        invalidEdgesAt.clear();
        currentCBSNode->m_invalidVerticesAt.push_back(invalidVerticesAt);
        currentCBSNode->m_invalidEdgesAt.push_back(invalidEdgesAt);

        conflictCfgsAt.clear();

        currentCBSNode->m_conflictCfgsAt.push_back(conflictCfgsAt);

      }
    }
    /// Getting total cost for feeding the m_cost of the CBS NOde
    for(auto& task : *groupTask) {
      auto robot = task.GetRobot();
      auto path = this->GetPath(robot);
      cost = cost + path->Length();
      paths.push_back(path);
      std::cout << "\n\nVID Path for robot " << robot->GetLabel() << ": " << path->VIDs() << std::endl;
    }
    // Pairwisely, we iterate over the set of paths to look for conflicts
    // @TODO: We should reimplement FindConflict(), since it need two rounds per each pair of paths,
    // We need a new function to avoid this "double work".

    // auto pairCfg = siTool->FindConflict(paths);

    // if( static_cast<int>(pairCfg.first.first) != -1) {
    //   auto newCBSNode1 = new CBSNode<MPTraits>;
    //   newCBSNode1->m_conflictCfgsAt = currentCBSNode->m_conflictCfgsAt;
    //   newCBSNode1->m_conflictCfgsAt[pairCfg.first.first].push_back(make_pair(pairCfg.first.second.first,pairCfg.first.second.second));
    //   newCBSNode1->m_cost = cost;
    //   cbsTree.Insert(newCBSNode1);

    //   // auto newCBSNode2 = new CBSNode<MPTraits>;
    //   // newCBSNode2->m_conflictCfgsAt = currentCBSNode->m_conflictCfgsAt;
    //   // newCBSNode2->m_conflictCfgsAt[pairCfg.second.first].push_back(make_pair(pairCfg.second.second.first,pairCfg.second.second.second));
    //   // newCBSNode2->m_cost = cost;
    //   // cbsTree.Insert(newCBSNode2);

    //   success = false;

    // }




    bool conflictFound = false;
    for(size_t i = 0 ; i < paths.size() ; ++i) {
      for(size_t j = 0; j < paths.size() ; j++) {
        MethodTimer mt(this->GetStatClass(), "GroupCBSQuery::Pairwised paths checking ");
        if(i == j) continue;
        //std::cout <<  "Robot: "  << i << " , Obstacle: " << j << std::endl;
        //std::cout <<  "Robot path length: "  << paths[i]->Length() << " , Obstacle path length: " << paths[j]->Length() << std::endl;
        
        //MethodTimer mt(this->GetStatClass(), "GroupCBSQuery::FindConflict");  
        Conflict<MPTraits> conflict = siTool->FindConflict(paths[i],paths[j]);
        
        if(conflict.emptyConflict) {
          //std::cout << "Empty conflict!" << std::endl;
        }
        else {
          //std::cout << "Conflict found!" << std::endl;
          auto newCBSNode = new CBSNode<MPTraits>;
          //newCBSNode->m_invalidVertices = currentCBSNode->m_invalidVertices;
          //newCBSNode->m_invalidEdges = currentCBSNode->m_invalidEdges;
          newCBSNode->m_invalidVerticesAt = currentCBSNode->m_invalidVerticesAt;
          newCBSNode->m_invalidEdgesAt = currentCBSNode->m_invalidEdgesAt;
          newCBSNode->m_conflictCfgsAt = currentCBSNode->m_conflictCfgsAt;
          //if(conflict.t1 == Conflict<MPTraits>::Type::Vertex)
            //newCBSNode->m_invalidVertices[i].push_back(conflict.id1);
            //newCBSNode->m_invalidVerticesAt[i].push_back(make_pair(conflict.id1,conflict.conflictTimestep));
          //if(conflict.t1 == Conflict<MPTraits>::Type::Edge)
            //newCBSNode->m_invalidEdgesAt[i].push_back(make_pair(make_pair(conflict.id1,conflict.id2),conflict.conflictTimestep));
            //newCBSNode->m_invalidEdges[i].push_back(make_pair(conflict.id1,conflict.id2));
          //if(conflict.conflictCfg.GetRobot()) {
          std::cout << "Conflicting Cfg Found!: " <<  conflict.conflictCfg.PrettyPrint() << " at timestep " << conflict.conflictTimestep << std::endl;
          newCBSNode->m_conflictCfgsAt[i].push_back(make_pair(conflict.conflictCfg,conflict.conflictTimestep));
          //}

          newCBSNode->m_cost = cost;

          cbsTree.Insert(newCBSNode);

          //std::cout << "CBSNode has " << newCBSNode->m_invalidEdges.size() << " conflicts" << std::endl;

          newCBSNode->PrintInvalidEdges();

          success = false;

          conflictFound = true;

          break;

        }
        if(conflictFound)
          break;
      }
    }
    std::cout << "CBSTree  has " << cbsTree.Length() << " nodes " << std::endl;

    paths.clear();
    if(success) {
      std::cout << "\n expanded-nodes: " << counter+1 << "\n" << std::endl;
      std::cout << "total-cost is  " << cost << std::endl;

      break;
    }
  } while (!cbsTree.Empty() );

  // for(auto& task : *groupTask) {
  //     /// Clearing invalidations
  //     auto robot = task.GetRobot();
  //     auto roadmap = this->GetRoadmap(robot);
  //     roadmap->ClearInvalidated();
  // }

  // for(auto& task : *groupTask) {
  //   auto robot = task.GetRobot();

  //   if(this->m_debug)
  //     std::cout << "\tQuerying path for robot '" << robot->GetLabel()
  //               << "', task '" << task.GetLabel() << "'."
  //               << std::endl;

  //   // Evaluate this task.
  //   this->GetMPLibrary()->SetTask(&task);
  //   success &= (*query)();
  //   if(!success)
  //     break;

  //       /// this parte is commented since the conflict-checking should be pairwised checked
  //       // // Success: add this robot/path as a dynamic obstacle for the remaining
  //       // // robots.
  //       // this->GetMPProblem()->AddDynamicObstacle(
  //       //     DynamicObstacle(robot, this->GetPath(robot)->FullCfgs(
  //       //         this->GetMPLibrary()))
  //       // );
  // }

  if(this->m_debug)
    std::cout << "\tDone." << std::endl;

  // Restore the group task.
  this->GetMPLibrary()->SetTask(nullptr);
  this->GetMPLibrary()->SetGroupTask(groupTask);
  this->GetMPProblem()->ClearDynamicObstacles();

  return success;
}

/*----------------------------------------------------------------------------*/

#endif
