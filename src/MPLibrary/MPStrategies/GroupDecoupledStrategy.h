#ifndef PMPL_GROUP_DECOUPLED_STRATEGY_H_
#define PMPL_GROUP_DECOUPLED_STRATEGY_H_

#include "GroupStrategyMethod.h"
#include "MPProblem/GroupTask.h"


////////////////////////////////////////////////////////////////////////////////
/// Runs a single-robot planner for each robot in the group, and manages
/// inter-robot conflicts at query time.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupDecoupledStrategy : public GroupStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::WeightType       WeightType;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::GroupWeightType  GroupWeightType;
    typedef typename RoadmapType::VID           VID;

    ///@}
    ///@name Construction
    ///@{

    GroupDecoupledStrategy();

    GroupDecoupledStrategy(XMLNode& _node);

    virtual ~GroupDecoupledStrategy() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Iterate() override;

    virtual void Finalize() override;

    ///@}

  private:

    ///@name
    ///@{

    std::string m_strategyLabel; ///< The individual strategy label.

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
GroupDecoupledStrategy<MPTraits>::
GroupDecoupledStrategy() : GroupStrategyMethod<MPTraits>() {
  this->SetName("GroupDecoupledStrategy");
}


template <typename MPTraits>
GroupDecoupledStrategy<MPTraits>::
GroupDecoupledStrategy(XMLNode& _node) : GroupStrategyMethod<MPTraits>(_node) {
  this->SetName("GroupDecoupledStrategy");
  m_strategyLabel = _node.Read("strategyLabel", true, "",
      "The indiviudal strategy to run for each robot.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
GroupDecoupledStrategy<MPTraits>::
Print(std::ostream& _os) const {
  GroupStrategyMethod<MPTraits>::Print(_os);
  _os << "\tstrategyLabel: " << m_strategyLabel
      << std::endl;
}


template <typename MPTraits>
void
GroupDecoupledStrategy<MPTraits>::
Initialize() {
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
GroupDecoupledStrategy<MPTraits>::
Iterate() {
  // For each individual task t in group task
  // - Set the individual task
  // - run the designated single-robot planner
  // Clear the individual task
  //auto s = this->GetMPStrategy(m_strategyLabel);

  // Save the group task and unset it.
  auto groupTask = this->GetGroupTask();
  this->GetMPLibrary()->SetGroupTask(nullptr);

  // Set the library to this individual task and initialize.
  for(auto& task : *groupTask) {
    auto robot = task.GetRobot();
    std::cout << "Running indiviudal strategy on robot " 
      << robot->GetLabel() << std::endl;
    auto s = this->GetMPStrategy(m_strategyLabel);
    if(robot->GetDefaultStrategyLabel() != "")
      s = this->GetMPStrategy(robot->GetDefaultStrategyLabel());
    this->GetMPLibrary()->SetTask(&task);

    (*s)();
  }

  // Restore the group task.
  this->GetMPLibrary()->SetTask(nullptr);
  this->GetMPLibrary()->SetGroupTask(groupTask);
}


template <typename MPTraits>
void
GroupDecoupledStrategy<MPTraits>::
Finalize() {

  double totalCost = 0;
  auto groupTask = this->GetGroupTask();
  auto group = groupTask->GetRobotGroup();
  auto groupRoadmap = this->GetGroupRoadmap();

  // Collect the full cfg paths for each robot.
  size_t longestPath = 0;
  //std::vector<std::vector<CfgType>> paths;
  std::vector<std::vector<VID>> paths;
  paths.reserve(groupTask->Size());
  size_t i = 0;
  for(auto& task : *groupTask) {
    auto robot = task.GetRobot();
    auto path = this->GetPath(robot);

    // If the path is empty, we didn't solve the problem.
    if(path->Empty())
      return;
    totalCost += path->Length();
    // Collect the path.
    paths.push_back(path->VIDs());
    longestPath = std::max(longestPath, paths.back().size());

    std::cout << "VID Path for robot " << robot->GetLabel() 
      << ": " << path->VIDs()
              << std::endl;

    if(path and path->Size()) {
      const std::string base = this->GetBaseFilename();
      ::WritePath(base +"."+ robot->GetLabel() + ".rdmp.path", path->Cfgs());
      ::WritePath(base +"."+ robot->GetLabel()  + ".path",
        path->FullCfgs(this->GetMPLibrary()));
      auto roadmap = path->GetRoadmap();
      roadmap->Write(base +"."+ robot->GetLabel() + ".map", 
        this->GetEnvironment());
    }
    ++i;
  }

  StatClass* stats = this->GetStatClass();
  stats->SetStat("GroupDecoupledQuery::TotalCost",totalCost);

  // Add each path configuration to the group roadmap.
  const size_t numRobots = groupTask->Size();
  size_t lastVID = INVALID_VID;
  for(size_t i = 0; i < longestPath; ++i) {
    std::cout << "Creating group path vertex " << i << std::endl;

    // Add the next node to the group map.
    GroupCfg cfg(groupRoadmap);
    for(size_t j = 0; j < numRobots; ++j) {
      const VID vid = i >= paths[j].size() ? paths[j].back() : paths[j][i];
      std::cout << "\tSetting robot " << j << "'s cfg to VID " << vid << "."
                << std::endl;
      cfg.SetRobotCfg(j, vid);
    }
    const VID vid = groupRoadmap->AddVertex(cfg);

    std::cout << "Created group VID " << vid << "." << std::endl;

    // If the last VID was valid, add an edge between the nodes.
    if(lastVID != INVALID_VID) {
      std::cout << "Creating group edge from " << lastVID 
        << " to " << vid << "."
                << std::endl;
      GroupWeightType lp(groupRoadmap);

      for(size_t j = 0; j < numRobots; ++j) {
        auto robot = group->GetRobot(j);
        auto roadmap = this->GetRoadmap(robot);
        typename RoadmapType::EI iter;
        const VID prevVID = i - 1 >= paths[j].size() ? paths[j].back()
                                                     : paths[j][i - 1],
                  thisVID = i >= paths[j].size() ? paths[j].back()
                                                 : paths[j][i];

        // This may be a self-edge if this robot's path isn't the longest one.
        // Check for and add it if necessary.
        if(prevVID == thisVID)
          roadmap->AddEdge(prevVID, thisVID, WeightType("", 0));

        roadmap->GetEdge(prevVID, thisVID, iter);

        std::cout << "\tSetting robot " << j << "'s edge to ("
                  << iter->descriptor().source() << ", "
                  << iter->descriptor().target() << ")."
                  << std::endl;

        lp.SetEdge(robot, iter->descriptor());
        lp.SetWeight(std::max(lp.GetWeight(), iter->property().GetWeight()));
      }

      groupRoadmap->AddEdge(lastVID, vid, lp);
    }
    lastVID = vid;
  }

  GroupStrategyMethod<MPTraits>::Finalize();
}

/*----------------------------------------------------------------------------*/

#endif
