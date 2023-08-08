#ifndef PMPL_M_STAR_H_
#define PMPL_M_STAR_H_

#include "GroupStrategyMethod.h"
#include "MPProblem/GroupTask.h"


////////////////////////////////////////////////////////////////////////////////
/// Follows the optimal policy for each individual robot until collisions
/// are encountered. Then, considers more of the composite space as needed.
///
/// Reference:
///   G. Wagner and H. Choset, "M*: A complete multirobot path planning 
///   algorithm with performance bounds," 2011 IEEE/RSJ International 
///   Conference on Intelligent Robots and Systems, San Francisco, CA, USA, 
///   2011, pp. 3260-3267, doi: 10.1109/IROS.2011.6095022.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MStar : public GroupStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::GroupCfgType     GroupCfgType;
    typedef typename MPTraits::WeightType       WeightType;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::GroupWeightType  GroupWeightType;
    typedef typename RoadmapType::VID           VID;

    ///@}
    ///@name Construction
    ///@{

    MStar();

    MStar(XMLNode& _node);

    virtual ~MStar() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Iterate() override;

    ///@}

  private:

    ///@name
    ///@{

    std::string m_strategyLabel; ///< The individual strategy label.

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
MStar<MPTraits>::
MStar() : GroupStrategyMethod<MPTraits>() {
  this->SetName("MStar");
}


template <typename MPTraits>
MStar<MPTraits>::
MStar(XMLNode& _node) : GroupStrategyMethod<MPTraits>(_node) {
  this->SetName("MStar");
  m_strategyLabel = _node.Read("strategyLabel", true, "",
      "The indiviudal strategy to run for each robot.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
MStar<MPTraits>::
Print(std::ostream& _os) const {
  GroupStrategyMethod<MPTraits>::Print(_os);
  _os << "\tstrategyLabel: " << m_strategyLabel
      << std::endl;
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
MStar<MPTraits>::
Iterate() {
  // For each individual task t in group task
  // - Set the individual task
  // - run the designated single-robot planner
  // Clear the individual task

  // Save the group task and unset it.
  auto groupTask = this->GetGroupTask();
  this->GetMPLibrary()->SetGroupTask(nullptr);

  // Make all of the robots virtual so they don't interfere with each other
  for(auto robot : groupTask->GetRobotGroup()->GetRobots()) {
    robot->SetVirtual(true);
  }

  // Set the library to this individual task and execute.
  for(auto& task : *groupTask) {
    auto robot = task.GetRobot();
    robot->SetVirtual(false);
    const std::string strategy = robot->GetDefaultStrategyLabel().empty()
                               ? m_strategyLabel
                               : robot->GetDefaultStrategyLabel();

    if(this->m_debug)
      std::cout << "Running indiviudal strategy '" << strategy << "' for "
                << "robot '" << robot->GetLabel() << "' (" << robot << ")."
                << std::endl;

    this->GetMPLibrary()->SetTask(&task);
    auto s = this->GetMPStrategy(strategy);
    s->EnableOutputFiles(false);
    (*s)();
    s->EnableOutputFiles(true);
    robot->SetVirtual(true);
  }

  // Restore the group task.
  this->GetMPLibrary()->SetTask(nullptr);
  this->GetMPLibrary()->SetGroupTask(groupTask);

  for(auto robot : groupTask->GetRobotGroup()->GetRobots()) {
    robot->SetVirtual(false);
  }
}

/*----------------------------------------------------------------------------*/

#endif
