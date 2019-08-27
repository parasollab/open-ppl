#ifndef PMPL_GROUP_DECOUPLED_QUERY_H_
#define PMPL_GROUP_DECOUPLED_QUERY_H_

#include "MapEvaluatorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Calls an individual query for each robot in the group to realize cooperative
/// A*. After each plan is extracted, that robot is treated as a dynamic
/// obstacle for the remaining robots.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupDecoupledQuery : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    GroupDecoupledQuery();

    GroupDecoupledQuery(XMLNode& _node);

    virtual ~GroupDecoupledQuery() = default;

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
GroupDecoupledQuery<MPTraits>::
GroupDecoupledQuery() {
  this->SetName("GroupDecoupledQuery");
}


template <typename MPTraits>
GroupDecoupledQuery<MPTraits>::
GroupDecoupledQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("GroupDecoupledQuery");

  m_queryLabel = _node.Read("queryLabel", true, "",
      "The individual query method.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
GroupDecoupledQuery<MPTraits>::
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
GroupDecoupledQuery<MPTraits>::
operator()() {
  // For each individual task t in group task:
  // - Set individual task
  // - Run query
  // - create dynamic obstacle from path
  // Clear individual task
  // Clear dynamic obstacles
  auto groupTask = this->GetGroupTask();
  auto group = groupTask->GetRobotGroup();
  if(this->m_debug)
    std::cout << "Running decoupled query for robot group '"
              << group->GetLabel() << "'."
              << std::endl;

  bool success = true;
  auto query = this->GetMapEvaluator(m_queryLabel);

  // Unset the group task.
  this->GetMPLibrary()->SetGroupTask(nullptr);

  for(auto& task : *groupTask) {
    auto robot = task.GetRobot();

    if(this->m_debug)
      std::cout << "\tQuerying path for robot '" << robot->GetLabel()
                << "', task '" << task.GetLabel() << "'."
                << std::endl;

    // Evaluate this task.
    this->GetMPLibrary()->SetTask(&task);
    success &= (*query)();
    if(!success)
      break;

    // Success: add this robot/path as a dynamic obstacle for the remaining
    // robots.

    this->GetMPProblem()->AddDynamicObstacle(
        DynamicObstacle(robot, this->GetPath(robot)->FullCfgs(this->GetMPLibrary()))
          );

  }

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
