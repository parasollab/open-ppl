#ifndef MP_TASK_TYPE_H_
#define MP_TASK_TYPE_H_

#include <vector>

#include "Utilities/PMPLExceptions.h"

class Constraint {}; /// @TODO Replace stand-in with real thing.
class RobotGroup;


////////////////////////////////////////////////////////////////////////////////
/// Describes a motion task with start/end conditions and constraints on
/// allowable trajectories.
///
/// @details Tasks are defined by a robot group and three sets of constraints:
///   @arg Start constraints describe the conditions required to start a task.
///   @arg Path constraints describe restrictions that must be observed
///        throughout a valid solution.
///   @arg Goal constraints describe the conditions that must be met to complete
///        a task.
/// The robot group is the group assigned to perform the task.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPTaskType {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::PathType Path;

    ///@}
    ///@name Local Types
    ///@{

    enum Status {OnDeck, InProgress, Complete};

    ///@}
    ///@name Constraint Accessors
    ///@{

    void AddStartConstraint(const Constraint& _c);
    void AddPathConstraint(const Constraint& _c);
    void AddGoalConstraint(const Constraint& _c);

    const std::vector<Constraint>& GetStartConstraints() const noexcept;
    const std::vector<Constraint>& GetPathConstraints() const noexcept;
    const std::vector<Constraint>& GetGoalConstraints() const noexcept;

    ///@}
    ///@name Constraint Evaluation
    ///@{

    /// Evaluate a path to see if it meets the constraints.
    Status Evaluate(const Path& _p);

  private:

    bool EvaluateStartConstraints(const Path& _p);
    bool EvaluatePathConstraints(const Path& _p);
    bool EvaluateGoalConstraints(const Path& _p);

    ///@}
    ///@name Internal State
    ///@{

    Status m_status{OnDeck};      ///< The status of this task.

    RobotGroup* m_group{nullptr}; ///< The robot group assigned to this task.

    std::vector<Constraint> m_startConstraints; ///< Req'd to start task.
    std::vector<Constraint> m_pathConstraints;  ///< Req'd during whole task.
    std::vector<Constraint> m_goalConstraints;  ///< Req'd to end task.

    ///@}

};

/*-------------------------- Constraint Accessors ----------------------------*/

template <typename MPTraits>
void
MPTaskType<MPTraits>::
AddStartConstraint(const Constraint& _c) {
  m_startConstraints.push_back(_c);
}


template <typename MPTraits>
void
MPTaskType<MPTraits>::
AddPathConstraint(const Constraint& _c) {
  m_pathConstraints.push_back(_c);
}


template <typename MPTraits>
void
MPTaskType<MPTraits>::
AddGoalConstraint(const Constraint& _c) {
  m_goalConstraints.push_back(_c);
}


template <typename MPTraits>
const std::vector<Constraint>&
MPTaskType<MPTraits>::
GetStartConstraints() const noexcept {
  return m_startConstraints;
}


template <typename MPTraits>
const std::vector<Constraint>&
MPTaskType<MPTraits>::
GetPathConstraints() const noexcept {
  return m_pathConstraints;
}


template <typename MPTraits>
const std::vector<Constraint>&
MPTaskType<MPTraits>::
GetGoalConstraints() const noexcept {
  return m_goalConstraints;
}

/*---------------------------- Constraint Evaluation -------------------------*/

template <typename MPTraits>
typename MPTaskType<MPTraits>::Status
MPTaskType<MPTraits>::
Evaluate(const Path& _p) {
  switch(m_status) {
    case OnDeck:
      // If start constraints are satisfied, mark task as in progress.
      /// @TODO Should we require path constraints here?
      if(EvaluateStartConstraints(_p))
        m_status = InProgress;
      else
        break;
    case InProgress:
      // If path and end constraints are satisfied, mark task as finished.
      if(EvaluatePathConstraints(_p) && EvaluateGoalConstraints(_p))
        m_status = Complete;
      else
        break;
    case Complete:
      break;
    default:
      throw RunTimeException(WHERE, "Unrecognized task status. Options are: "
          "OnDeck, InProgress, Finished.");
  }
  return m_status;
}


template <typename MPTraits>
bool
MPTaskType<MPTraits>::
EvaluateStartConstraints(const Path& _p) {
  /// @TODO Check each constraint against _p.
  return false;
}


template <typename MPTraits>
bool
MPTaskType<MPTraits>::
EvaluatePathConstraints(const Path& _p) {
  /// @TODO Check each constraint against _p.
  return false;
}


template <typename MPTraits>
bool
MPTaskType<MPTraits>::
EvaluateGoalConstraints(const Path& _p) {
  /// @TODO Check each constraint against _p.
  return false;
}

/*----------------------------------------------------------------------------*/

#endif
