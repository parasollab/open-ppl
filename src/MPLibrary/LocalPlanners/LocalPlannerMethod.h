#ifndef PMPL_LOCAL_PLANNER_METHOD_H_
#define PMPL_LOCAL_PLANNER_METHOD_H_

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/GroupLPOutput.h"
#include "MPLibrary/MPBaseObject.h"
#include "MPProblem/Environment/Environment.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref LocalPlanners.
///
/// LocalPlannerMethod has two main functions: @c IsConnected and
/// @c ReconstructPath.
///
/// @c IsConnected takes as input two configurations \f$c_1\f$, \f$c_2\f$, an
/// LPOutput, validation resolutions, and optional booleans dictating whether to
/// check collision and save the path. The function both returns true or false
/// to validate the simple path between \f$c_1\f$ and \f$c_2\f$, but also
/// populates the LPOutput structure with useful information.
///
/// @c ReconstructPath is used to reconstruct a specific polygonal chain from a
/// WeightType object's intermediate configurations. The function takes as input
/// two configurations, a set of intermediate configurations, and validity
/// resolutions.
///
/// @todo All local planners need to use a distance metric to set their edge
///       weights properly; currently all of them are simply using the number of
///       intermediate steps as a weight.
/// @todo Local planners and Extenders represent the same concepts and should be
///       merged into a single class with both an Extend and LocalPlan function.
///       This will help simplify several other objects within PMPL as well,
///       such as bi-directional RRT.
///
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LocalPlannerMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::GroupCfgType  GroupCfgType;
    typedef typename GroupCfgType::Formation Formation;

    ///@}
    ///@name Construction
    ///@{

    LocalPlannerMethod(bool _saveIntermediates = false);

    LocalPlannerMethod(XMLNode& _node);

    virtual ~LocalPlannerMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Local Planner Interface
    ///@{

    /// Validate a simple path between two nodes.
    /// @param _start The starting configuration.
    /// @param _end The ending configuration.
    /// @param _col The witness configuration on failure.
    /// @param _lpOutput Weight and path computed from local plan.
    /// @param _posRes Positional DOF resolution.
    /// @param _oriRes Rotational DOF resolution.
    /// @param _checkCollision Use validity checking?
    /// @param _savePath Save all configurations along the path?
    /// @return A boolean indicating whether the connection succeeded.
    ///
    /// @usage
    /// @code
    /// LocalPlannerPointer lp = this->GetLocalPlanner(m_lpLabel);
    /// Environment* env = this->GetEnvironment();
    /// CfgType c1, c2, col;
    /// LPOutput<MPTraits> lpOut;
    /// lp->IsConnected(c1, c2, col, &lpOut, env->GetPositionRes(),
    ///     env->GetOrientationRes());
    /// @endcode
    virtual bool IsConnected(const CfgType& _start, const CfgType& _end,
        CfgType& _col, LPOutput<MPTraits>* _lpOutput, double _posRes,
        double _oriRes, bool _checkCollision = true, bool _savePath = false) = 0;

    /// This version does not return a collision node.
    /// @overload
    virtual bool IsConnected(const CfgType& _start, const CfgType& _end,
        LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false);

    /// This version is for group configurations.
    /// @overload
    virtual bool IsConnected(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _col, GroupLPOutput<MPTraits>* _lpOutput, double _posRes,
        double _oriRes, bool _checkCollision = true, bool _savePath = false,
        const Formation& _formation = Formation());

    /// This version for group configurations does not return a collision node.
    /// @overload
    virtual bool IsConnected(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupLPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false,
        const Formation& _formation = Formation());

    /// Reconstruct a previously computed simple path between two nodes
    /// @param _start Configuration 1
    /// @param _end Configuration 2
    /// @param _intermediates Intermediate configurations along simple path's
    ///        polygonal chain
    /// @param _posRes Positional DOF resolution
    /// @param _oriRes Rotational DOF resolution
    /// @return Configurations along path from _start to _end up to a resolution
    ///         (_posRes, _oriRes)
    ///
    /// @usage
    /// @code
    /// LocalPlannerPointer lp = this->GetLocalPlanner(m_lpLabel);
    /// Environment* env = this->GetEnvironment();
    /// CfgType c1, c2;
    /// std::vector<CfgType> intermediates;
    /// lp->ReconstructPath(c1, c2, intermediates, env->GetPositionRes(),
    ///     env->GetOrientationRes());
    /// @endcode
    virtual std::vector<CfgType> ReconstructPath(const CfgType& _start,
        const CfgType& _end, const std::vector<CfgType>& _intermediates,
        double _posRes, double _oriRes);


    /// GroupCfg overload:
    virtual std::vector<GroupCfgType> ReconstructPath(const GroupCfgType& _start,
        const GroupCfgType& _end, const std::vector<GroupCfgType>& _intermediates,
        double _posRes, double _oriRes,
        const Formation& _formation = Formation());

    ///@}

  protected:
    ///@name Helper
    //@{

    /// How to determine which robot to use when validating an edge.
    /// @param _start Configuration 1
    /// @param _end Configuration 2
    /// @return If start and end have the same robot, return this robot.
    ///         Otherwise return null.
    Robot* GetRobot(const CfgType& _start, const CfgType& _end) const noexcept;

    /// How to determine which robot to use when validating an edge.
    /// @param _start Configuration 1
    /// @param _end Configuration 2
    /// @return If start and end have the same robot group, return this robot
    ///         group. Otherwise return null.
    RobotGroup* GetRobotGroup(const GroupCfgType& _start, const GroupCfgType& _end) const noexcept;

    ///@}

    ///@name Internal State
    ///@{

    bool m_saveIntermediates{false}; ///< Save the intermediates in the roadmap?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LocalPlannerMethod<MPTraits>::
LocalPlannerMethod(const bool _saveIntermediates)
  : m_saveIntermediates(_saveIntermediates)
{ }


template <typename MPTraits>
LocalPlannerMethod<MPTraits>::
LocalPlannerMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  m_saveIntermediates = _node.Read("saveIntermediates", false,
      m_saveIntermediates, "Save intermediate nodes");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
LocalPlannerMethod<MPTraits>::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel()
      << "\n\tSave intermediates: " << m_saveIntermediates
      << std::endl;
}

/*------------------------ LocalPlanner Interface ----------------------------*/

template <typename MPTraits>
bool
LocalPlannerMethod<MPTraits>::
IsConnected(const CfgType& _start, const CfgType& _end,
    LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath) {
  CfgType col(this->GetTask()->GetRobot());
  return IsConnected(_start, _end, col, _lpOutput, _posRes,
      _oriRes, _checkCollision, _savePath);
}


template <typename MPTraits>
bool
LocalPlannerMethod<MPTraits>::
IsConnected(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupCfgType& _col,
    GroupLPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, const Formation& _formation) {
  throw NotImplementedException(WHERE) << "No default implementation provided.";
}


template <typename MPTraits>
bool
LocalPlannerMethod<MPTraits>::
IsConnected(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupLPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, const Formation& _formation) {
  GroupCfgType col(this->GetGroupRoadmap());
  return IsConnected(_start, _end, col, _lpOutput, _posRes,
                     _oriRes, _checkCollision, _savePath, _formation);
}


template <typename MPTraits>
std::vector<typename MPTraits::CfgType>
LocalPlannerMethod<MPTraits>::
ReconstructPath(const CfgType& _start, const CfgType& _end,
    const std::vector<CfgType>& _intermediates, double _posRes, double _oriRes) {
  LPOutput<MPTraits> lpOutput;
  IsConnected(_start, _end, &lpOutput, _posRes, _oriRes, false, true);
  return lpOutput.m_path;
}


template <typename MPTraits>
std::vector<typename MPTraits::GroupCfgType>
LocalPlannerMethod<MPTraits>::
ReconstructPath(const GroupCfgType& _start, const GroupCfgType& _end,
    const std::vector<GroupCfgType>& _intermediates, double _posRes,
    double _oriRes, const Formation& _formation) {
  GroupLPOutput<MPTraits> lpOutput(_start.GetGroupRoadmap());
  IsConnected(_start, _end, &lpOutput, _posRes, _oriRes, false, true, _formation);
  return lpOutput.m_path;
}

/*----------------------------------------------------------------------------*/
template <typename MPTraits>
Robot*
LocalPlannerMethod<MPTraits>::
GetRobot(const CfgType& _start, const CfgType& _end) const noexcept {
  auto robot1 = _start.GetRobot();
  auto robot2 = _end.GetRobot();
  if(robot1 == robot2)
    return robot1;
  return nullptr;
}

template <typename MPTraits>
RobotGroup*
LocalPlannerMethod<MPTraits>::
GetRobotGroup(const GroupCfgType& _start, const GroupCfgType& _end) const noexcept {
  auto group1 = _start.GetRobotGroup();
  auto group2 = _end.GetRobotGroup();
  if(group1 == group2)
    return group1;
  return nullptr;
}
#endif
