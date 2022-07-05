#ifndef PPL_GROUP_CFG_H_
#define PPL_GROUP_CFG_H_

#include <cstddef>
#include <iostream>
#include <vector>

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/CompositeState.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "nonstd.h"
#include "Transformation.h"
#include "Vector.h"

class Environment;
class Robot;
class RobotGroup;


////////////////////////////////////////////////////////////////////////////////
/// An aggregate configuration which represents a configuration for each robot
/// in a robot group.
///
/// The main point of group cfg is to take advantage of everything implemented
/// for individual robots. This means that we use VIDs from individual roadmaps
/// to keep track of the robot cfgs referred to in a group cfg. In the case that
/// a group cfg is modified or new in some way, there is a temporary local
/// storage (m_localCfgs) which stores individual cfgs not yet in a roadmap.
/// When adding a group cfg to a group roadmap, the VID is used in place after
/// adding the individual cfg to the individual roadmap.
/// 
/// 'GraphType' represents the individual roadmap type for a single robot.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
class GroupCfg final : public CompositeState<GraphType> {

  public:

    ///@name Local Types
    ///@{

    typedef CompositeState<GraphType>                         BaseType;
    typedef GroupRoadmap<GroupCfg, GroupLocalPlan<GraphType>> GroupRoadmapType;

    typedef typename BaseType::VID                            VID;
    typedef typename BaseType::GroupGraphType                 GroupGraphType;
    typedef typename BaseType::CfgType                        IndividualCfg; 

    /// A formation represents a group of robots which are maintaining their
    /// configurations relative to a leader, such as maintaining a square or
    /// V-shape while moving. The values are robot indexes (within the group,
    /// not problem) with the first index denoting the leader robot. These are
    /// not stored in configurations but may be required for edges.
    typedef std::vector<size_t> Formation;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a group configuration.
    /// @param _groupMap The group roadmap to which this configuration belongs,
    ///                  or null if it is not in a map.
    explicit GroupCfg(GroupRoadmapType* const& _groupMap = nullptr);

    /// Construct a group configuration.
    /// @param _group The group to which this configuration belongs.
    explicit GroupCfg(RobotGroup* const& _group);

    ///@}
    ///@name Equality
    ///@{

    /// Check if the current and given group configurations are equal.
    /// @param _other The given group configuration.
    /// @return True if equal, false otherwise.
    bool operator==(const GroupCfg& _other) const noexcept;

    /// Check if the current group cfg is less than the given group cfg.
    /// @param _other The given group configuration.
    /// @return True if less than, false otherwise.
    bool operator<(const GroupCfg& _other) const noexcept;

    ///@}
    ///@name Arithmetic
    ///@{

    /// Find the sum of the current and a given group configuration, 
    /// by each degree of freedom.
    /// @param _other The group configuration to be added. 
    /// @return The sum of the group configurations.
    GroupCfg operator+(const GroupCfg& _other) const;

    /// Find the difference of the current and a 
    /// given group configuration, by each degree of freedom.
    /// @param _other The group configuration to be subtracted. 
    /// @return The difference of the group configurations.
    GroupCfg operator-(const GroupCfg& _other) const;

    /// Find the product of the current and a given group configuration, 
    /// by each degree of freedom.
    /// @param _other The group configuration used to multiply the current. 
    /// @return The product of the group configurations.
    GroupCfg operator*(const double& _other) const;

    /// Add a given group configuration to the current, by each degree of freedom.
    /// @param _other The group configuration to be added.
    GroupCfg& operator+=(const GroupCfg& _other);

    /// Subtract a given group configuration from the current, by each degree of freedom.
    /// @param _other The group configuration to be subtracted.
    GroupCfg& operator-=(const GroupCfg& _other);

    /// Multiply the current group configuration by a scalar.
    /// @param _val The scalar used to multiply the current.
    GroupCfg& operator*=(const double& _val);

    ///@}
    ///@name Roadmap Accessors
    ///@{
    /// These functions provide access to the related group map (if any) and
    /// descriptors for non-local individual configurations.

    /// Get the group roadmap this group cfg is with respect to.
    GroupRoadmapType* GetGroupRoadmap() const noexcept;

    /// Change the roadmap that this group is using/in reference to. Also
    /// performs compatibility/verification tests to see if it's possible.
    /// @note Does NOT add this new cfg to any roadmap, but makes all cfg info
    ///       local (everything will be in m_localCfgs) so that there are no
    ///       issues when adding to the roadmap later.
    void SetGroupRoadmap(GroupRoadmapType* const _newRoadmap);

    ///@}
    ///@name DOF Accessors
    ///@{

    /// Check if there is a nonholonomic robot in the group.
    bool IsNonholonomic() const noexcept;

    /// Compute the total DOF for this robot group.
    size_t CompositeDOF() const;

    /// Compute the composite magnitude.
    double Magnitude() const;

    /// Compute the composite manitude of the positional components.
    double PositionMagnitude() const;

    /// Compute the composite magnitude of the orientation components.
    double OrientationMagnitude() const;

    ///@}
    ///@name Configuration Helpers
    ///@{

    /// Configure each individual cfg that this group cfg represents.
    /// This is for template cooperation and is also a needed function before
    /// dealing with CD calls and such.
    /// Note this will throw an exception if no cfg is present for any robot.
    void ConfigureRobot() const;

    /// Check that another GroupCfg is within a resolution as specified.
    /// @param _cfg The test configuration.
    /// @param _posRes The position resolution.
    /// @param _oriRes The orientation resolution.
    /// @return True if each individual configuration in this is within a
    ///         resolution distance of _cfg.
    bool WithinResolution(const GroupCfg& _cfg, const double _posRes,
        const double _oriRes) const;

    ///@}
    ///@name DOF Modifiers
    ///@{

    // Note: Using these functions will make this configuration utilize the
    // local cfgs, which won't be in group/individual roadmaps until added.


    /// Given this GroupCfg as the starting configuration, this function applies
    /// a rotation to all the robots that are listed, assuming the first one is
    /// the formation's leader.
    /// Note: Currently assumes all robots just have ONE body. The case of multi-
    /// bodied robots would need to be specially handled (right now it should just
    /// be split into multiple robots if a group is needed).
    /// @param _robotList This list of bodies to rotate. First one is leader body.
    /// @param _rotation The change in orientation that should be applied to _cfg.
    /// @param _debug A flag to print to cout (no access to an m_debug flag here).
    void RotateFormationAboutLeader(const Formation& _robotList,
                                    const mathtool::Orientation& _rotation,
                                    const bool _debug = false);

    /// Given this GroupCfg as the starting configuration, this function applies
    /// a transformation uniformly over all robots listed.
    /// Note: Currently assumes all robots just have ONE body. The case of multi-
    /// bodied robots would need to be specially handled (right now it should just
    /// be split into multiple robots if a group is needed).
    /// Note: This is assuming 6 DOFs!
    /// @param _robotList This list of bodies to rotate. First one is leader body.
    /// @param _transform The change in orientation that should be applied to _cfg.
    /// @param _relativeTransform (Optional) The transformation to "undo" before
    ///        applying _transform. If default, it will be a simple transform
    ///        application. See RotateFormationAboutLeader for usage example.
    void ApplyTransformationForRobots(const Formation& _robotList,
                             const mathtool::Transformation& _transform,
                             const mathtool::Transformation& _relativeTransform
                                                  = mathtool::Transformation());


    /// Given this configuration, add in the same DOF values to each body given.
    /// It assumes that all robots in the formation given have the same DOFs.
    /// This is a common thing to do in assembly planning/composite C-Spaces.
    /// @param _dofs The values to add in to each body. This function assumes each
    ///              body has #dofs = _dofs.size().
    /// @param _robots This list of bodies to update. Order doesn't matter.
    void AddDofsForRobots(const std::vector<double>& _dofs,
                          const Formation& _robots);


    /// This function adds all positional dofs in _dofs. It will handle 1-3 dofs
    /// based on each IndividualCfg's PosDof value. Does not add in orientation.
    /// Note: This function is slightly more efficient than the std::vector
    ///       version, as we do not need to check the size of _dofs.
    /// @param _dofs The positional values to add in to each body.
    /// @param _robots This list of bodies to update. Order doesn't matter.
    void AddDofsForRobots(const mathtool::Vector3d& _dofs,
                          const Formation& _robots);

    /// Given new DOF values, overwrite the existing values for each individual
    /// cfg in this group cfg that is listed in _robots. Note that _dofs needs
    /// to be the same number of DOFs as each individual cfg in the group.
    /// @param _fromCfg The configuration to take values from.
    /// @param _robots This list of bodies to update. Order doesn't matter.
    void OverwriteDofsForRobots(const std::vector<double>& _dofs,
                                const Formation& _robots);


    /// Given new DOF values, overwrite the existing values for each individual
    /// cfg in this group cfg that is listed in _robots. Note that _dofs needs
    /// to be the same number of DOFs as each individual cfg in the group.
    /// @param _fromCfg The configuration to take values from.
    /// @param _robots This list of bodies to update. Order doesn't matter.
    void OverwriteDofsForRobots(const mathtool::Vector3d& _dofs,
                                const Formation& _robots);


    /// Given this and another configuration, copy the DOF values from the other
    /// to the DOF values in this one, but only for each given robot indexed.
    /// @param _fromCfg The configuration to take values from.
    /// @param _robots This list of bodies to update. Order doesn't matter.
    void OverwriteDofsForRobots(const GroupCfg& _fromCfg,
                                const Formation& _robots);

    /// @overload to handle robot pointers.
    void OverwriteDofsForRobots(const GroupCfg& _fromCfg,
                                const std::vector<Robot*>& _robots);


    /// Overwrites all data in this cfg, assumes the length of _dofs is the same
    /// as CompositeDOF(). Basically just converts a composite C-Space vector
    /// into its individual pieces for a Group Cfg.
    void SetData(const std::vector<double>& _dofs);

    /// Find the c-space increment and number of steps needed to move from a
    /// start to a goal, taking steps no larger than the designated resolutions.
    /// @param _start The start configuration.
    /// @param _goal The goal configuration.
    /// @param _nTicks The number of steps to take (NOT computed by this method)
    void FindIncrement(const GroupCfg& _start, const GroupCfg& _goal,
        const int _nTicks);

    /// Find the c-space increment and number of steps needed to move from a
    /// start to a goal, taking steps no larger than the designated resolutions.
    /// @param _start The start configuration.
    /// @param _goal The goal configuration.
    /// @param _nTicks The number of steps to take (computed by this method).
    /// @param _positionRes The position resolution to use.
    /// @param _orientationRes The orientation resolution to use.
    void FindIncrement(const GroupCfg& _start, const GroupCfg& _goal,
        int* const _nTicks, const double _positionRes,
        const double _orientationRes);

    /// Test if a group configuration lies within a boundary and also within the
    /// robot's c-space limits.
    /// @param _boundary The boundary to check.
    /// @return True if the configuration places the robot inside both the
    ///         boundary and its DOF limits.
    bool InBounds(const Boundary* const _b) const noexcept;
    /// @overload
    bool InBounds(const Environment* const _env) const noexcept;

    /// Create a group configuration where every vertex of every robots
    /// is guaranteed to lie within the specified boundary. If a group cfg
    /// cannot be found, the program will abort. The function will try a
    /// predefined number of times.
    /// @param _b The bondary to sample within.
    void GetRandomGroupCfg(const Boundary* const _b);

    /// Create a group cfg where all robots are guaranteed to lie within
    /// the input environment.
    /// @param _env The environment to sample within.
    void GetRandomGroupCfg(Environment* _env);

    /// Normalize Orientation DOFs for a Group Cfg
    virtual void NormalizeOrientation(const std::vector<size_t>& _robots = {})
        noexcept;

    ///@}
    ///@name Output Utilities
    ///@{

    std::string PrettyPrint(const size_t _precision = 4) const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Initialize the set of local configurations if not already done.
    virtual void InitializeLocalCfgs() noexcept override;

    ///@}

};

template <typename GraphType>
GroupCfg<GraphType>::
GroupCfg(GroupRoadmapType* const& _groupMap) 
     : CompositeState<GraphType>((GroupGraphType*)_groupMap) {}

template <typename GraphType>
GroupCfg<GraphType>::
GroupCfg(RobotGroup* const& _group) : CompositeState<GraphType>(_group) {}

/*--------------------------------- Equality ---------------------------------*/

template <typename GraphType>
bool
GroupCfg<GraphType>::
operator==(const GroupCfg& _other) const noexcept {
  // If _other is for another group, these are not the same.
  if(this->m_group != _other.m_group)
    return false;

  // If _other is from another map, these are not the same.
  if(this->m_groupMap != _other.m_groupMap)
    return false;

  // Else, compare VIDs if both are valid, or by-value other wise.
  for(size_t i = 0; i < this->m_vids.size(); ++i) {
    const VID thisVID  = this->m_vids[i],
              otherVID = _other.m_vids[i];

    if(thisVID != INVALID_VID and otherVID != INVALID_VID) {
      if(thisVID != otherVID)
        return false;
    }
    else if(this->GetRobotCfg(i) != _other.GetRobotCfg(i))
      return false;
  }

  return true;
}


template <typename GraphType>
bool
GroupCfg<GraphType>::
operator<(const GroupCfg& _other) const noexcept {

  const auto& robots = this->GetRobots();

  for(size_t i = 0; i < robots.size(); i++) {
    const auto& cfg1 = this->GetRobotCfg(i);
    const auto& cfg2 = _other.GetRobotCfg(i);
    if(cfg1 < cfg2)
      return true;
    else if(cfg2 < cfg1)
      return false;
  }

  return false;
}
/*-------------------------------- Arithmetic --------------------------------*/

template <typename GraphType>
GroupCfg<GraphType>
GroupCfg<GraphType>::
operator+(const GroupCfg& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg += _other);
}


template <typename GraphType>
GroupCfg<GraphType>
GroupCfg<GraphType>::
operator-(const GroupCfg& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg -= _other);
}


template <typename GraphType>
GroupCfg<GraphType>
GroupCfg<GraphType>::
operator*(const double& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg *= _other);
}


template <typename GraphType>
GroupCfg<GraphType>&
GroupCfg<GraphType>::
operator+=(const GroupCfg& _other) {
  // We must require the exact same group, which indicates everything
  // lines up between the two cfgs (namely the exact robots/order of the group).
  if(this->m_group != _other.m_group)
    throw RunTimeException(WHERE, "Cannot add GroupCfgs with different groups!");

  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    this->SetRobotCfg(i, this->GetRobotCfg(i) + _other.GetRobotCfg(i));

  return *this;
}


template <typename GraphType>
GroupCfg<GraphType>&
GroupCfg<GraphType>::
operator-=(const GroupCfg& _other) {
  // We must require the exact same group, which indicates everything
  // lines up between the two cfgs (namely the exact robots/order of the group).
  if(this->m_group != _other.m_group)
    throw RunTimeException(WHERE, "Cannot subtract GroupCfgs with different groups!");

  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    this->SetRobotCfg(i, this->GetRobotCfg(i) - _other.GetRobotCfg(i));

  return *this;
}


template <typename GraphType>
GroupCfg<GraphType>&
GroupCfg<GraphType>::
operator*=(const double& _val) {
  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    this->SetRobotCfg(i, this->GetRobotCfg(i) * _val);

  return *this;
}

/*---------------------------- Roadmap Accessors -----------------------------*/

template <typename GraphType>
typename GroupCfg<GraphType>::GroupRoadmapType*
GroupCfg<GraphType>::
GetGroupRoadmap() const noexcept {
  return (GroupRoadmapType*)this->m_groupMap;
}


template <typename GraphType>
void
GroupCfg<GraphType>::
SetGroupRoadmap(GroupRoadmapType* const _newRoadmap) {
  // Check that groups are compatible.
  if(this->m_group != _newRoadmap->GetGroup())
    throw RunTimeException(WHERE) << "Trying to change roadmaps on incompatible "
                                  << "groups!";

  // Set the group graph of the composite state.
  this->m_groupMap = (GroupGraphType*) _newRoadmap;

  // Put all individual cfgs into the group cfg so that all are local:
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    this->SetRobotCfg(i, IndividualCfg(this->GetRobotCfg(i)));
}

/*------------------------------ DOF Accessors -------------------------------*/

template <typename GraphType>
bool
GroupCfg<GraphType>::
IsNonholonomic() const noexcept {
  for(auto robot : this->GetRobots())
    if(robot->IsNonholonomic())
      return true;
  return false;
}


template <typename GraphType>
size_t
GroupCfg<GraphType>::
CompositeDOF() const {
  size_t dofSum = 0;
  for(auto robot : this->GetRobots())
    dofSum += robot->GetMultiBody()->DOF();
  return dofSum;
}


template <typename GraphType>
double
GroupCfg<GraphType>::
Magnitude() const {
  double result = 0;
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    const double m = this->GetRobotCfg(i).Magnitude();
    result += m * m;
  }
  return std::sqrt(result);
}


template <typename GraphType>
double
GroupCfg<GraphType>::
PositionMagnitude() const {
  double result = 0;
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    const double m = this->GetRobotCfg(i).PositionMagnitude();
    result += m * m;
  }
  return std::sqrt(result);
}


template <typename GraphType>
double
GroupCfg<GraphType>::
OrientationMagnitude() const {
  double result = 0;
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    const double m = this->GetRobotCfg(i).OrientationMagnitude();
    result += m * m;
  }
  return std::sqrt(result);
}

/*------------------------- Configuration Helpers ----------------------------*/

template <typename GraphType>
void
GroupCfg<GraphType>::
ConfigureRobot() const {
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    this->GetRobotCfg(i).ConfigureRobot();
}


template <typename GraphType>
bool
GroupCfg<GraphType>::
WithinResolution(const GroupCfg& _cfg, const double _posRes,
    const double _oriRes) const {
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    if(!this->GetRobotCfg(i).WithinResolution(_cfg.GetRobotCfg(i), _posRes, _oriRes))
      return false;

  return true;
}

/*------------------------------DOF Modifiers---------------------------------*/

template <typename GraphType>
void
GroupCfg<GraphType>::
RotateFormationAboutLeader(const Formation& _robotList,
    const mathtool::Orientation& _rotation, const bool _debug) {
  /// Note: Currently assumes all robots just have ONE body. The case of multi-
  /// bodied robots would need to be specially handled (right now it should just
  /// be split into multiple robots if a group is needed).

  /// @todo We can probably compute this without having to configure the models
  ///       (which cost a lot of transformations).
  ConfigureRobot(); // Configure all individual cfgs.

  const size_t leaderIndex = _robotList[0];

  // Get transformation of leader before rotation:
  const IndividualCfg& leaderCfg = this->GetRobotCfg(leaderIndex);

  // TODO update this to handle multiple bodies per robot.
  // Use the multibody's body 0, since we assume each MB just has a single body.
  mathtool::Transformation initialLeaderTransform = leaderCfg.GetMultiBody()->
                                           GetBody(0)->GetWorldTransformation();

  const mathtool::Transformation rotation(mathtool::Vector3d(0,0,0), _rotation);

  if(_debug)
    std::cout << "Rotating bodies " << _robotList << " with rotation = "
              << rotation << std::endl;

  // The transform to be applied to all parts (including the first one). We
  // move the part to its relative world position with A at the world origin,
  // then the rotation is applied, and we return the part to its relative
  // position from A.
  const mathtool::Transformation transform = initialLeaderTransform * rotation;

  ApplyTransformationForRobots(_robotList, transform, initialLeaderTransform);
}


template <typename GraphType>
void
GroupCfg<GraphType>::
ApplyTransformationForRobots(const Formation& _robotList,
    const mathtool::Transformation& _transform,
    const mathtool::Transformation& _relativeTransform) {
  //Compute each robot's needed transformation and set dofs in cfg.
  for (const size_t robotIndex : _robotList) {
    const IndividualCfg& robotCfg = this->GetRobotCfg(robotIndex);

    /// @todo Generalize this to handle robots with more than one body.
    if(robotCfg.GetMultiBody()->GetNumBodies() > 1)
      throw RunTimeException(WHERE) << "Multiple bodies not supported!";

    // Retrieve current position and rotation of robot:
    const mathtool::Transformation& initialRobotTransform =
                  robotCfg.GetMultiBody()->GetBody(0)->GetWorldTransformation();

    // From right to left: apply the inverse relative transform to the initial
    // robot transform (puts the robot into the desired frame). Then apply
    // the transform given.
    const mathtool::Transformation newTransformation =   _transform *
                                                       (-_relativeTransform) *
                                                         initialRobotTransform;

    // Extract the transformation. Note: This is assuming 6 DOFs!
    const std::vector<double>& transformed = newTransformation.GetCfg();

    OverwriteDofsForRobots(transformed, {robotIndex});
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
AddDofsForRobots(const std::vector<double>& _dofs, const Formation& _robots) {
  for(const size_t robotIndex : _robots) {
    if(this->IsLocalCfg(robotIndex)) {
      // We can simply modify the local values, since it's not a roadmap cfg yet
      IndividualCfg& cfg = this->GetRobotCfg(robotIndex);

      // Ensure this robot has the correct number of DOF.
      if(_dofs.size() != cfg.DOF())
        throw RunTimeException(WHERE) << "Tried to add " << _dofs.size()
                                      << "dofs to robot " << robotIndex
                                      << ", which has " << cfg.DOF() << " DOFs.";

      // Update the robot's cfg.
      for(unsigned int i = 0; i < _dofs.size(); ++i)
        cfg[i] += _dofs[i];
    }
    else {
      // Must copy the cfg since it is not local.
      IndividualCfg cfg = this->GetRobotCfg(robotIndex);

      // Ensure this robot has the correct number of DOF.
      if(_dofs.size() != cfg.DOF())
        throw RunTimeException(WHERE) << "Tried to add " << _dofs.size()
                                      << "dofs to robot " << robotIndex
                                      << ", which has " << cfg.DOF() << " DOFs.";

      // Update the robot's cfg.
      for(unsigned int i = 0; i < _dofs.size(); ++i)
        cfg[i] += _dofs[i];
      this->SetRobotCfg(robotIndex, std::move(cfg));
    }
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
AddDofsForRobots(const mathtool::Vector3d& _dofs, const Formation& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg robotCfg = this->GetRobotCfg(robotIndex);
    for(size_t i = 0; i < robotCfg.PosDOF(); ++i)
      robotCfg[i] += _dofs[i];
    this->SetRobotCfg(robotIndex, std::move(robotCfg));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
OverwriteDofsForRobots(const std::vector<double>& _dofs,
    const Formation& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg newIndividualCfg(this->GetRobot(robotIndex));
    newIndividualCfg.SetData(_dofs);
    this->SetRobotCfg(robotIndex, std::move(newIndividualCfg));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
OverwriteDofsForRobots(const mathtool::Vector3d& _dofs,
    const Formation& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg newIndividualCfg(this->GetRobot(robotIndex));
    newIndividualCfg.SetLinearPosition(_dofs);
    this->SetRobotCfg(robotIndex, std::move(newIndividualCfg));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
OverwriteDofsForRobots(const GroupCfg& _fromCfg, const Formation& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg robotCfg = _fromCfg.GetRobotCfg(robotIndex);
    this->SetRobotCfg(robotIndex, std::move(robotCfg));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
OverwriteDofsForRobots(const GroupCfg& _fromCfg,
    const std::vector<Robot*>& _robots) {
  auto fromGroup = _fromCfg.GetGroupRoadmap()->GetGroup(),
       toGroup   = this->m_groupMap->GetGroup();

  for(Robot* const robot : _robots) {
    const size_t fromIndex = fromGroup->GetGroupIndex(robot),
                 toIndex   = toGroup->GetGroupIndex(robot);
    IndividualCfg robotCfg = _fromCfg.GetRobotCfg(fromIndex);
    this->SetRobotCfg(toIndex, std::move(robotCfg));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
SetData(const std::vector<double>& _dofs) {
  if(_dofs.size() != CompositeDOF())
    throw RunTimeException(WHERE) << "Tried to set " << _dofs.size()
                                  << " DOFs on a robot group with "
                                  << CompositeDOF() << " DOFs.";

  size_t compositeIndex = 0;
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    const size_t robotDof = this->GetRobot(i)->GetMultiBody()->DOF();
    IndividualCfg& robotCfg = this->GetRobotCfg(i);

    for(size_t i = 0; i < robotDof; ++i, ++compositeIndex)
      robotCfg[i] = _dofs[compositeIndex];
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
FindIncrement(const GroupCfg& _start, const GroupCfg& _goal, const int _nTicks) {
  // Need positive number of ticks.
  if(_nTicks <= 0)
    throw RunTimeException(WHERE) << "Divide by 0";
  if(_start.m_group != _goal.m_group)
    throw RunTimeException(WHERE) << "Cannot use two different groups "
                                  << "with this operation currently!";

  // For each robot in the group, find the increment for the individual cfg
  // given the number of ticks found.
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    IndividualCfg incr(this->GetRobot(i));
    incr.FindIncrement(_start.GetRobotCfg(i), _goal.GetRobotCfg(i), _nTicks);
    this->SetRobotCfg(i, std::move(incr));
  }
}


template <typename GraphType>
void
GroupCfg<GraphType>::
FindIncrement(const GroupCfg& _start, const GroupCfg& _goal, int* const _nTicks,
    const double _positionRes, const double _orientationRes) {
  const GroupCfg<GraphType> diff = _goal - _start;

  *_nTicks = std::max(1., std::ceil(std::max(
                      diff.PositionMagnitude() / _positionRes,
                      diff.OrientationMagnitude() / _orientationRes)));

  FindIncrement(_start, _goal, *_nTicks);
}


template <typename GraphType>
bool
GroupCfg<GraphType>::
InBounds(const Boundary* const _b) const noexcept {
  for(size_t i = 0; i < this->GetNumRobots(); ++i)
    if(!this->GetRobotCfg(i).InBounds(_b))
      return false;

  return true;
}


template <typename GraphType>
bool
GroupCfg<GraphType>::
InBounds(const Environment* const _env) const noexcept {
  return InBounds(_env->GetBoundary());
}

template <typename GraphType>
void
GroupCfg<GraphType>::
GetRandomGroupCfg(const Boundary* const _b) {
  std::set<Robot*> found;
  auto group = this->m_group;

  for(size_t i = 0; i < this->GetNumRobots(); i++) {
    this->m_vids[i] = INVALID_VID;
  }
  InitializeLocalCfgs();

  auto robots = group->GetRobots();
  for(size_t i = 0; i < robots.size(); i++) {
    auto robot = robots[i];

    // Check if robot cfg was set by formation sampling.
    if(found.count(robot))
      continue;

    // If not, get a random configuration for it.
    Cfg cfg(robot);
    cfg.GetRandomCfg(_b);

    // Save cfg to local cfgs.
    this->m_localCfgs[i] = cfg;
  }
}

template <typename GraphType>
void
GroupCfg<GraphType>::
GetRandomGroupCfg(Environment* _env) {
  GetRandomGroupCfg(_env->GetBoundary());
}

template <typename GraphType>
void
GroupCfg<GraphType>::
NormalizeOrientation(const std::vector<size_t>& _robots) noexcept {
  if(_robots.empty()) // Do all robots in this case.
    for(size_t i = 0; i < this->GetNumRobots(); ++i)
      this->GetRobotCfg(i).NormalizeOrientation();
  else
    for(size_t i : _robots)
      this->GetRobotCfg(i).NormalizeOrientation();
}

/*------------------------------ Output Helpers ------------------------------*/

template <typename GraphType>
std::string
GroupCfg<GraphType>::
PrettyPrint(const size_t _precision) const {
  std::ostringstream oss;
  oss.precision(_precision);
  oss << "{ ";
  for(size_t i = 0; i < this->GetNumRobots(); ++i) {
    const IndividualCfg& robotCfg = this->GetRobotCfg(i);
    if(this->IsLocalCfg(i))
      oss << "Local: ";
    oss << robotCfg.PrettyPrint(_precision) << ", ";
  }
  oss << " }";

  return oss.str();
}

/*----------------------------------------------------------------------------*/

template <typename GraphType>
void
GroupCfg<GraphType>::
InitializeLocalCfgs() noexcept {
  // We will assume the local cfgs are initialized if the container size is
  // correct.
  const size_t numRobots = this->GetNumRobots();
  if(this->m_localCfgs.size() == numRobots)
    return;

  this->m_localCfgs.clear();
  this->m_localCfgs.resize(numRobots);

  for(size_t i = 0; i < numRobots; ++i)
    this->m_localCfgs[i] = IndividualCfg(this->GetRobot(i));
}

/*----------------------------------------------------------------------------*/

template <typename GraphType>
std::ostream&
operator<<(std::ostream& _os, const GroupCfg<GraphType>& _groupCfg) {
  // Might not need to be hidden behind GROUP_MAP, but doing for consistency
#ifdef GROUP_MAP
  _os << "0 ";
#endif

  // Loop through all robots in the group and print each one's cfg in order.
  for(size_t i = 0; i < _groupCfg.GetNumRobots(); ++i)
    _os << _groupCfg.GetRobotCfg(i);

  return _os;
}

#endif
