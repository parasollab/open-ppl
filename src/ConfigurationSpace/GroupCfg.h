#ifndef GROUP_CFG_H_
#define GROUP_CFG_H_

#include <cstddef>
#include <iostream>
#include <vector>

#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/GroupLocalPlan.h"

#include "Transformation.h"
#include "Vector.h"

class Cfg;
class Robot;
class RobotGroup;


////////////////////////////////////////////////////////////////////////////////
/// An aggregate configuration which represents a configuration for each robot
/// in a robot group.
////////////////////////////////////////////////////////////////////////////////
class GroupCfg final {

  public:

    ///@name Local Types
    ///@{

    typedef size_t           VID;      ///< A VID in an individual Robot roadmap.
    typedef std::vector<VID> VIDSet;   ///< A set of VIDs from indiv. Robot roadmaps.

    typedef Cfg              IndividualCfg;
    typedef GroupRoadmap<GroupCfg, GroupLocalPlan<IndividualCfg>> GraphType;

    typedef std::vector<size_t> Formation;

    ///@}
    ///@name Construction
    ///@{

    /// If _init is true, this will construct a new GroupCfg by populating
    /// m_localCfgs with new default configurations.
    explicit GroupCfg(GraphType* const _groupMap = nullptr,
                      const bool _init = false);

    ///@}
    ///@name Equality
    ///@{

    bool operator==(const GroupCfg& _other) const noexcept;
    bool operator!=(const GroupCfg& _other) const noexcept;

    ///@}
    ///@name Addition and Subtraction
    ///@{

    GroupCfg operator+(const GroupCfg& _other) const;
    GroupCfg operator-(const GroupCfg& _other) const;
    GroupCfg operator*(const double& _other) const;

    GroupCfg& operator+=(const GroupCfg& _other);
    GroupCfg& operator-=(const GroupCfg& _other);

    /// Constant multiplication:
    GroupCfg& operator*=(const double& _val);


    ///@}
    ///@name Accessors
    ///@{

    /// Set the individual cfg for a robot to a local copy of an cfg.
    /// @param _robot The robot which the cfg refers to.
    /// @param _cfg The cfg.
    //    void SetCfg(Robot* const _robot, Cfg&& _cfg) explicit;
    void SetCfg(Robot* const _robot, Cfg&& _cfg);

    /// Set the individual cfg for a robot to a local copy of an cfg.
    /// @param _index The robot's group index which the cfg refers to.
    /// @param _cfg The cfg.
    void SetCfg(const size_t _index, Cfg&& _cfg);

    /// Set the individual cfg for a robot to a roadmap copy of an cfg.
    /// @param _robot The robot which the cfg refers to.
    /// @param _vid The cfg descriptor.
    void SetCfg(Robot* const _robot, const VID _vid);

    /// Set the individual cfg for a robot to a roadmap copy of an cfg.
    /// @param _index The robot's group index which the cfg refers to.
    /// @param _vid The cfg descriptor.
    void SetCfg(const size_t _index, const VID _vid);

    /// Clear the Local Cfg information in the cfg (for after adding to roadmap)
    void ClearLocalData();

    /// Get the individual Cfg for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual configuration for the indexed robot.
    Cfg& GetRobotCfg(const size_t _index);

    /// Get the individual Cfg for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual configuration for the indexed robot.
    const Cfg& GetRobotCfg(const size_t _index) const;

    Robot* GetRobot(const size_t _index) const;

    const VIDSet& GetVIDSet() const { return m_vids; }

    VID GetVID(const size_t _index) const noexcept;

    GraphType* GetGroupMap() const noexcept {return m_groupMap;}

    size_t GetNumRobots() const noexcept {return m_groupMap->GetGroup()->Size();}

    std::vector<Robot*> GetRobots() const noexcept
        { return m_groupMap->GetGroup()->GetRobots(); }

    /// We assume homogeneous robots right now, so the default argument just
    /// grabs the values for the first one.
    size_t PosDOF(const size_t _index = 0) const;
    size_t OriDOF(const size_t _index = 0) const;
    size_t DOF(const size_t _index = 0) const;

    /// TODO: This should ideally check if ANY robot is nonholonomic in the
    //        group. Right now it just checks the first.
    bool IsNonholonomic() const noexcept;

    /// Returns the self dot product of two group cfgs, summed over all robots,
    /// not caring about DOF types.
    double Magnitude() const;

    /// Returns the self dot product of two group cfgs, summed over all robots,
    /// but only for positional DOF types.
    double PositionMagnitude() const;

    /// Returns the self dot product of two group cfgs, summed over all robots,
    /// but only for non-positional DOF types.
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
    bool WithinResolution(const GroupCfg& _cfg, const double _posRes,
                          const double _oriRes) const;

    ///@}
    ///@name DOF Modifiers
    ///@{

    // Note: Using these functions will make this configuration utilize the
    // local cfgs, which won't be in group/individual roadmaps until added.


    /// Given this GroupCfg as the starting configuration, this function applies
    /// a rotation to all the other bodies in the robot that are listed in the
    /// body list, assuming the first one is the formation's leader.
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
    /// a transformation to all the other bodies in the robot that are listed in
    /// the robot list, applied uniformly over all robots.
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
    /// This is a common thing to do in assembly planning/composite C-Spaces.
    /// @param _dofs The values to add in to each body. This function assumes each
    ///              body has #dofs = _dofs.size().
    /// @param _bodies This list of bodies to update. Order doesn't matter.
    void AddDofsForRobots(const std::vector<double>& _dofs,
                          const Formation& _robots);


    /// This function adds all positional dofs in _dofs. It will handle 1-3 dofs
    /// based on each IndividualCfg's PosDof value. Does not add in orientation.
    /// Note: This function is slightly more efficient than the std::vector
    ///       version, as we do not need to check the size of _dofs.
    /// @param _dofs The positional values to add in to each body.
    /// @param _bodies This list of bodies to update. Order doesn't matter.
    void AddDofsForRobots(const mathtool::Vector3d& _dofs,
                          const Formation& _robots);

    /// Given new DOF values, overwrite the existing values for each individual
    /// cfg in this group cfg that is listed in _robots. Note that _dofs needs
    /// to be the same number of DOFs as each individual cfg in the group.
    /// @param _fromCfg The configuration to take values from.
    /// @param _bodies This list of bodies to update. Order doesn't matter.
    void OverwriteDofsForRobots(const std::vector<double>& _dofs,
                                const Formation& _robots);


    /// Given new DOF values, overwrite the existing values for each individual
    /// cfg in this group cfg that is listed in _robots. Note that _dofs needs
    /// to be the same number of DOFs as each individual cfg in the group.
    /// @param _fromCfg The configuration to take values from.
    /// @param _bodies This list of bodies to update. Order doesn't matter.
    void OverwriteDofsForRobots(const mathtool::Vector3d& _dofs,
                                const Formation& _robots);


    /// Given this and another configuration, copy the DOF values from the other
    /// to the DOF values in this one, but only for each given robot indexed.
    /// @param _fromCfg The configuration to take values from.
    /// @param _bodies This list of bodies to update. Order doesn't matter.
    void OverwriteDofsForRobots(const GroupCfg& _fromCfg,
                                const Formation& _robots);


    /// Overwrites all data in this cfg, assumes the length of _dofs is the same
    /// as the length of DOF() * GetNumRobots(). Inherently assumes that all
    /// robots have the same number of Dofs.
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
                       int* const _nTicks,
                       const double _positionRes, const double _orientationRes);

    /// Test if a group configuration lies within a boundary and also within the
    /// robot's c-space limits.
    /// @param[in] _boundary The boundary to check.
    /// @return True if the configuration places the robot inside both the
    ///         boundary and its DOF limits.
    bool InBounds(const Boundary* const _b) const noexcept;
    /// @overload
    bool InBounds(const Environment* const _env) const noexcept;


    /// Normalize Orientation DOFs for a Group Cfg
    virtual void NormalizeOrientation(
                               const Formation& _robots = Formation()) noexcept;


    /// Change the roadmap that this group is using/in reference to. Also
    /// performs compatibility/verification tests to see if it's possible.
    /// Note: Does NOT add this new cfg to any roadmap, but makes all cfg info
    ///       local (everything will be in m_localCfgs) so that there are no
    ///       issues when adding to the roadmap later.
    GroupCfg ChangeRoadmap(GraphType* const _newRoadmap) const;


    /// Return whether the cfg for the robot is local or in an individual
    /// roadmap already.
    bool IsLocalCfg(const size_t _robotIndex) const noexcept;

    ///@}
    ///@name Output Utilities
    ///@{

    std::string PrettyPrint(const size_t _precision = 4) const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    GraphType* m_groupMap{nullptr};  ///< The robot group.

    VIDSet m_vids;   ///< The individual VIDs in this aggregate configuration.
    std::vector<Cfg> m_localCfgs; ///< Individual cfgs not in a map.

    ///@}

};

std::ostream& operator<<(std::ostream&, const GroupCfg&);

#endif
