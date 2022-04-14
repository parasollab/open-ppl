#ifndef PPL_COMPOSITE_STATE_H_
#define PPL_COMPOSITE_STATE_H_

// #include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/CompositeGraph.h"
#include "ConfigurationSpace/CompositeEdge.h"

#include <cstddef>
#include <iostream>
#include <vector>

#include "Vector.h"

class Robot;
class RobotGroup;

template <typename GraphType>
class CompositeEdge;


////////////////////////////////////////////////////////////////////////////////
/// An aggregate configuration which represents a state for each robot
/// in a robot group.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
class CompositeState {

  public:

    ///@name Local Types
    ///@{

    typedef typename GraphType::CfgType CfgType;
    // typedef typename CompositeState<GraphType> CompositeStateType;
    typedef CompositeEdge<GraphType>  CompositeEdgeType;

    typedef CompositeGraph<CompositeState, CompositeEdgeType> GroupGraphType;

    typedef size_t           VID;      ///< A VID in an individual graph.
    typedef std::vector<VID> VIDSet;   ///< A set of VIDs from indiv. graphs.

    typedef GraphType IndividualGraph;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a group configuration.
    /// @param _groupMap The group roadmap to which this configuration belongs,
    ///                  or null if it is not in a map.
    /// @param _init Default-initialize local configurations?
    /// @todo This object does not work at all without a group map. We should
    ///       throw relevant exceptions if needed.
    explicit CompositeState(GroupGraphType* const _groupGraph = nullptr,  
      CfgType& (*_vertexGetter)(const VID) = nullptr);

    ///@}
    ///@name Equality
    ///@{

    /// Check if the current and given group configurations are equal.
    /// @param _other The given group configuration.
    /// @return True if equal, false otherwise.
    bool operator==(const CompositeState& _other) const noexcept;

    /// Check if the current and given group configurations are unequal.
    /// @param _other The given group configuration.
    /// @return True if unequal, false otherwise.
    bool operator!=(const CompositeState& _other) const noexcept;

    ///@}
    ///@name Robots
    ///@{
    /// Access the robots within this group configuration.

    /// Get the number of robots.
    size_t GetNumRobots() const noexcept;

    /// Get the full vector of robot pointers.
    const std::vector<Robot*>& GetRobots() const noexcept;

    /// Get the robot pointer for a group member by index.
    /// @param _index The desired index.
    Robot* GetRobot(const size_t _index) const;

    ///@}
    ///@name Roadmap Accessors
    ///@{
    /// These functions provide access to the related group map (if any) and
    /// descriptors for non-local individual configurations.

    /// Get the group roadmap this group cfg is with respect to.
    GroupGraphType* GetGroupRoadmap() const noexcept;

    /// Get the VID for a particular robot.
    /// @param _index The index (within the group) of the robot.
    /// @return The VID of the robot's individual configuration, or INVALID_VID
    ///         if it is a local configuration.
    VID GetVID(const size_t _index) const noexcept;

    VID GetVID(Robot* const _robot) const;
 
    ///@}
    ///@name Individual Configurations
    ///@{
    /// These functions manage the individual configurations that comprise this
    /// group configuration.

    /// Set the individual cfg for a robot to a roadmap copy of an cfg.
    /// @param _robot The robot which the cfg refers to.
    /// @param _vid The cfg descriptor.
    void SetRobotCfg(Robot* const _robot, const VID _vid);

    /// Set the individual cfg for a robot to a roadmap copy of an cfg.
    /// @param _index The robot's group index which the cfg refers to.
    /// @param _vid The cfg descriptor.
    void SetRobotCfg(const size_t _index, const VID _vid);

    /// Get the individual Cfg for a robot in the group.
    /// @param _robot The robot which the cfg refers to.
    /// @return The individual configuration for the indexed robot.
    CfgType& GetRobotCfg(Robot* const _robot);

    /// Get the individual Cfg for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual configuration for the indexed robot.
    CfgType& GetRobotCfg(const size_t _index);

    /// Get the individual Cfg for a robot in the group.
    /// @param _robot The robot which the cfg refers to.
    /// @return The individual configuration for the indexed robot.
    const CfgType& GetRobotCfg(Robot* const _robot) const;

    /// Get the individual Cfg for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual configuration for the indexed robot.
    const CfgType& GetRobotCfg(const size_t _index) const;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Verify that an index is valid. Throw an exception if not.
    /// @param _robotIndex The (group) index to verify.
    void VerifyIndex(const size_t _robotIndex) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    GroupGraphType* m_groupGraph{nullptr};  ///< The group graph.

    VIDSet m_vids;   ///< The individual VIDs in this aggregate configuration.

    mutable CfgType& (*m_vertexGetter)(const VID){nullptr};

    ///@}

};

template <typename GraphType>
std::ostream& operator<<(std::ostream&, const CompositeState<GraphType>&);

#endif
