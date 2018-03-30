#ifndef GROUP_CFG_H_
#define GROUP_CFG_H_

#include <cstddef>
#include <vector>

class Cfg;
template <typename Vertex, typename Edge> class GroupRoadmap;
template <typename Vertex> class GroupLocalPlan;
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

    ///@}
    ///@name Construction
    ///@{

    explicit GroupCfg(GraphType* const _groupMap = nullptr);

    ///@}
    ///@name Equality
    ///@{

    bool operator==(const GroupCfg& _other) const noexcept;
    bool operator!=(const GroupCfg& _other) const noexcept;

    ///@}
    ///@name Accessors
    ///@{

    /// Set the individual cfg for a robot to a local copy of an cfg.
    /// @param _robot The robot which the cfg refers to.
    /// @param _cfg The cfg.
    void SetCfg(Robot* const _robot, Cfg&& _cfg);

    /// Set the individual cfg for a robot to a roadmap copy of an cfg.
    /// @param _robot The robot which the cfg refers to.
    /// @param _vid The cfg descriptor.
    void SetCfg(Robot* const _robot, const VID _vid);

    /// Get the individual Cfg for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual configuration for the indexed robot.
    Cfg& GetRobotCfg(const size_t _index);

    /// Get the individual Cfg for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual configuration for the indexed robot.
    const Cfg& GetRobotCfg(const size_t _index) const;

    const VIDSet& GetVIDSet() const { return m_vids; }

    VID GetVID(const size_t _index) const noexcept;

    ///@}

  private:

    ///@name Internal State
    ///@{

    GraphType* m_groupMap{nullptr};  ///< The robot group.

    VIDSet m_vids;   ///< The individual VIDs in this aggregate configuration.
    std::vector<Cfg> m_localCfgs; ///< Individual cfgs not in a map.

    ///@}

};

#endif
