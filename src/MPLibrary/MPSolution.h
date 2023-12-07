#ifndef PMPL_MP_SOLUTION_TYPE_H_
#define PMPL_MP_SOLUTION_TYPE_H_

#include "ConfigurationSpace/LocalObstacleMap.h"
#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/GroupPath.h"
#include "ConfigurationSpace/Path.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "Utilities/MetricUtils.h"

// #include <map>
// #include <memory>
// #include <unordered_map>

class Robot;
class Path;
class GroupPath;


////////////////////////////////////////////////////////////////////////////////
/// Container for the output of a planning algorithm. Includes free and blocked
/// roadmaps, a path, and a local obstacle map for each robot. Also includes a
/// stat class for performance tracking.
///
/// @todo Currently this object can represent a solution for each single robot
///       and several robot group. It can almost support multiple of each - it
///       just needs an interface for adding more robots/groups to the container.
///
/// @note This object makes only one stat class, which is shared across all
///       uses.
////////////////////////////////////////////////////////////////////////////////
class MPSolutionType final {

  public:

    ///@name Solution Object Types
    ///@{

    typedef GenericStateGraph<Cfg, DefaultWeight<Cfg>> RoadmapType;
    typedef GroupCfg<RoadmapType> GroupCfgType;
    typedef GroupRoadmap<GroupCfgType, GroupLocalPlan<RoadmapType>> GroupRoadmapType;

    /// The outputs for an individual robot.
    struct RobotSolution {
      std::unique_ptr<RoadmapType>  freeMap; ///< The free-space roadmap.
      std::unique_ptr<RoadmapType>  obstMap; ///< The obstacle-space roadmap.
      std::unique_ptr<Path>            path; ///< The current solution path.
      std::unique_ptr<LocalObstacleMap> lom; ///< The local obstacle map.

      RobotSolution() = default;

      /// Initialize a solution for a single robot.
      /// @param _robot The robot.
      /// @param _stats The stats object for timing.
      RobotSolution(Robot* const _robot, StatClass* const _stats);
    };

    /// The outputs for a robot group.
    struct GroupSolution {
      std::unique_ptr<GroupRoadmapType>  freeMap; ///< The free-space roadmap.
      std::unique_ptr<GroupRoadmapType>  obstMap; ///< The obstacle-space roadmap.
      std::unique_ptr<GroupPath>        path; ///< The current solution path.

      GroupSolution() = default;

      /// Initialize a solution for a robot group.
      /// @param _group The robot group.
      GroupSolution(RobotGroup* const _group, MPSolutionType* const _solution);
    };

    ///@}
    ///@name Construction
    ///@{

    MPSolutionType(Robot* const _r);

    MPSolutionType(RobotGroup* const _g);

    ///@}
    ///@name Modifiers
    ///@{

    void AddRobot(Robot* const _r) noexcept;

    void SetRobot(Robot* const _r) noexcept;

    void SetRoadmap(Robot* const _r, RoadmapType* _roadmap) noexcept;

    void SetPath(Robot* const _r, Path* _path) noexcept;

    void AddRobotGroup(RobotGroup* const _g) noexcept;

    void SetGroupRoadmap(RobotGroup* const _g, GroupRoadmapType* _roadmap) noexcept;

    void SetGroupPath(RobotGroup* const _r, GroupPath* _path) noexcept;

    ///@}
    ///@name Accessors
    ///@{

    RoadmapType* GetRoadmap(Robot* const _r = nullptr) const noexcept;

    RoadmapType* GetBlockRoadmap(Robot* const _r = nullptr) const noexcept;

    Path* GetPath(Robot* const _r = nullptr) const noexcept;

    LocalObstacleMap* GetLocalObstacleMap(Robot* const _r = nullptr) const
        noexcept;

    GroupRoadmapType* GetGroupRoadmap(RobotGroup* const _g = nullptr) const
        noexcept;

    GroupPath* GetGroupPath(RobotGroup* const _g = nullptr) const noexcept;

    StatClass* GetStatClass() const noexcept;

    Robot* GetRobot() const noexcept;

    RobotGroup* GetRobotGroup() const noexcept;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Get the solution for a robot.
    /// @param _r The robot.
    /// @return The solution object for _r.
    const RobotSolution* GetRobotSolution(Robot* _r) const noexcept;

    /// Get the solution for a robot.
    /// @param _r The robot.
    /// @return The solution object for _r.
    RobotSolution* GetRobotSolution(Robot* _r) noexcept;

    /// Get the solution for a robot group.
    /// @param _g The robot group.
    /// @return The solution object for _g.
    const GroupSolution* GetGroupSolution(RobotGroup* _g) const noexcept;

    /// Get the solution for a robot group.
    /// @param _g The robot group.
    /// @return The solution object for _g.
    GroupSolution* GetGroupSolution(RobotGroup* _g) noexcept;
    ///@}
    ///@name Internal State
    ///@{

    Robot* m_robot{nullptr};            ///< The robot executing this task.
    RobotGroup* m_group{nullptr};       ///< The robot group.

    std::unique_ptr<StatClass> m_stats; ///< Performance tracking.

    /// The solution object for each robot and group.
    std::unordered_map<Robot*, RobotSolution> m_individualSolutions;
    std::unordered_map<RobotGroup*, GroupSolution> m_groupSolutions;

    ///@}

};

#endif
