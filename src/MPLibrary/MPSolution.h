#ifndef MP_SOLUTION_TYPE_H_
#define MP_SOLUTION_TYPE_H_

#include "ConfigurationSpace/LocalObstacleMap.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "Utilities/MetricUtils.h"

#include <map>
#include <memory>
#include <unordered_map>

class Robot;


////////////////////////////////////////////////////////////////////////////////
/// Container for the output of a planning algorithm. Includes free and blocked
/// roadmaps, a path, and a local obstacle map for each robot. Also includes a
/// stat class for performance tracking.
///
/// Currently this object can represent a solution for either a single robot or
/// a group. To be homogenized after the groups code matures.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPSolutionType final {

  public:

    ///@name Solution Object Types
    ///@{

    typedef typename MPTraits::Path             Path;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::LocalObstacleMap LocalObstacleMap;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::WeightType       WeightType;
    typedef RoadmapGraph<CfgType, WeightType>   GraphType;

    /// The outputs for an individual robot.
    struct RobotSolution {
      std::unique_ptr<RoadmapType>  freeMap; ///< The free-space roadmap.
      std::unique_ptr<RoadmapType>  obstMap; ///< The obstacle-space roadmap.
      std::unique_ptr<Path>            path; ///< The current solution path.
      std::unique_ptr<LocalObstacleMap> lom; ///< The local obstacle map.

      RobotSolution() = default;

      /// Initialize a solution for a single robot.
      RobotSolution(Robot* const _robot, StatClass* const _stats);

    };

    ///@}
    ///@name Construction
    ///@{

    MPSolutionType(Robot* const _r);

    MPSolutionType(RobotGroup* const _g);

    ///@}
    ///@name Modifiers
    ///@{

    void SetRobot(Robot* const _r) noexcept;

    ///@}
    ///@name Accessors
    ///@{

    RoadmapType* GetRoadmap(Robot* const _r = nullptr) const noexcept;

    RoadmapType* GetBlockRoadmap(Robot* const _r = nullptr) const noexcept;

    Path* GetPath(Robot* const _r = nullptr) const noexcept;

    LocalObstacleMap* GetLocalObstacleMap(Robot* const _r = nullptr) const noexcept;

    GroupRoadmapType* GetGroupRoadmap() const noexcept;

    StatClass* GetStatClass() const noexcept;

    ///@}

  private:

    ///@name Helpers
    ///@{

    const RobotSolution& GetRobotSolution(Robot* _r) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    Robot* m_robot{nullptr};            ///< The robot executing this task.
    RobotGroup* m_group{nullptr};       ///< The robot group.

    std::unique_ptr<StatClass>  m_stats; ///< Performance tracking.

    /// The solution object for each robot.
    std::unordered_map<Robot*, RobotSolution> m_solutions;

    /// The group roadmap.
    std::unique_ptr<GroupRoadmapType> m_groupMap;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MPSolutionType<MPTraits>::
RobotSolution::
RobotSolution(Robot* const _robot, StatClass* const _stats)
  : freeMap(new RoadmapType(_robot)),
    obstMap(new RoadmapType(_robot)),
    path   (new Path(freeMap.get())),
    lom    (new LocalObstacleMap(_stats)) { }


template <typename MPTraits>
MPSolutionType<MPTraits>::
MPSolutionType(Robot* const _r)
  : m_robot(_r), m_group(nullptr), m_stats(new StatClass()) {
  m_solutions[m_robot] = std::move(RobotSolution(m_robot, m_stats.get()));
}


template <typename MPTraits>
MPSolutionType<MPTraits>::
MPSolutionType(RobotGroup* const _g)
  : m_robot(nullptr), m_group(_g), m_stats(new StatClass()) {

  for(auto robot : *m_group)
    m_solutions[robot] = std::move(RobotSolution(robot, m_stats.get()));
}

/*-------------------------------- Modifiers ---------------------------------*/

template <typename MPTraits>
void
MPSolutionType<MPTraits>::
SetRobot(Robot* const _r) noexcept {
  // Move m_robot's solution to match the new pointer.
  auto iter = m_solutions.find(m_robot);
  m_solutions[_r] = std::move(iter->second);
  m_solutions.erase(iter);

  m_robot = _r;

  m_solutions[_r].freeMap->SetRobot(_r);
  m_solutions[_r].obstMap->SetRobot(_r);
  m_solutions[_r].path->FlushCache();
}

/*---------------------------- Roadmap Accessors -----------------------------*/

template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPSolutionType<MPTraits>::
GetRoadmap(Robot* const _r) const noexcept {
  return GetRobotSolution(_r).freeMap.get();
}


template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPSolutionType<MPTraits>::
GetBlockRoadmap(Robot* const _r) const noexcept {
  return GetRobotSolution(_r).obstMap.get();
}


template <typename MPTraits>
inline
typename MPTraits::Path*
MPSolutionType<MPTraits>::
GetPath(Robot* const _r) const noexcept {
  return GetRobotSolution(_r).path.get();
}


template <typename MPTraits>
inline
typename MPTraits::LocalObstacleMap*
MPSolutionType<MPTraits>::
GetLocalObstacleMap(Robot* const _r) const noexcept {
  return GetRobotSolution(_r).lom.get();
}


template <typename MPTraits>
inline
typename MPTraits::GroupRoadmapType*
MPSolutionType<MPTraits>::
GetGroupRoadmap() const noexcept {
  return m_groupMap.get();
}


template <typename MPTraits>
inline
StatClass*
MPSolutionType<MPTraits>::
GetStatClass() const noexcept {
  return m_stats.get();
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
inline
const typename MPSolutionType<MPTraits>::RobotSolution&
MPSolutionType<MPTraits>::
GetRobotSolution(Robot* _r) const noexcept {
  if(!_r)
    _r = m_robot;
  try {
    return m_solutions.at(_r);
  }
  catch(const std::runtime_error& _e) {
    std::ostringstream oss;
    oss << "Robot " << _r << " has no data in this solution.";
    throw RunTimeException(WHERE, oss.str());
  }
}

/*----------------------------------------------------------------------------*/

#endif
