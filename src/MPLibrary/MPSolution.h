#ifndef PMPL_MP_SOLUTION_TYPE_H_
#define PMPL_MP_SOLUTION_TYPE_H_

#include "ConfigurationSpace/LocalObstacleMap.h"
#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "Utilities/MetricUtils.h"

#include <map>
#include <memory>
#include <unordered_map>

class Robot;
//class InteractionTemplate;


////////////////////////////////////////////////////////////////////////////////
/// Container for the output of a planning algorithm. Includes free and blocked
/// roadmaps, a path, and a local obstacle map for each robot. Also includes a
/// stat class for performance tracking.
///
/// @todo Currently this object can represent a solution for each single robot
///       and also a single robot group. Extend support to include multiple
///       robot groups.
///
/// @note This object makes only one stat class, which is shared across all
///       usees.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPSolutionType final {

  public:

    ///@name Solution Object Types
    ///@{

    typedef typename MPTraits::Path             Path;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename RoadmapType::GraphType     GraphType;
    typedef typename MPTraits::LocalObstacleMap LocalObstacleMap;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::GroupPathType    GroupPathType;

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

    GroupPathType* GetGroupPath() const noexcept;

    StatClass* GetStatClass() const noexcept;

    Robot* GetRobot() const noexcept;

    RobotGroup* GetRobotGroup() const noexcept;

    std::vector<std::unique_ptr<InteractionTemplate>>& GetInteractionTemplates();

    void AddInteractionTemplate(InteractionTemplate*);

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

    std::unique_ptr<StatClass> m_stats; ///< Performance tracking.

    /// The solution object for each robot.
    std::unordered_map<Robot*, RobotSolution> m_solutions;

    /// The group roadmap.
    std::unique_ptr<GroupRoadmapType> m_groupMap;

    /// The group path.
    std::unique_ptr<GroupPathType> m_groupPath;

    /// The set of Interaction Templates
    std::vector<std::unique_ptr<InteractionTemplate>> m_interactionTemplates;

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
  : m_robot(_r), m_stats(new StatClass()) {
  m_solutions[m_robot] = std::move(RobotSolution(m_robot, m_stats.get()));
}


template <typename MPTraits>
MPSolutionType<MPTraits>::
MPSolutionType(RobotGroup* const _g)
  : m_group(_g), m_stats(new StatClass()),
    m_groupMap(nullptr), m_groupPath(nullptr) {
  for(auto robot : *m_group)
    m_solutions[robot] = std::move(RobotSolution(robot, m_stats.get()));

  // Must happen after solutions are populated!
  m_groupMap = std::unique_ptr<GroupRoadmapType>(new GroupRoadmapType(_g, this));
  m_groupPath = std::unique_ptr<GroupPathType>(new GroupPathType(m_groupMap.get()));
}

/*-------------------------------- Modifiers ---------------------------------*/

template <typename MPTraits>
void
MPSolutionType<MPTraits>::
SetRobot(Robot* const _r) noexcept {
  // Move m_robot's solution to match the new pointer.
  if(m_robot == _r){
    return;
  }
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
typename MPSolutionType<MPTraits>::GroupPathType*
MPSolutionType<MPTraits>::
GetGroupPath() const noexcept {
  return m_groupPath.get();
}


template <typename MPTraits>
inline
StatClass*
MPSolutionType<MPTraits>::
GetStatClass() const noexcept {
  return m_stats.get();
}


template <typename MPTraits>
Robot*
MPSolutionType<MPTraits>::
GetRobot() const noexcept {
  return m_robot;
}


template <typename MPTraits>
RobotGroup*
MPSolutionType<MPTraits>::
GetRobotGroup() const noexcept {
  return m_group;
}

template<typename MPTraits>
std::vector<std::unique_ptr<InteractionTemplate>>&
MPSolutionType<MPTraits>::
GetInteractionTemplates(){
  return m_interactionTemplates;
}

template<typename MPTraits>
void
MPSolutionType<MPTraits>::
AddInteractionTemplate(InteractionTemplate* _it){
  m_interactionTemplates.emplace_back(std::unique_ptr<InteractionTemplate>(_it));
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
  catch(const std::out_of_range& _e) {
    throw RunTimeException(WHERE) << "Robot " << _r->GetLabel()
                                  << " (" << _r << ") "
                                  << "has no data in this solution.";
  }
}

/*----------------------------------------------------------------------------*/

#endif
