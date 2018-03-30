#ifndef MP_SOLUTION_TYPE_H_
#define MP_SOLUTION_TYPE_H_

#include "Utilities/MetricUtils.h"

#include "ConfigurationSpace/LocalObstacleMap.h"


////////////////////////////////////////////////////////////////////////////////
/// Container for the output of a planning algorithm. Includes free and blocked
/// roadmaps, a path, and a stat class.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPSolutionType final {

  public:

    ///@name Solution Object Types
    ///@{

    typedef typename MPTraits::Path             Path;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::LocalObstacleMap LocalObstacleMap;

    ///@}
    ///@name Construction
    ///@{

    MPSolutionType(Robot* const _r);

    ///@}
    ///@name Modifiers
    ///@{

    void SetRobot(Robot* const _r) noexcept;

    ///@}
    ///@name Accessors
    ///@{

    RoadmapType* GetRoadmap() const noexcept;

    RoadmapType* GetBlockRoadmap() const noexcept;

    Path* GetPath() const noexcept;

    StatClass* GetStatClass() const noexcept;

    LocalObstacleMap* GetLocalObstacleMap() const noexcept;

    ///@}

  private:

    ///@name Internal State
    ///@{

    Robot* m_robot{nullptr};            ///< The robot executing this task.

    std::unique_ptr<RoadmapType>  m_freeMap; ///< The free-space roadmap.
    std::unique_ptr<RoadmapType>  m_obstMap; ///< The obstacle-space roadmap.
    std::unique_ptr<Path>            m_path; ///< The current solution path.
    std::unique_ptr<StatClass>      m_stats; ///< Performance tracking.
    std::unique_ptr<LocalObstacleMap> m_lom; ///< The local obstacle map.

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MPSolutionType<MPTraits>::
MPSolutionType(Robot* const _r)
  : m_robot(_r),
    m_freeMap(new RoadmapType(_r)),
    m_obstMap(new RoadmapType(_r)),
    m_path(new Path(m_freeMap.get())),
    m_stats(new StatClass()),
    m_lom(new LocalObstacleMap(m_stats.get()))
{ }

/*-------------------------------- Modifiers ---------------------------------*/

template <typename MPTraits>
void
MPSolutionType<MPTraits>::
SetRobot(Robot* const _r) noexcept {
  m_robot = _r;
  m_freeMap->SetRobot(_r);
  m_obstMap->SetRobot(_r);
  m_path->FlushCache();
}

/*---------------------------- Roadmap Accessors -----------------------------*/

template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPSolutionType<MPTraits>::
GetRoadmap() const noexcept {
  return m_freeMap.get();
}


template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPSolutionType<MPTraits>::
GetBlockRoadmap() const noexcept {
  return m_obstMap.get();
}


template <typename MPTraits>
inline
typename MPTraits::Path*
MPSolutionType<MPTraits>::
GetPath() const noexcept {
  return m_path.get();
}


template <typename MPTraits>
inline
StatClass*
MPSolutionType<MPTraits>::
GetStatClass() const noexcept {
  return m_stats.get();
}


template <typename MPTraits>
inline
typename MPTraits::LocalObstacleMap*
MPSolutionType<MPTraits>::
GetLocalObstacleMap() const noexcept {
  return m_lom.get();
}

/*----------------------------------------------------------------------------*/

#endif
