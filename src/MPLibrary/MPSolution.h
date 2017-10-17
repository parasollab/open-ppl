#ifndef MP_SOLUTION_TYPE_H_
#define MP_SOLUTION_TYPE_H_

#include "Utilities/MetricUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// Container for the output of a planning algorithm. Includes free and blocked
/// roadmaps, a path, and a stat class.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPSolutionType final {

  public:

    ///@name Solution Object Types
    ///@{

    typedef typename MPTraits::Path        Path;
    typedef typename MPTraits::RoadmapType RoadmapType;

    ///@}
    ///@name Construction
    ///@{

    MPSolutionType(Robot* const _r);

    ~MPSolutionType();

    ///@}
    ///@name Roadmap Accessors
    ///@{

    void SetRoadmap(RoadmapType* const _r) noexcept;
    RoadmapType* GetRoadmap() const noexcept;
    Robot* GetRobot() const noexcept;   ///< Probably will be deleted, not sure yet.
    RoadmapType* GetBlockRoadmap() const noexcept;

    ///@}
    ///@name Path Accessors
    ///@{

    Path* GetPath() noexcept;

    ///@}
    ///@name StatClass Accessor
    ///@{

    StatClass* GetStatClass() noexcept;

    ///@}

  private:

    ///@name Internal State
    ///@{

    Robot* const m_robot;            ///< The robot executing this task.
    RoadmapType* m_freeMap{nullptr}; ///< The free-space roadmap.
    RoadmapType* m_obstMap{nullptr}; ///< The obstacle-space roadmap.
    Path*        m_path{nullptr};    ///< The current solution path.
    StatClass*   m_stats{nullptr};   ///< Performance tracking.

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MPSolutionType<MPTraits>::
MPSolutionType(Robot* const _r) : m_robot(_r) {
  m_freeMap = new RoadmapType(_r);
  m_obstMap = new RoadmapType(_r);
  m_path = new Path(m_freeMap);
  m_stats = new StatClass();
}


template <typename MPTraits>
MPSolutionType<MPTraits>::
~MPSolutionType() {
  delete m_freeMap;
  delete m_obstMap;
  delete m_path;
  delete m_stats;
}

/*---------------------------- Roadmap Accessors -----------------------------*/

template <typename MPTraits>
inline
void
MPSolutionType<MPTraits>::
SetRoadmap(RoadmapType* const _r) noexcept {
  m_freeMap->SetGraph(_r->GetGraph());
}


template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPSolutionType<MPTraits>::
GetRoadmap() const noexcept {
  return m_freeMap;
}


template <typename MPTraits>
inline
Robot*
MPSolutionType<MPTraits>::
GetRobot() const noexcept {
  return m_robot;
}


template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPSolutionType<MPTraits>::
GetBlockRoadmap() const noexcept {
  return m_obstMap;
}

/*------------------------------ Path Accessors ------------------------------*/

template <typename MPTraits>
inline
typename MPTraits::Path*
MPSolutionType<MPTraits>::
GetPath() noexcept {
  return m_path;
}

/*------------------------------ Stats Accessors -----------------------------*/

template <typename MPTraits>
inline
StatClass*
MPSolutionType<MPTraits>::
GetStatClass() noexcept {
  return m_stats;
}

/*----------------------------------------------------------------------------*/

#endif
