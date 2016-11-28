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

    typedef typename MPTraits::PathType    PathType;
    typedef typename MPTraits::RoadmapType RoadmapType;

    ///@}
    ///@name Construction
    ///@{

    MPSolutionType();
    ~MPSolutionType();

    ///@}
    ///@name Roadmap Accessors
    ///@{

    RoadmapType* GetRoadmap() noexcept;
    RoadmapType* GetBlockRoadmap() noexcept;

    ///@}
    ///@name Path Accessors
    ///@{

    PathType* GetPath() noexcept;

    ///@}
    ///@name StatClass Accessor
    ///@{

    StatClass* GetStatClass() noexcept;

    ///@}

  private:

    ///@name Internal State
    ///@{

    /// @TODO Add robot group pointer so that we know which group this belongs
    ///       to.

    RoadmapType* m_freeMap{nullptr}; ///< The free-space roadmap.
    RoadmapType* m_obstMap{nullptr}; ///< The obstacle-space roadmap.
    PathType*    m_path{nullptr};    ///< The current solution path.

    StatClass*   m_stats{nullptr};   ///< Performance tracking.

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MPSolutionType<MPTraits>::
MPSolutionType() {
  m_freeMap = new RoadmapType();
  m_obstMap = new RoadmapType();
  m_stats = new StatClass();
}


template <typename MPTraits>
MPSolutionType<MPTraits>::
~MPSolutionType() {
  delete m_freeMap;
  delete m_obstMap;
  delete m_stats;
}

/*---------------------------- Roadmap Accessors -----------------------------*/

template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPSolutionType<MPTraits>::
GetRoadmap() noexcept {
  return m_freeMap;
}


template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPSolutionType<MPTraits>::
GetBlockRoadmap() noexcept {
  return m_obstMap;
}

/*------------------------------ Path Accessors ------------------------------*/

template <typename MPTraits>
inline
typename MPTraits::PathType*
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
