#ifndef PATH_H_
#define PATH_H_

#include <algorithm>

#include "Roadmap.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "Utilities/PMPLExceptions.h"

////////////////////////////////////////////////////////////////////////////////
/// A path of connected configurations from a given roadmap.
///
/// The implementation uses a vector of VID's as the primary representation.
/// The corresponding configurations are computed lazily upon request.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PathType {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::WeightType    WeightType;
    typedef typename MPTraits::RoadmapType   RoadmapType;
    typedef typename RoadmapType::GraphType  GraphType;
    typedef typename RoadmapType::VID        VID;

    ///@}
    ///@name Construction
    ///@{

    /// Construct an empty path.
    /// @param[in] _r The roadmap used by this path.
    PathType(RoadmapType* const _r);

    ///@}
    ///@name Path Interface
    ///@{

    /// Get the roadmap used by this path.
    RoadmapType* GetRoadmap() const noexcept;

    /// Get the number of cfgs in the path.
    size_t Size() const noexcept;

    /// Get the total edge weight.
    double Length() const;

    /// Get the VIDs in the path.
    const std::vector<VID>& VIDs() const noexcept;

    /// Get a copy of the Cfgs in the path.
    /// @warning If the cfgs in the roadmap are later altered (i.e., if the DOF
    ///          values or labels are edited), this copy will be out-of-date.
    const std::vector<CfgType>& Cfgs() const;

    /// Get the current full Cfg path with steps spaced one environment
    ///        resolution apart. This is not cached due to its size and
    ///        infrequent usage.
    /// @param[in] _lib The planning library to use.
    /// @param[in] _lp  The local planner label to use when connecting cfgs.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes.
    template <typename MPLibrary>
    const std::vector<CfgType> FullCfgs(MPLibrary* const _lib,
        const string& _lp = "") const;

    /// Append another path to the end of this one.
    /// @param[in] _p The path to append.
    PathType& operator+=(const PathType& _p);

    /// Add another path to the end of this one and return the result.
    /// @param[in] _p The path to add.
    PathType operator+(const PathType& _p) const;

    /// Append a new set of VIDs to the end of this path.
    /// @param[in] _vids The VIDs to append.
    PathType& operator+=(const std::vector<VID>& _vids);

    /// Add a new set of VIDs to the end of this path and return the result.
    /// @param[in] _vids The VIDs to add.
    PathType operator+(const std::vector<VID>& _vids) const;

    /// Clear all data in the path.
    void Clear();

    ///@}

  private:

    ///@name Helpers
    ///@{

    void AssertSameMap(const PathType& _p) const;

    ///@}
    ///@name Internal State
    ///@{

    RoadmapType* const m_roadmap;       ///< The roadmap.
    std::vector<VID> m_vids;            ///< The vids of the path configurations.

    std::vector<CfgType> m_cfgs;        ///< The path configurations.
    mutable bool m_cfgsCached{false};   ///< Are the current cfgs correct?

    double m_length{0};                 ///< The path length.
    mutable bool m_lengthCached{false}; ///< Is the current path length correct?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
PathType<MPTraits>::
PathType(RoadmapType* const _r) : m_roadmap(_r) { }

/*------------------------------ Path Interface ------------------------------*/

template <typename MPTraits>
typename MPTraits::RoadmapType*
PathType<MPTraits>::
GetRoadmap() const noexcept {
  return m_roadmap;
}


template <typename MPTraits>
size_t
PathType<MPTraits>::
Size() const noexcept {
  return m_vids.size();
}


template <typename MPTraits>
double
PathType<MPTraits>::
Length() const {
  if(!m_lengthCached) {
    double& length = const_cast<double&>(m_length);
    length = 0;
    for(auto start = m_vids.begin(); start + 1 < m_vids.end(); ++start) {
      if(*start == *(start + 1))
        continue;  // Skip repeated vertices.
      typename GraphType::edge_descriptor ed(*start, *(start + 1));
      typename GraphType::vertex_iterator vi;
      typename GraphType::adj_edge_iterator ei;
      if(m_roadmap->GetGraph()->find_edge(ed, vi, ei))
        length += (*ei).property().GetWeight();
      else
        throw RunTimeException(WHERE, "Tried to compute length for a path "
            "containing the edge (" + to_string(*start) + "," +
            to_string(*(start + 1)) + "), but that edge was not found in the "
            "graph.");
    }
    m_lengthCached = true;
  }
  return m_length;
}


template <typename MPTraits>
const std::vector<typename PathType<MPTraits>::VID>&
PathType<MPTraits>::
VIDs() const noexcept {
  return m_vids;
}


template <typename MPTraits>
const std::vector<typename MPTraits::CfgType>&
PathType<MPTraits>::
Cfgs() const {
  if(!m_cfgsCached) {
    std::vector<CfgType>& cfgs = const_cast<std::vector<CfgType>&>(m_cfgs);
    cfgs.clear();
    cfgs.reserve(m_vids.size());
    for(const auto& vid : m_vids)
      cfgs.push_back(m_roadmap->GetGraph()->GetVertex(vid));
    m_cfgsCached = true;
  }
  return m_cfgs;
}


template <typename MPTraits>
template <typename MPLibrary>
const std::vector<typename MPTraits::CfgType>
PathType<MPTraits>::
FullCfgs(MPLibrary* const _lib, const string& _lp) const {
  if(m_vids.empty())
    return std::vector<CfgType>();

  GraphType* g = m_roadmap->GetGraph();
  std::vector<CfgType> out = {g->GetVertex(m_vids.front())};

  // Set up local planner to recreate edges. If none was provided, use edge
  // planner, or fall back to straight-line.
  auto env = _lib->GetMPProblem()->GetEnvironment();

  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    // Get the next edge.
    typename GraphType::adj_edge_iterator ei;
    {
      typename GraphType::edge_descriptor ed(*it, *(it+1));
      typename GraphType::vertex_iterator vi;
      g->find_edge(ed, vi, ei);
    }

    // Use the local planner from parameter if specified.
    // If not specified, use the edge lp.
    // Fall back to straight-line if edge lp is not available (this will always
    // happen if it was grown with an extender).
    typename MPLibrary::LocalPlannerPointer lp;
    if(!_lp.empty())
      lp = _lib->GetLocalPlanner(_lp);
    else {
      try {
        lp = _lib->GetLocalPlanner(ei->property().GetLPLabel());
      }
      catch(...) {
        lp = _lib->GetLocalPlanner("sl");
      }
    }

    // Recreate this edge, including intermediates.
    CfgType& start = g->GetVertex(*it);
    CfgType& end   = g->GetVertex(*(it+1));
    std::vector<CfgType> recreatedEdge = ei->property().GetIntermediates();
    recreatedEdge.insert(recreatedEdge.begin(), start);
    recreatedEdge.push_back(end);

    // Construct a resolution-level path along the recreated edge.
    for(auto cit = recreatedEdge.begin(); cit + 1 != recreatedEdge.end(); ++cit) {
      std::vector<CfgType> edge = lp->ReconstructPath(*cit, *(cit+1),
          std::vector<CfgType>(), env->GetPositionRes(),
          env->GetOrientationRes());
      out.insert(out.end(), edge.begin(), edge.end());
    }
    out.push_back(end);
  }
  return out;
}


template <typename MPTraits>
PathType<MPTraits>&
PathType<MPTraits>::
operator+=(const PathType& _p) {
  AssertSameMap(_p);
  return *this += _p.m_vids;
}


template <typename MPTraits>
PathType<MPTraits>
PathType<MPTraits>::
operator+(const PathType& _p) const {
  AssertSameMap(_p);
  return *this + _p.m_vids;
}


template <typename MPTraits>
PathType<MPTraits>&
PathType<MPTraits>::
operator+=(const std::vector<VID>& _vids) {
  std::copy(_vids.begin(), _vids.end(), back_inserter(m_vids));
  m_lengthCached = false;
  m_cfgsCached = false;
  return *this;
}


template <typename MPTraits>
PathType<MPTraits>
PathType<MPTraits>::
operator+(const std::vector<VID>& _vids) const {
  PathType out(*this);
  out += _vids;
  return out;
}


template <typename MPTraits>
void
PathType<MPTraits>::
Clear() {
  m_lengthCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
  m_vids.clear();
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
PathType<MPTraits>::
AssertSameMap(const PathType& _p) const {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE, "Can't add paths from different roadmaps!");
}

/*----------------------------------------------------------------------------*/

#endif
