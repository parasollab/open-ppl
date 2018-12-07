#ifndef GROUP_PATH_H_
#define GROUP_PATH_H_

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "Utilities/PMPLExceptions.h"

#include <algorithm>


////////////////////////////////////////////////////////////////////////////////
/// A path of connected configurations from a given roadmap.
///
/// The implementation uses a vector of VID's as the primary representation.
/// The corresponding configurations are computed lazily upon request.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupPath final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType       GroupCfg;
    typedef typename MPTraits::GroupRoadmapType   GroupRoadmapType;
    typedef typename GroupRoadmapType::VID        VID;
    typedef typename GroupCfg::Formation          Formation;

    ///@}
    ///@name Construction
    ///@{

    /// Construct an empty path.
    /// @param _r The roadmap used by this path.
    GroupPath(GroupRoadmapType* const _r = nullptr);

    ///@}
    ///@name Path Interface
    ///@{

    /// Get the roadmap used by this path.
    GroupRoadmapType* GetRoadmap() const noexcept;

    /// Get the number of cfgs in the path.
    size_t Size() const noexcept;

    /// Check if the path is empty.
    bool Empty() const noexcept;

    /// Get the total edge weight.
    double Length() const;

    /// Get the VIDs in the path.
    const std::vector<VID>& VIDs() const noexcept;

    /// Get a copy of the Cfgs in the path.
    /// @warning If the cfgs in the roadmap are later altered (i.e., if the DOF
    ///          values or labels are edited), this copy will be out-of-date.
    const std::vector<GroupCfg>& Cfgs() const;

    /// Get the current full Cfg path with steps spaced one environment
    ///        resolution apart. This is not cached due to its size and
    ///        infrequent usage.
    /// @param _lib The planning library to use.
    /// @param _lp  The local planner label to use when connecting cfgs.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes.
    template <typename MPLibrary>
    const std::vector<GroupCfg> FullCfgs(MPLibrary* const _lib,
        const string& _lp = "") const;

    /// Append another path to the end of this one.
    /// @param _p The path to append.
    GroupPath& operator+=(const GroupPath& _p);

    /// Add another path to the end of this one and return the result.
    /// @param _p The path to add.
    GroupPath operator+(const GroupPath& _p) const;

    /// Append a new set of VIDs to the end of this path.
    /// @param _vids The VIDs to append.
    GroupPath& operator+=(const std::vector<VID>& _vids);

    /// Add a new set of VIDs to the end of this path and return the result.
    /// @param _vids The VIDs to add.
    GroupPath operator+(const std::vector<VID>& _vids) const;

    /// Copy assignment operator.
    GroupPath& operator=(const GroupPath& _p);

    /// Clear all data in the path.
    void Clear();

    /// Clear cached data, but leave the VIDs.
    void FlushCache();

    ///@}

  private:

    ///@name Helpers
    ///@{

    void AssertSameMap(const GroupPath& _p) const;

    ///@}
    ///@name Internal State
    ///@{

    GroupRoadmapType* const m_roadmap;       ///< The roadmap.
    std::vector<VID> m_vids;            ///< The vids of the path configurations.

    std::vector<GroupCfg> m_cfgs;        ///< The path configurations.
    mutable bool m_cfgsCached{false};   ///< Are the current cfgs correct?

    double m_length{0};                 ///< The path length.
    mutable bool m_lengthCached{false}; ///< Is the current path length correct?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GroupPath<MPTraits>::
GroupPath(GroupRoadmapType* const _r) : m_roadmap(_r) { }

/*------------------------------ Path Interface ------------------------------*/

template <typename MPTraits>
typename MPTraits::GroupRoadmapType*
GroupPath<MPTraits>::
GetRoadmap() const noexcept {
  return m_roadmap;
}


template <typename MPTraits>
size_t
GroupPath<MPTraits>::
Size() const noexcept {
  return m_vids.size();
}


template <typename MPTraits>
bool
GroupPath<MPTraits>::
Empty() const noexcept {
  return m_vids.empty();
}


template <typename MPTraits>
double
GroupPath<MPTraits>::
Length() const {
  if(!m_lengthCached) {
    double& length = const_cast<double&>(m_length);
    length = 0;
    for(auto start = m_vids.begin(); start + 1 < m_vids.end(); ++start) {
      if(*start == *(start + 1))
        continue;  // Skip repeated vertices.
      typename GroupRoadmapType::edge_descriptor ed(*start, *(start + 1));
      typename GroupRoadmapType::vertex_iterator vi;
      typename GroupRoadmapType::adj_edge_iterator ei;
      if(m_roadmap->find_edge(ed, vi, ei))
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
const std::vector<typename GroupPath<MPTraits>::VID>&
GroupPath<MPTraits>::
VIDs() const noexcept {
  return m_vids;
}


template <typename MPTraits>
const std::vector<typename MPTraits::GroupCfgType>&
GroupPath<MPTraits>::
Cfgs() const {
  if(!m_cfgsCached) {
    std::vector<GroupCfg>& cfgs = const_cast<std::vector<GroupCfg>&>(m_cfgs);
    cfgs.clear();
    cfgs.reserve(m_vids.size());
    for(const auto& vid : m_vids)
      cfgs.push_back(m_roadmap->GetVertex(vid));
    m_cfgsCached = true;
  }
  return m_cfgs;
}


template <typename MPTraits>
template <typename MPLibrary>
const std::vector<typename MPTraits::GroupCfgType>
GroupPath<MPTraits>::
FullCfgs(MPLibrary* const _lib, const string& _lp) const {
  if(m_vids.empty())
    return std::vector<GroupCfg>();

  std::vector<GroupCfg> out = {m_roadmap->GetVertex(m_vids.front())};

  // Set up local planner to recreate edges. If none was provided, use edge
  // planner, or fall back to straight-line.
  auto env = _lib->GetMPProblem()->GetEnvironment();

  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    // Get the next edge.
    bool validEdge = false;
    typename GroupRoadmapType::adj_edge_iterator ei;
    {
      typename GroupRoadmapType::edge_descriptor ed(*it, *(it+1));
      typename GroupRoadmapType::vertex_iterator vi;
      validEdge = m_roadmap->find_edge(ed, vi, ei);

      if(!validEdge)
        throw RunTimeException(WHERE) << "Edge " << *it << ", " << *(it+1)
                                      << " doesn't exist in roadmap!";
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

    Formation formation = ei->property().GetActiveRobots();

    // Recreate this edge, including intermediates.
    GroupCfg& start = m_roadmap->GetVertex(*it);
    GroupCfg& end   = m_roadmap->GetVertex(*(it+1));

    // Construct a resolution-level path along the recreated edge.
    if(!ei->property().SkipEdge()) {
      std::vector<GroupCfg> recreatedEdge = ei->property().GetIntermediates();
      recreatedEdge.insert(recreatedEdge.begin(), start);
      recreatedEdge.push_back(end);
      for(auto cit = recreatedEdge.begin(); cit + 1 != recreatedEdge.end(); ++cit) {
        std::vector<GroupCfg> edge = lp->ReconstructPath(*cit, *(cit+1),
                                std::vector<GroupCfg>(), env->GetPositionRes(),
                                env->GetOrientationRes(), formation);
        out.insert(out.end(), edge.begin(), edge.end());
      }
    }
    else {
      //For assembly planning: Don't reconstruct the edge when it's for a part
      // that has been placed off to the side, just use the two cfgs.
      out.push_back(start); // This edge will just be (start, end)
    }
    out.push_back(end);
  }
  return out;
}


template <typename MPTraits>
GroupPath<MPTraits>&
GroupPath<MPTraits>::
operator+=(const GroupPath& _p) {
  AssertSameMap(_p);
  return *this += _p.m_vids;
}


template <typename MPTraits>
GroupPath<MPTraits>
GroupPath<MPTraits>::
operator+(const GroupPath& _p) const {
  AssertSameMap(_p);
  return *this + _p.m_vids;
}


template <typename MPTraits>
GroupPath<MPTraits>&
GroupPath<MPTraits>::
operator+=(const std::vector<VID>& _vids) {
  if(_vids.size()) {
    std::copy(_vids.begin(), _vids.end(), back_inserter(m_vids));
    m_lengthCached = false;
    m_cfgsCached = false;
  }
  return *this;
}


template <typename MPTraits>
GroupPath<MPTraits>
GroupPath<MPTraits>::
operator+(const std::vector<VID>& _vids) const {
  GroupPath out(*this);
  out += _vids;
  return out;
}


template <typename MPTraits>
GroupPath<MPTraits>&
GroupPath<MPTraits>::
operator=(const GroupPath& _p) {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE, "Can't assign path from another roadmap");

  m_vids         = _p.m_vids;
  m_cfgs         = _p.m_cfgs;
  m_cfgsCached   = _p.m_cfgsCached;
  m_length       = _p.m_length;
  m_lengthCached = _p.m_lengthCached;

  return *this;
}


template <typename MPTraits>
void
GroupPath<MPTraits>::
Clear() {
  m_lengthCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
  m_vids.clear();
}


template <typename MPTraits>
void
GroupPath<MPTraits>::
FlushCache() {
  m_lengthCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
GroupPath<MPTraits>::
AssertSameMap(const GroupPath& _p) const {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE, "Can't add paths from different roadmaps!");
}

/*----------------------------------------------------------------------------*/

#endif
