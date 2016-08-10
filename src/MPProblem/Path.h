#ifndef PATH_H_
#define PATH_H_

#include <algorithm>

#include "Roadmap.h"
#include "LocalPlanners/StraightLine.h"
#include "Utilities/PMPLExceptions.h"

////////////////////////////////////////////////////////////////////////////////
/// \brief A path of connected configurations from a given roadmap.
///
/// The implementation uses a vector of VID's as the primary representation.
/// The corresponding configurations are computed lazily upon request.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class Path {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::CfgRef           CfgRef;
    typedef typename MPTraits::WeightType       WeightType;
    typedef typename MPTraits::MPProblemType    MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType   GraphType;
    typedef typename MPProblemType::VID         VID;

    ///@}
    ///\name Construction
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Construct an empty path.
    /// \param[in] _r The roadmap used by this path.
    Path(MPProblemType* _p, RoadmapType* _r) : m_problem(_p), m_roadmap(_r) { }

    ///@}
    ///\name Path Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Get the roadmap used by this path.
    RoadmapType* GetRoadmap() const {return m_roadmap;}

    ////////////////////////////////////////////////////////////////////////////
    /// \brief The number of cfgs in the path.
    size_t Size() const {return m_vids.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// \brief The total edge weight.
    double Length() const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Get the VIDs in the path.
    const vector<VID>& VIDs() const {return m_vids;}

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Get a copy of the Cfgs in the path.
    /// \warning If the cfgs in the roadmap are later altered (i.e., if the DOF
    ///          values or labels are edited), this copy will be out-of-date.
    const vector<CfgType>& Cfgs() const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Get the current full Cfg path with steps spaced one environment
    ///        resolution apart. This is not cached due to its size and
    ///        infrequent usage.
    /// \param[in] _lp The local planner to use when connecting cfgs.
    /// \return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes.
    const vector<CfgType> FullCfgs(const string& _lp = "") const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Append another path to the end of this one.
    /// \param[in] _p The path to append.
    Path& operator+=(const Path& _p);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Add another path to the end of this one and return the result.
    /// \param[in] _p The path to add.
    Path operator+(const Path& _p) const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Append a new set of VIDs to the end of this path.
    /// \param[in] _vids The VIDs to append.
    Path& operator+=(const vector<VID>& _vids);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Add a new set of VIDs to the end of this path and return the
    ///        result.
    /// \param[in] _vids The VIDs to add.
    Path operator+(const vector<VID>& _vids) const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Clear all data in the path.
    void Clear();

    ///@}

  private:

    ///\name Helpers
    ///@{

    void AssertSameMap(const Path& _p) const;

    ///@}
    ///\name Internal State
    ///@{

    MPProblemType* const m_problem;     ///< The associated MPProblem.
    RoadmapType* const m_roadmap;       ///< The roadmap.
    vector<VID> m_vids;                 ///< The vids of the path configurations.

    vector<CfgType> m_cfgs;             ///< The path configurations.
    mutable bool m_cfgsCached{false};   ///< Are the current cfgs correct?

    double m_length{0};                 ///< The path length.
    mutable bool m_lengthCached{false}; ///< Is the current path length correct?

    ///@}
};

/*---------------------------- Path Interface --------------------------------*/

template <typename MPTraits>
double
Path<MPTraits>::
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
const vector<typename MPTraits::CfgType>&
Path<MPTraits>::
Cfgs() const {
  if(!m_cfgsCached) {
    vector<CfgType>& cfgs = const_cast<vector<CfgType>&>(m_cfgs);
    cfgs.clear();
    cfgs.reserve(m_vids.size());
    for(const auto& vid : m_vids)
      cfgs.push_back(m_roadmap->GetGraph()->GetVertex(vid));
    m_cfgsCached = true;
  }
  return m_cfgs;
}


template <typename MPTraits>
const vector<typename MPTraits::CfgType>
Path<MPTraits>::
FullCfgs(const string& _lp) const {
  GraphType* g = m_roadmap->GetGraph();
  vector<CfgType> out = {g->GetVertex(m_vids.front())};

  // Set up local planner to recreate edges. If none was provided, use edge
  // planner, or fall back to straight-line.
  auto env = m_problem->GetEnvironment();

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
    typename MPTraits::MPProblemType::LocalPlannerPointer lp;
    if(!_lp.empty())
      lp = m_problem->GetLocalPlanner(_lp);
    else {
      try {
        lp = m_problem->GetLocalPlanner(ei->property().GetLPLabel());
      }
      catch(...) {
        lp = m_problem->GetLocalPlanner("sl");
      }
    }

    // Recreate this edge, including intermediates.
    CfgRef start = g->GetVertex(*it);
    CfgRef end   = g->GetVertex(*(it+1));
    vector<CfgType> recreatedEdge = ei->property().GetIntermediates();
    recreatedEdge.insert(recreatedEdge.begin(), start);
    recreatedEdge.push_back(end);

    // Construct a resolution-level path along the recreated edge.
    for(auto cit = recreatedEdge.begin(); cit + 1 != recreatedEdge.end(); ++cit) {
      vector<CfgType> edge = lp->ReconstructPath(*cit, *(cit+1),
          vector<CfgType>(), env->GetPositionRes(), env->GetOrientationRes());
      out.insert(out.end(), edge.begin(), edge.end());
    }
    out.push_back(end);
  }
  return out;
}


template <typename MPTraits>
Path<MPTraits>&
Path<MPTraits>::
operator+=(const Path& _p) {
  AssertSameMap(_p);
  return *this += _p.m_vids;
}


template <typename MPTraits>
Path<MPTraits>
Path<MPTraits>::
operator+(const Path& _p) const {
  AssertSameMap(_p);
  return *this + _p.m_vids;
}


template <typename MPTraits>
Path<MPTraits>&
Path<MPTraits>::
operator+=(const vector<VID>& _vids) {
  std::copy(_vids.begin(), _vids.end(), back_inserter(m_vids));
  m_lengthCached = false;
  m_cfgsCached = false;
  return *this;
}


template <typename MPTraits>
Path<MPTraits>
Path<MPTraits>::
operator+(const vector<VID>& _vids) const {
  Path out(*this);
  out += _vids;
  return out;
}


template <typename MPTraits>
void
Path<MPTraits>::
Clear() {
  m_lengthCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
  m_vids.clear();
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
Path<MPTraits>::
AssertSameMap(const Path& _p) const {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE, "Can't add paths from different roadmaps!");
}

/*----------------------------------------------------------------------------*/

#endif
