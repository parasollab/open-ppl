#ifndef PMPL_PATH_H_
#define PMPL_PATH_H_

#include "MPLibrary/MPLibrary.h"
#include "Utilities/PMPLExceptions.h"

#include <algorithm>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A path of connected configurations from a given roadmap.
///
/// The implementation uses a vector of VID's as the primary representation.
/// The corresponding configurations are computed lazily upon request.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PathType final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::RoadmapType   RoadmapType;
    typedef typename RoadmapType::VID        VID;


    ///@}
    ///@name Construction
    ///@{

    /// Construct an empty path.
    /// @param _r The roadmap used by this path.
    PathType(RoadmapType* const _r = nullptr);

    ///@}
    ///@name Path Interface
    ///@{

    /// Get the robot which travels this path.
    Robot* GetRobot() const noexcept;

    /// Get the roadmap used by this path.
    RoadmapType* GetRoadmap() const noexcept;

    /// Get the number of cfgs in the path.
    size_t Size() const noexcept;

    /// Check if the path is empty.
    bool Empty() const noexcept;

    /// Get the total edge weight.
    double Length() const;

    /// Get the VIDs in the path.
    const std::vector<VID>& VIDs() const noexcept;

    /// Set the VIDs in the path.
    void SetVIDs(std::vector<VID>);

		/// Get the VIDs and timesteps waiting in the path.
		const std::pair<std::vector<VID>,std::vector<size_t>> VIDsWaiting() const noexcept;

    /// Get the start VID in the path.
    const size_t& StartVID() const noexcept;
    /// Get the goal VID in the path.
    const size_t& GoalVID() const noexcept;

    /// Get a copy of the Cfgs in the path.
    /// @warning If the cfgs in the roadmap are later altered (i.e., if the DOF
    ///          values or labels are edited), this copy will be out-of-date.
    const std::vector<CfgType>& Cfgs() const;

    /// Get the current full Cfg path with steps spaced one environment
    ///        resolution apart. This is not cached due to its size and
    ///        infrequent usage.
    /// @param _lib The planning library to use.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes.
    template <typename MPLibrary>
    const std::vector<CfgType> FullCfgs(MPLibrary* const _lib) const;

    /// Similar to FullCfgs function, constructs a full cfg path from edges
    ///       with different resolution sizes.
    /// @param _lib The planning library to use.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes.
    template <typename MPLibrary>
    const std::vector<CfgType> FullCfgsWithDurations(MPLibrary* const _lib) const;


    /// Get the current full Cfg path with wait times. Steps are spaced one
    /// environment resolution apart. This is not cached due to its size and
    /// infrequent usage.
    /// @param _lib The planning library to use.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes and waiting.
    template <typename MPLibrary>
    const std::vector<CfgType> FullCfgsWithWait(MPLibrary* const _lib) const;

    /// Get the number of timesteps calculated to traverse the path,
    /// including waiting times.
    size_t TimeSteps() const;

    /// Hardcode number of timesteps in the path.
    void SetTimeSteps(size_t _timesteps);

    /// Reset the number of timesteps to 0.
    void ResetTimeSteps();

    /// Append another path to the end of this one.
    /// @param _p The path to append.
    PathType& operator+=(const PathType& _p);

    /// Add another path to the end of this one and return the result.
    /// @param _p The path to add.
    PathType operator+(const PathType& _p) const;

    /// Append a new set of VIDs to the end of this path.
    /// @param _vids The VIDs to append.
    PathType& operator+=(const std::vector<VID>& _vids);

    /// Append a new set of VIDs (with waiting timesteps) to the end of this path.
    /// @param _waitingVIDS The waiting VIDs to append.
    //PathType& operator+=(const std::pair<std::vector<VID>,std::vector<size_t>>& _waitingVIDs);

    /// Add a new set of VIDs to the end of this path and return the result.
    /// @param _vids The VIDs to add.
    PathType operator+(const std::vector<VID>& _vids) const;

    /// Copy assignment operator.
    PathType& operator=(const PathType& _p);

    /// Clear all data in the path.
    void Clear();

    /// Clear cached data, but leave the VIDs.
    void FlushCache();

    /// Set the number of timesteps to wait after traversal of the path.
    /// @param _timeSteps The number of desired timesteps.
    void SetFinalWaitTimeSteps(const size_t& _timeSteps);

    /// Get the number of timesteps to wait after traversal of the path.
    const size_t GetFinalWaitTimeSteps() const;

    /// Set the wait times at each vertex in path
    /// Used in Safe Interval Path Planning
    void SetWaitTimes(std::vector<size_t> _waitTimes);

    /// Get the wait time at each vertex index in the path.
    std::vector<size_t> GetWaitTimes();

    /// Set the durations of each edge in the path
    /// Used to extract individual paths from composite paths
    void SetDurations(std::vector<size_t> _durations);

    /// Get the duration time of each edge index in the path.
    std::vector<size_t> GetDurations();

    /// Get the (source,target) of the path at the input timestep.
    /// @param _timestep The timestep to find the corresponding edge.
    /// @return  The source and target of the corresponding edge.
    std::pair<std::pair<size_t,size_t>,std::pair<size_t,size_t>>
                        GetEdgeAtTimestep(size_t _timestep);

    ///@}

  private:

    ///@name Helpers
    ///@{

    void AssertSameMap(const PathType& _p) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    RoadmapType* const m_roadmap;        ///< The roadmap.
    mutable std::vector<VID> m_vids;             ///< The VIDs in the path.
    mutable std::vector<size_t> m_waitingTimesteps; ///< The number of timesteps to wait at each vid.i
    mutable std::vector<size_t> m_durations; ///< The duration of each edge in the path.


    mutable std::vector<CfgType> m_cfgs; ///< The path configurations.
    mutable bool m_cfgsCached{false};    ///< Are the current cfgs correct?

    mutable double m_length{0};          ///< The path length.
    mutable bool m_lengthCached{false};  ///< Is the current length correct?

		size_t m_finalWaitTimeSteps{0}; ///< Temp - need to move this logic into the waiting timesteps.

    mutable size_t m_timesteps{0};
    mutable bool m_timestepsCached{false};

    //std::vector<FlexibleEdge> m_edges; ///< A path represented as a sequence of flexible edges.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
PathType<MPTraits>::
PathType(RoadmapType* const _r) : m_roadmap(_r) { }

/*------------------------------ Path Interface ------------------------------*/

template <typename MPTraits>
Robot*
PathType<MPTraits>::
GetRobot() const noexcept {
  return m_roadmap->GetRobot();
}


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
bool
PathType<MPTraits>::
Empty() const noexcept {
  return m_vids.empty();
}


template <typename MPTraits>
double
PathType<MPTraits>::
Length() const {
  // If the length is cached, we don't need to recompute.
  if(m_lengthCached)
    return m_length;
  m_lengthCached = true;

  // Recompute the length by summing the edge weights.
  m_length = 0;
  for(auto start = m_vids.begin(); start + 1 < m_vids.end(); ++start) {
    // Skip repeated vertices.
    /// @todo This will be an error if we allow self-edges.
    if(*start == *(start + 1))
      continue;

    // Add this edge's weight to the sum.
    const auto& edge = m_roadmap->GetEdge(*start, *(start + 1));
    m_length += edge.GetWeight();
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
void
PathType<MPTraits>::
SetVIDs(std::vector<typename PathType<MPTraits>::VID> _vids)  {
  m_vids = _vids;
}


template <typename MPTraits>
const size_t&
PathType<MPTraits>::
StartVID() const noexcept {
  return m_vids.front();
}


template <typename MPTraits>
const size_t&
PathType<MPTraits>::
GoalVID() const noexcept {
  return m_vids.back();
}


template <typename MPTraits>
const std::pair<std::vector<typename PathType<MPTraits>::VID>,std::vector<size_t>>
PathType<MPTraits>::
VIDsWaiting() const noexcept {
	return std::make_pair(m_vids,m_waitingTimesteps);
}

template <typename MPTraits>
const std::vector<typename MPTraits::CfgType>&
PathType<MPTraits>::
Cfgs() const {
  // If the cfgs are cached, we don't need to recompute.
  if(m_cfgsCached)
    return m_cfgs;
  m_cfgsCached = true;

  m_cfgs.clear();
  m_cfgs.reserve(m_vids.size());
  if(!m_vids.empty()) {
    for(const auto& vid : m_vids) {
      m_cfgs.push_back(m_roadmap->GetVertex(vid));
    }
  }
  return m_cfgs;
}


template <typename MPTraits>
template <typename MPLibrary>
const std::vector<typename MPTraits::CfgType>
PathType<MPTraits>::
FullCfgs(MPLibrary* const _lib) const {
  if(!m_durations.empty())
    return FullCfgsWithDurations(_lib);
  if(!m_waitingTimesteps.empty())
    return FullCfgsWithWait(_lib);
  if(m_vids.empty())
    return std::vector<CfgType>();

  // Insert the first vertex.
  std::vector<CfgType> out = {m_roadmap->GetVertex(m_vids.front())};

  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    // Insert intermediates between vertices.
    std::vector<CfgType> edge = _lib->ReconstructEdge(m_roadmap, *it, *(it+1));
    out.insert(out.end(), edge.begin(), edge.end());

    //Insert the next vertex.
    out.push_back(m_roadmap->GetVertex(*(it+1)));
  }
	auto last = out.back();
	for(size_t i = 0; i < m_finalWaitTimeSteps; i++) {
		out.push_back(last);
	}
  return out;
}

template <typename MPTraits>
template <typename MPLibrary>
const std::vector<typename MPTraits::CfgType>
PathType<MPTraits>::
FullCfgsWithDurations(MPLibrary* const _lib) const {

  // Insert the first vertex.
  auto vertex = m_roadmap->GetVertex(m_vids[0]);
  std::vector<CfgType> out = {vertex};

  // Insert first vertex however many timesteps it waits
  //for(size_t i = 0; i < m_waitingTimesteps[0]; ++i) {
  if(!m_waitingTimesteps.empty()) {
    for(size_t i = 0; i < m_waitingTimesteps[0]; ++i) {
      out.push_back(vertex);
    }
  }

  // Insert intermediates between vertices. If the number of steps is specified
  // we will use a specific ReconstructEdge function, if not we go for the default function.
  auto cnt = 1;

  for(size_t i = 0 ; i < m_durations.size() ; ++i) {
    std::vector<CfgType> edge;
    if(m_durations[i] != std::numeric_limits<size_t>::infinity()) {
      edge = _lib->ReconstructEdge(m_roadmap, m_vids[i],m_vids[i+1] , m_durations[i]);
    }
    else if(m_vids[i] == m_vids[i+1]) {
      auto v = m_roadmap->GetVertex(m_vids[i]);
      for(size_t i = 0; i < m_durations[i]; i++) {
        edge.push_back(v);
      }
    }
    else {
      edge = _lib->ReconstructEdge(m_roadmap, m_vids[i], m_vids[i+1]);
    }

    out.insert(out.end(), edge.begin()+1, edge.end());
    //Insert the next vertex.
    vertex = m_roadmap->GetVertex(m_vids[i+1]);
    //out.push_back(vertex);

    if(!m_waitingTimesteps.empty()) {
      if((size_t) cnt < m_waitingTimesteps.size()) {
        for(size_t i = 0; i < m_waitingTimesteps[cnt]; ++i) {
          out.push_back(vertex);
        }
      }
      cnt++;
    }
  }
  return out;
}


template <typename MPTraits>
template <typename MPLibrary>
const std::vector<typename MPTraits::CfgType>
PathType<MPTraits>::
FullCfgsWithWait(MPLibrary* const _lib) const {
  if(m_waitingTimesteps.empty())
    return FullCfgs(_lib);
  // Insert the first vertex.
  size_t vid = m_vids.front();
  CfgType vertex = m_roadmap->GetVertex(vid);
  std::vector<CfgType> out = {vertex};

  // Insert first vertex however many timesteps it waits
  //for(size_t i = 0; i < m_waitingTimesteps[0]; ++i) {
  for(size_t i = 0; i < m_waitingTimesteps[0]; ++i) {
    out.push_back(vertex);
  }

  auto cnt = 1;
  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    // Insert intermediates between vertices.

    std::vector<CfgType> edge = _lib->ReconstructEdge(m_roadmap, *it, *(it+1));
    out.insert(out.end(), edge.begin(), edge.end());

    // Insert the next vertex.
    vid = *(it+1);
    vertex = m_roadmap->GetVertex(vid);
    out.push_back(vertex);
    if((size_t) cnt < m_waitingTimesteps.size()) {
      for(size_t i = 0; i < m_waitingTimesteps[cnt]; ++i) {
        out.push_back(vertex);
      }
    }
    cnt++;
  }

  auto last = out.back();
  for(size_t i = 0; i < m_finalWaitTimeSteps; i++) {
    out.push_back(last);
  }
  return out;
}



template <typename MPTraits>
size_t
PathType<MPTraits>::
TimeSteps() const {

  //if(m_timestepsCached)
  //  return m_timesteps;
  m_timestepsCached = true;
  m_timesteps = 0;

  // calculate timesteps using m_vids
  for(size_t i = 0; i < m_vids.size()-1; i++) {
    if(!m_durations.empty() and
      m_durations[i] != std::numeric_limits<size_t>::infinity()) {
      m_timesteps += m_durations[i];
    } else {
        auto edge = m_roadmap->GetEdge(m_vids[i], m_vids[i+1]);
        m_timesteps += edge.GetTimeSteps();
    }

    if(m_vids[i] == m_vids[i+1]) {
            m_timesteps++;
            continue;
    }

    if(!m_waitingTimesteps.empty())
      m_timesteps += m_waitingTimesteps[i];

  }

  if(!m_waitingTimesteps.empty())
    m_timesteps += m_waitingTimesteps.back();

  return m_timesteps;
}

template <typename MPTraits>
void
PathType<MPTraits>::
SetTimeSteps(size_t _timesteps) {
  m_timestepsCached = true;
  m_timesteps = _timesteps;
}


template <typename MPTraits>
void
PathType<MPTraits>::
ResetTimeSteps() {
  m_timestepsCached = false;
}

template <typename MPTraits>
PathType<MPTraits>&
PathType<MPTraits>::
operator+=(const PathType& _p) {
  AssertSameMap(_p);

  // Durations can be merged with no issue, but we have to handle the cases when
  // edges have timesteps or duartions.
  if(m_durations.empty() and !_p.m_durations.empty())
    m_durations = std::vector<size_t>(m_vids.size()-1, std::numeric_limits<size_t>::infinity());
  if (!m_durations.empty() and _p.m_durations.empty())
    _p.m_durations = std::vector<size_t>(_p.m_vids.size()-1, std::numeric_limits<size_t>::infinity());
  if (!m_durations.empty() or! _p.m_durations.empty())
    m_durations.insert(m_durations.end(), _p.m_durations.begin(), _p.m_durations.end());

  // VIDS will be merged, but we have to remove the repeated VID
  if(!m_vids.empty()) {
    if(m_vids.back() != _p.m_vids.front())
      throw RunTimeException(WHERE) << "end VID of path1 does not match start VID of path2";
    m_vids.insert(m_vids.end(), _p.m_vids.begin()+1, _p.m_vids.end());
    m_waitingTimesteps.back() += _p.m_waitingTimesteps.front();
    m_waitingTimesteps.insert(m_waitingTimesteps.end(), _p.m_waitingTimesteps.begin()+1, _p.m_waitingTimesteps.end());
  } else {
    m_vids.insert(m_vids.end(), _p.m_vids.begin(), _p.m_vids.end());
    m_waitingTimesteps.insert(m_waitingTimesteps.end(), _p.m_waitingTimesteps.begin(), _p.m_waitingTimesteps.end());
  }

  // Waiting timesteps be merged similarly to vids, but we have to add the
  // timesteps of the repated vid
  // if(m_vids.empty()) 
  //   m_waitingTimesteps = _p.m_waitingTimesteps;
  // else {   
  //   if(!m_waitingTimesteps.empty() and !_p.m_waitingTimesteps.empty())
  //     m_waitingTimesteps.back() += _p.m_waitingTimesteps.front();
  //   m_waitingTimesteps.insert(m_waitingTimesteps.end(), _p.m_waitingTimesteps.begin()+1, _p.m_waitingTimesteps.end());
  // }
  //m_cfgs.insert(m_cfgs.end(), _p.m_cfgs.begin()+1, _p.m_cfgs.end());

  //m_vids             += _p.m_vids;
  //m_waitingTimesteps += _p.m_waitingTimesteps;
  //m_durations        += _p.m_durations;
  //m_cfgs             += _p.m_cfgs;
  m_cfgsCached       += _p.m_cfgsCached;
  m_length           += _p.m_length;
  m_lengthCached     += _p.m_lengthCached;

  return *this;
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
  if(_vids.size()) {
    FlushCache();

    // // if the local path is generated by decoupled,
    // // i.e, given by vids we need to make sure there's no duplicate vids
    // std::vector<VID> vids = _vids;
    // if(m_vids.size()) {
    //   if(m_vids.back() == vids.front())
    //     vids.erase(vids.begin());
    // }

    std::copy(_vids.begin(), _vids.end(), std::back_inserter(m_vids));
  }
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
PathType<MPTraits>&
PathType<MPTraits>::
operator=(const PathType& _p) {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE) << "Can't assign path from another roadmap";

  m_vids             = _p.m_vids;
  m_durations        = _p.m_durations;
  m_waitingTimesteps = _p.m_waitingTimesteps;
  m_cfgs             = _p.m_cfgs;
  m_cfgsCached       = _p.m_cfgsCached;
  m_length           = _p.m_length;
  m_lengthCached     = _p.m_lengthCached;

  return *this;
}


template <typename MPTraits>
void
PathType<MPTraits>::
Clear() {
  FlushCache();
  m_vids.clear();
  m_durations.clear();
  m_waitingTimesteps.clear();
}


template <typename MPTraits>
void
PathType<MPTraits>::
FlushCache() {
  m_lengthCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
}

template <typename MPTraits>
void
PathType<MPTraits>::
SetFinalWaitTimeSteps(const size_t& _timeSteps) {
	m_finalWaitTimeSteps = _timeSteps;
}

template <typename MPTraits>
const size_t
PathType<MPTraits>::
GetFinalWaitTimeSteps() const {
	return m_finalWaitTimeSteps;
}

template <typename MPTraits>
void
PathType<MPTraits>::
SetWaitTimes(std::vector<size_t> _waitTimes) {
  m_waitingTimesteps = _waitTimes;

  // Check to make sure that size matches path length
  if(!m_waitingTimesteps.empty() && !m_durations.empty()) {
    if(m_waitingTimesteps.size() != m_vids.size() and m_waitingTimesteps.size() != m_durations.size()+1) {
      throw RunTimeException(WHERE) << "Size of timestep vector does not "
                                    << "match path length";
    }
  }
}

template <typename MPTraits>
std::vector<size_t>
PathType<MPTraits>::
GetWaitTimes() {
  return m_waitingTimesteps;
}


template <typename MPTraits>
void
PathType<MPTraits>::
SetDurations(std::vector<size_t> _durations) {
  m_durations = _durations;

  // Check to make sure that size matches path length
  if(!m_durations.empty() && !m_vids.empty()) {
    if(m_durations.size() + 1 != m_vids.size()) {
      throw RunTimeException(WHERE) << "Size of durations vector has size "
                                    << m_durations.size() << ", which does not "
                                    << "match path length " << m_vids.size();
    }
  }
}

template <typename MPTraits>
std::vector<size_t>
PathType<MPTraits>::
GetDurations() {
  return m_durations;
}

template <typename MPTraits>
std::pair<std::pair<size_t,size_t>,std::pair<size_t,size_t>>
PathType<MPTraits>::
GetEdgeAtTimestep(size_t _timestep) {


  size_t step = 0;
  if(m_durations.empty()) {

    for(size_t i = 0; i + 1 < m_vids.size(); i++) {
      if(!m_waitingTimesteps.empty())
        step += m_waitingTimesteps[i];

      if(_timestep <= step) {
        auto vertex = std::make_pair(m_vids[i],m_vids[i]);
        size_t min = m_waitingTimesteps.empty() ? 0 : step - m_waitingTimesteps[i];
        auto interval = std::make_pair(min,step);
        return std::make_pair(vertex,interval);
      }

      auto duration = m_roadmap->GetEdge(m_vids[i],m_vids[i+1]).GetTimeSteps();

      step += duration;

      if(_timestep <= step) {
        auto edge = std::make_pair(m_vids[i],m_vids[i+1]);
        auto interval = std::make_pair(step-duration,step);
        return std::make_pair(edge,interval);
      }
    }

    auto vertex = std::make_pair(m_vids.back(),m_vids.back());
    auto interval = std::make_pair(step,SIZE_MAX);
    return std::make_pair(vertex,interval);
  }
  else {

    for(size_t i = 0; i < m_durations.size(); i++) {

      auto duration = m_durations[i];
      auto source = m_vids[i];
      auto edge = std::make_pair(m_vids[i],m_vids[i+1]);

      if(!m_waitingTimesteps.empty())
        step += m_waitingTimesteps[i];

      if(_timestep <= step) {
        auto vertex = std::make_pair(source,source);
        size_t min = m_waitingTimesteps.empty() ? 0 : step - m_waitingTimesteps[i];
        auto interval = std::make_pair(min,step);
        return std::make_pair(vertex,interval);
      }

      step += duration;

      if(_timestep <= step) {
        auto vertex = std::make_pair(edge.first,edge.first);
        auto interval = std::make_pair(step-duration,step);
        return std::make_pair(vertex,interval);
      }
    }

    auto vertex = std::make_pair(m_vids.back(),m_vids.back());
    auto interval = std::make_pair(step,SIZE_MAX);
    return std::make_pair(vertex,interval);
  }
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
inline
void
PathType<MPTraits>::
AssertSameMap(const PathType& _p) const noexcept {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE) << "Can't add paths from different roadmaps "
                                  << "(source = " << _p.m_roadmap << ","
                                  << " target = " << m_roadmap << ").";
}

/*----------------------------------------------------------------------------*/

#endif
