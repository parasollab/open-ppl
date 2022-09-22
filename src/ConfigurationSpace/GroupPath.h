#ifndef PMPL_GROUP_PATH_H_
#define PMPL_GROUP_PATH_H_

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
class GroupPath final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType       GroupCfg;
    typedef typename MPTraits::GroupRoadmapType   GroupRoadmapType;
    typedef typename GroupRoadmapType::VID        VID;

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

    size_t TimeSteps() const;

    /// Get the VIDs in the path.
    const std::vector<VID>& VIDs() const noexcept;

		/// Get the VIDs and timesteps waiting in the path.
		const std::pair<std::vector<VID>,std::vector<size_t>> VIDsWaiting() const noexcept;

    /// Get a copy of the Cfgs in the path.
    /// @warning If the cfgs in the roadmap are later altered (i.e., if the DOF
    ///          values or labels are edited), this copy will be out-of-date.
    const std::vector<GroupCfg>& Cfgs() const;

    /// Get the current full Cfg path with steps spaced one environment
    ///        resolution apart. This is not cached due to its size and
    ///        infrequent usage.
    /// @param _lib The planning library to use.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes.
    template <typename MPLibrary>
    const std::vector<GroupCfg> FullCfgs(MPLibrary* const _lib) const;

    /// Get the current full Cfg path with wait times. Steps are spaced one
    /// environment resolution apart. This is not cached due to its size and
    /// infrequent usage.
    /// @param _lib The planning library to use.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes and waiting.
    template <typename MPLibrary>
    const std::vector<GroupCfg> FullCfgsWithWait(MPLibrary* const _lib) const;

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

    /// Set the wait times at each vertex in path
    /// Used in Safe Interval Path Planning
    void SetWaitTimes(std::vector<size_t> _waitTimes);
  
    /// Get the wait times at each vertex in the path.
    std::vector<size_t> GetWaitTimes();

    /// Get the (source,target) of the path at the input timestep.
    /// @param _timestep The timestep to find the corresponding edge.
    /// @return  The source and target of the corresponding edge.
    std::pair<size_t,size_t> GetEdgeAtTimestep(size_t _timestep);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Enforce that both group paths are using the same roadmap.
    /// @param _p The group path to check against.
    void AssertSameMap(const GroupPath& _p) const;

    ///@}
    ///@name Internal State
    ///@{

    GroupRoadmapType* const m_roadmap;     ///< The roadmap.
    std::vector<VID> m_vids;               ///< The vids of the path configurations.
    std::vector<size_t> m_waitingTimesteps; ///< The number of timesteps to wait at each vid.
		size_t m_finalWaitTimeSteps{0}; ///< Temp - need to move this logic into the waiting timesteps.

    mutable std::vector<GroupCfg> m_cfgs;  ///< The path configurations.
    mutable bool m_cfgsCached{false};      ///< Are the current cfgs correct?

    mutable double m_length{0};            ///< The path length.
    mutable bool m_lengthCached{false};    ///< Is the current path length correct?

    mutable double m_timesteps{0};            ///< The number of timesteps.
    mutable bool m_timestepsCached{false};    ///< Is the current path number of timesteps correct?
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
size_t
GroupPath<MPTraits>::
TimeSteps() const {
  // If the length is cached, we don't need to recompute.
  if(m_timestepsCached)
    return m_timesteps;
  m_timestepsCached = true;

  // Recompute the length by summing the edge weights.
  m_timesteps = 0;
  for(auto start = m_vids.begin(); start + 1 < m_vids.end(); ++start) {
    // Skip repeated vertices.
    /// @todo This will be an error if we allow self-edges.
    if(*start == *(start + 1))
      continue;

    // Add this edge's weight to the sum.
    const auto& edge = m_roadmap->GetEdge(*start, *(start + 1));
    m_timesteps += edge.GetTimeSteps();
  }

  for(auto w : m_waitingTimesteps) {
    m_timesteps += w;
  }

  return m_timesteps;
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
  // If the cfgs are cached, we don't need to recompute.
  if(m_cfgsCached)
    return m_cfgs;
  m_cfgsCached = true;

  m_cfgs.clear();
  m_cfgs.reserve(m_vids.size());
  for(const auto& vid : m_vids)
    m_cfgs.push_back(m_roadmap->GetVertex(vid));

  return m_cfgs;
}


template <typename MPTraits>
template <typename MPLibrary>
const std::vector<typename MPTraits::GroupCfgType>
GroupPath<MPTraits>::
FullCfgs(MPLibrary* const _lib) const {
  if(m_vids.empty())
    return std::vector<GroupCfg>();

  // Insert the first vertex.
  std::vector<GroupCfg> out = {m_roadmap->GetVertex(m_vids.front())};

  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    const VID source = *it,
              target = *(it + 1);
    const auto& edge = m_roadmap->GetEdge(source, target);

    // Insert intermediates between vertices. For assembly planning (skip edge):
    // don't reconstruct the edge when it's for a part that has been placed off
    // to the side, just use the two cfgs. This edge will just be (start, end).
    if(!edge.SkipEdge()) {
      auto e = m_roadmap->GetEdge(source,target);
      auto edge = !e.GetIntermediates().empty() ? e.GetIntermediates()
                                              : _lib->ReconstructEdge(m_roadmap, source, target);

      if(!edge.empty()) {
        // Only grab the intermediate cfgs.
        auto startIter = edge.begin();

        auto endIter = edge.end();

        out.insert(out.end(), startIter, endIter);
      }
    }
  
    // Insert the next vertex.
    out.push_back(m_roadmap->GetVertex(target));
  }
  return out;
}

template <typename MPTraits>
template <typename MPLibrary>
const std::vector<typename MPTraits::GroupCfgType>
GroupPath<MPTraits>::
FullCfgsWithWait(MPLibrary* const _lib) const {
  if(m_vids.empty())
    return std::vector<GroupCfg>();

  if(m_waitingTimesteps.empty())
    return FullCfgs(_lib);

  // Insert the first vertex.
  size_t vid = m_vids.front();
  GroupCfg vertex = m_roadmap->GetVertex(vid);
  std::vector<GroupCfg> out = {vertex};

  // Insert first vertex however many timesteps it waits
  //for(size_t i = 0; i < m_waitingTimesteps[0]; ++i) {
  for(size_t i = 0; i < m_waitingTimesteps[0]; ++i) {
    out.push_back(vertex);
  }

  auto cnt = 1;
  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    // Insert intermediates between vertices.

    //std::vector<GroupCfg> edge = _lib->ReconstructEdge(m_roadmap, *it, *(it+1));
    auto e = m_roadmap->GetEdge(*it,*(it+1));
    auto edge = !e.GetIntermediates().empty() ? e.GetIntermediates()
                                              : _lib->ReconstructEdge(m_roadmap,*it,*(it+1));
    out.insert(out.end(), edge.begin(), edge.end());

    // Insert the next vertex.
    vid = *(it+1);
    vertex = m_roadmap->GetVertex(vid);
    out.push_back(vertex);
    for(size_t i = 0; i < m_waitingTimesteps[cnt]; ++i) {
      out.push_back(vertex);
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
    FlushCache();
    std::copy(_vids.begin(), _vids.end(), std::back_inserter(m_vids));
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
    throw RunTimeException(WHERE) << "Can't assign path from another roadmap";

  m_vids         = _p.m_vids;
  m_cfgs         = _p.m_cfgs;
  m_cfgsCached   = _p.m_cfgsCached;
  m_length       = _p.m_length;
  m_lengthCached = _p.m_lengthCached;
  m_timestepsCached = _p.m_timestepsCached;
  m_waitingTimesteps = _p.m_waitingTimesteps;

  return *this;
}


template <typename MPTraits>
void
GroupPath<MPTraits>::
Clear() {
  FlushCache();
  m_vids.clear();
  m_waitingTimesteps.clear();
}


template <typename MPTraits>
void
GroupPath<MPTraits>::
FlushCache() {
  m_lengthCached = false;
  m_timestepsCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
}

template <typename MPTraits>
void
GroupPath<MPTraits>::
SetWaitTimes(std::vector<size_t> _waitTimes) {
  m_waitingTimesteps = _waitTimes;

  // Check to make sure that size matches path length
  if(m_waitingTimesteps.size() != m_vids.size()) {
    throw RunTimeException(WHERE) << "Size of timestep vector does not "
                                  << "match path length";
  }
}

template <typename MPTraits>
std::vector<size_t>
GroupPath<MPTraits>::
GetWaitTimes() {
  return m_waitingTimesteps;
}

template <typename MPTraits>
std::pair<size_t,size_t>
GroupPath<MPTraits>::
GetEdgeAtTimestep(size_t _timestep) {
 
  size_t step = 0;
  for(size_t i = 0; i + 1 < m_vids.size(); i++) {

    if(!m_waitingTimesteps.empty())
      step += m_waitingTimesteps[i];
    
    if(_timestep <= step)
      return std::make_pair(m_vids[i],m_vids[i]);

    auto duration = m_roadmap->GetEdge(m_vids[i],m_vids[i+1]).GetTimeSteps();

    step += duration;

    if(_timestep <= step)
      return std::make_pair(m_vids[i],m_vids[i+1]);
  }

  return std::make_pair(m_vids.back(),m_vids.back());
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
GroupPath<MPTraits>::
AssertSameMap(const GroupPath& _p) const {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE) << "Can't add paths from different roadmaps "
                                  << "(source = " << _p.m_roadmap << ","
                                  << " target = " << m_roadmap << ").";
}

/*----------------------------------------------------------------------------*/

#endif
