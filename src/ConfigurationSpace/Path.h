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

		/// Get the VIDs and timesteps waiting in the path.
		const std::pair<std::vector<VID>,std::vector<size_t>> VIDsWaiting() const noexcept;		

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

		size_t TimeSteps() const;

    void SetTimeSteps(size_t _timesteps);

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

    /// Add a new set of VIDs to the end of this path and return the result.
    /// @param _vids The VIDs to add.
    PathType operator+(const std::vector<VID>& _vids) const;

    /// Copy assignment operator.
    PathType& operator=(const PathType& _p);

    /// Clear all data in the path.
    void Clear();

    /// Clear cached data, but leave the VIDs.
    void FlushCache();

		void SetFinalWaitTimeSteps(const size_t& _timeSteps);

		const size_t GetFinalWaitTimeSteps() const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    void AssertSameMap(const PathType& _p) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    RoadmapType* const m_roadmap;        ///< The roadmap.
    std::vector<VID> m_vids;             ///< The VIDs in the path.
		std::vector<size_t> m_waitingTimesteps; ///< The number of timesteps to wait at each vid.

    mutable std::vector<CfgType> m_cfgs; ///< The path configurations.
    mutable bool m_cfgsCached{false};    ///< Are the current cfgs correct?

    mutable double m_length{0};          ///< The path length.
    mutable bool m_lengthCached{false};  ///< Is the current length correct?

		size_t m_finalWaitTimeSteps{0}; ///< Temp - need to move this logic into the waiting timesteps.

    mutable size_t m_timesteps{0};
    mutable bool m_timestepsCached{false};

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
  for(const auto& vid : m_vids)
    m_cfgs.push_back(m_roadmap->GetVertex(vid));

  return m_cfgs;
}


template <typename MPTraits>
template <typename MPLibrary>
const std::vector<typename MPTraits::CfgType>
PathType<MPTraits>::
FullCfgs(MPLibrary* const _lib) const {
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
size_t
PathType<MPTraits>::
TimeSteps() const {

  if(m_timestepsCached)
    return m_timesteps;

  m_timestepsCached = true;

  if(m_vids.empty())
    return 0;

	m_timesteps = 0;

  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
		if(*it == *(it+1)) {
			m_timesteps++;
			continue;
		}
		auto edge = m_roadmap->GetEdge(*it, *(it+1));
		m_timesteps += edge.GetTimeSteps();
  }
	m_timesteps += m_finalWaitTimeSteps;
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
  if(_vids.size()) {
    FlushCache();
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

  m_vids         = _p.m_vids;
  m_cfgs         = _p.m_cfgs;
  m_cfgsCached   = _p.m_cfgsCached;
  m_length       = _p.m_length;
  m_lengthCached = _p.m_lengthCached;

  return *this;
}


template <typename MPTraits>
void
PathType<MPTraits>::
Clear() {
  FlushCache();
  m_vids.clear();
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
