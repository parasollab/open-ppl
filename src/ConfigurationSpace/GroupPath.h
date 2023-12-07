#ifndef PMPL_GROUP_PATH_H_
#define PMPL_GROUP_PATH_H_

#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
// #include "MPLibrary/MPLibrary.h"
#include "Utilities/PMPLExceptions.h"

#include <algorithm>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A path of connected configurations from a given roadmap.
///
/// The implementation uses a vector of VID's as the primary representation.
/// The corresponding configurations are computed lazily upon request.
////////////////////////////////////////////////////////////////////////////////
class GroupPath final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef GenericStateGraph<Cfg, DefaultWeight<Cfg>> RoadmapType;
    typedef GroupCfg<RoadmapType> GroupCfgType;
    typedef GroupRoadmap<GroupCfgType, GroupLocalPlan<RoadmapType>> GroupRoadmapType;
    typedef typename GroupRoadmapType::VID VID;

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
    const std::vector<GroupCfgType>& Cfgs() const;

    /// Get the current full Cfg path with steps spaced one environment
    ///        resolution apart. This is not cached due to its size and
    ///        infrequent usage.
    /// @param _lib The planning library to use.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes.
    template <typename MPLibrary>
    const std::vector<GroupCfgType> FullCfgs(MPLibrary* const _lib) const;

    /// Get the current full Cfg path with wait times. Steps are spaced one
    /// environment resolution apart. This is not cached due to its size and
    /// infrequent usage.
    /// @param _lib The planning library to use.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes and waiting.
    template <typename MPLibrary>
    const std::vector<GroupCfgType> FullCfgsWithWait(MPLibrary* const _lib) const;

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

    mutable std::vector<GroupCfgType> m_cfgs;  ///< The path configurations.
    mutable bool m_cfgsCached{false};      ///< Are the current cfgs correct?

    mutable double m_length{0};            ///< The path length.
    mutable bool m_lengthCached{false};    ///< Is the current path length correct?

    mutable double m_timesteps{0};            ///< The number of timesteps.
    mutable bool m_timestepsCached{false};    ///< Is the current path number of timesteps correct?
    ///@}
};


template <typename MPLibrary>
const std::vector<typename GroupPath::GroupCfgType>
GroupPath::
FullCfgs(MPLibrary* const _lib) const {
  if(m_vids.empty())
    return std::vector<GroupCfgType>();

  // Insert the first vertex.
  std::vector<GroupCfgType> out = {m_roadmap->GetVertex(m_vids.front())};

  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    const VID source = *it,
              target = *(it + 1);
    // const auto& edge = m_roadmap->GetEdge(source, target);

    // Insert intermediates between vertices. For assembly planning (skip edge):
    // don't reconstruct the edge when it's for a part that has been placed off
    // to the side, just use the two cfgs. This edge will just be (start, end).
    // if(!edge.SkipEdge()) {
      auto e = m_roadmap->GetEdge(source,target);
      auto edge = !e.GetIntermediates().empty() ? e.GetIntermediates()
                                              : _lib->ReconstructEdge(m_roadmap, source, target);

      if(!edge.empty()) {
        // Only grab the intermediate cfgs.
        auto startIter = edge.begin();

        auto endIter = edge.end();

        out.insert(out.end(), startIter, endIter);
      }
    // }
  
    // Insert the next vertex.
    out.push_back(m_roadmap->GetVertex(target));
  }
  return out;
}


template <typename MPLibrary>
const std::vector<typename GroupPath::GroupCfgType>
GroupPath::
FullCfgsWithWait(MPLibrary* const _lib) const {
  if(m_vids.empty())
    return std::vector<GroupCfgType>();

  if(m_waitingTimesteps.empty())
    return FullCfgs(_lib);

  // Insert the first vertex.
  size_t vid = m_vids.front();
  GroupCfgType vertex = m_roadmap->GetVertex(vid);
  std::vector<GroupCfgType> out = {vertex};

  // Insert first vertex however many timesteps it waits
  //for(size_t i = 0; i < m_waitingTimesteps[0]; ++i) {
  for(size_t i = 0; i < m_waitingTimesteps[0]; ++i) {
    out.push_back(vertex);
  }

  auto cnt = 1;
  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    // Insert intermediates between vertices.

    //std::vector<GroupCfgType> edge = _lib->ReconstructEdge(m_roadmap, *it, *(it+1));
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

#endif
