#ifndef PMPL_PATH_H_
#define PMPL_PATH_H_

#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"

// #include "MPLibrary/MPLibrary.h"
#include "Utilities/PMPLExceptions.h"

// #include <algorithm>
// #include <vector>

class MPLibrary;


////////////////////////////////////////////////////////////////////////////////
/// A path of connected configurations from a given roadmap.
///
/// The implementation uses a vector of VID's as the primary representation.
/// The corresponding configurations are computed lazily upon request.
////////////////////////////////////////////////////////////////////////////////
class Path final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef GenericStateGraph<Cfg, DefaultWeight<Cfg>> RoadmapType;
    typedef typename RoadmapType::VID        VID;

    ///@}
    ///@name Construction
    ///@{

    /// Construct an empty path.
    /// @param _r The roadmap used by this path.
    Path(RoadmapType* const _r = nullptr);

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
    const std::vector<Cfg>& Cfgs() const;

    /// Get the current full Cfg path with steps spaced one environment
    ///        resolution apart. This is not cached due to its size and
    ///        infrequent usage.
    /// @param _lib The planning library to use.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes.
    const std::vector<Cfg> FullCfgs(MPLibrary* const _lib) const;

    /// Get the current full Cfg path with wait times. Steps are spaced one
    /// environment resolution apart. This is not cached due to its size and
    /// infrequent usage.
    /// @param _lib The planning library to use.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes and waiting.
    const std::vector<Cfg> FullCfgsWithWait(MPLibrary* const _lib) const;

    /// Get the number of timesteps calculated to traverse the path,
    /// including waiting times.
    size_t TimeSteps() const;

    /// Hardcode number of timesteps in the path.
    void SetTimeSteps(size_t _timesteps);

    /// Reset the number of timesteps to 0.
    void ResetTimeSteps();

    /// Append another path to the end of this one.
    /// @param _p The path to append.
    Path& operator+=(const Path& _p);

    /// Add another path to the end of this one and return the result.
    /// @param _p The path to add.
    Path operator+(const Path& _p) const;

    /// Append a new set of VIDs to the end of this path.
    /// @param _vids The VIDs to append.
    Path& operator+=(const std::vector<VID>& _vids);

    /// Add a new set of VIDs to the end of this path and return the result.
    /// @param _vids The VIDs to add.
    Path operator+(const std::vector<VID>& _vids) const;

    /// Copy assignment operator.
    Path& operator=(const Path& _p);

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

    /// Get the (source,target) of the path at the input timestep.
    /// @param _timestep The timestep to find the corresponding edge.
    /// @return  The source and target of the corresponding edge.
    std::pair<std::pair<size_t,size_t>,std::pair<size_t,size_t>> 
                        GetEdgeAtTimestep(size_t _timestep);

    ///@}

  private:

    ///@name Helpers
    ///@{

    void AssertSameMap(const Path& _p) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    RoadmapType* const m_roadmap;        ///< The roadmap.
    std::vector<VID> m_vids;             ///< The VIDs in the path.
    std::vector<size_t> m_waitingTimesteps; ///< The number of timesteps to wait at each vid.

    mutable std::vector<Cfg> m_cfgs; ///< The path configurations.
    mutable bool m_cfgsCached{false};    ///< Are the current cfgs correct?

    mutable double m_length{0};          ///< The path length.
    mutable bool m_lengthCached{false};  ///< Is the current length correct?

		size_t m_finalWaitTimeSteps{0}; ///< Temp - need to move this logic into the waiting timesteps.

    mutable size_t m_timesteps{0};
    mutable bool m_timestepsCached{false};

    ///@}
};

#endif
