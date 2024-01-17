#ifndef PMPL_SAFE_INTERVAL_TOOL_H_
#define PMPL_SAFE_INTERVAL_TOOL_H_

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/XMLNode.h"

////////////////////////////////////////////////////////////////////////////////
/// Computes safe intervals for Cfgs and Edges. A 'safe interval' is an interval
/// of time where collision with a known dynamic obstacle (with known
/// trajectory) will not occur.
///
/// This tool implements the concept of a 'Safe Intervals' from the paper:
///
/// Phillips, Mike, and Maxim Likhachev. "Sipp: Safe interval path planning for
/// dynamic environments." Robotics and Automation (ICRA), 2011 IEEE International
/// Conference on. IEEE, 2011.
////////////////////////////////////////////////////////////////////////////////
class SafeIntervalTool final : public MPBaseObject {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType        RoadmapType;
    typedef typename MPBaseObject::WeightType         WeightType;
    typedef typename RoadmapType::VID                 VID;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::vector<Range<double>> Intervals; ///< A set of time intervals.

    ///@}
    ///@name Construction
    ///@{

    SafeIntervalTool();

    SafeIntervalTool(XMLNode& _node);

    virtual ~SafeIntervalTool();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Interval Computation
    ///@{

    /// Compute the safe intervals for a given Cfg.
    /// @param _cfg The configuration to compute safeIntervals's for.
    /// @return The set of safe intervals for _cfg.
    Intervals ComputeIntervals(const Cfg& _cfg);

    /// Compute the safe intervals for a given Edge, a source and a target.
    /// @param _weight The edge to compute safeIntervals's for.
    /// @param _source The source VID to compute safeIntervals's for.
    /// @param _target The target VID to compute safeIntervals's for.
    /// @param _roadmap The current roadmap.
    /// @return The set of safe intervals for _weight.
    Intervals ComputeIntervals(const WeightType& _weight, const VID _source,
      const VID _target, RoadmapType* _roadmap);

    ///@}
    ///@name Interval Checking
    ///@{

    /// Determine if a timestep is contained within a SafeInterval
    /// @param _intervals The safe intervals to check in.
    /// @param _timestep The timestep to check.
    bool ContainsTimestep(const Intervals& _intervals, const double _timestep);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Determine if a configuration is safe at the given time step.
    /// @param _cfg The configuration to check.
    /// @param _timestep The timestep for the dynamic obstacles.
    bool IsSafe(const Cfg& _cfg, const double _timestep);

    /// Computes the safe interval(s) for a set of configurations.
    /// @param _cfgs The cfgs to compute the safeIntervals for. These are
    ///              assumed to be a sequence which will be followed by the robot
    ///              at a rate of one cfg per time resolution.
    /// @return The set of time intervals for which it is safe to start
    ///         following the configuration sequence.
    Intervals ComputeSafeIntervals(const std::vector<Cfg>& _cfgs);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_vcLabel; ///< The validity checker to use.

    /// A cache of computed safe intervals for roadmap configurations.
    std::unordered_map<const Cfg*, Intervals> m_cfgIntervals;

    /// A cache of computed safe intervals for roadmap edges.
    std::unordered_map<const WeightType*, Intervals> m_edgeIntervals;

    ///@}
};

#endif
