#ifndef PMPL_MEDIAL_AXIS_UTILITY_H_
#define PMPL_MEDIAL_AXIS_UTILITY_H_

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
//#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"
#include "MPLibrary/MPTools/ClearanceUtilities.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// Tool for pushing configurations to the medial axis.
////////////////////////////////////////////////////////////////////////////////
class MedialAxisUtility : public ClearanceUtility {

  public:

    ///@name Motion Planning Types
    ///@{

    

    ///@}
    ///@name Construction
    ///@{

    /// Construct a MedialAxisUtility object
    /// @param _vcLabel The validity checker to use.
    /// @param _dmLabel The distance metric to use.
    /// @param _exactClearance Use exact clearance or not.
    /// @param _exactPenetration Use exact penetration or not.
    /// @param _clearanceRays The number of clearance rays to use.
    /// @param _penetrationRays The number of penetration rays to use.
    /// @param _useBBX Use BBX or not.
    /// @param _positionalDofsOnly Use only positional DOFs.
    /// @param _debug Set to debug mode or not.
    /// @param _epsilon The epsilon value to use.
    /// @param _historyLength The history length to use.
    MedialAxisUtility(
        string _vcLabel = "", string _dmLabel = "",
        bool _exactClearance = false, bool _exactPenetration = false,
        size_t _clearanceRays = 10, size_t _penetrationRays = 10,
        bool _useBBX = true, bool _positionalDofsOnly = true, bool _debug = false,
        double _epsilon = 0.1, size_t _historyLength = 5);

    /// Construct a MedialAxisUtility object from an XML node
    /// @param _node The XML node to use.
    MedialAxisUtility(XMLNode& _node);

    virtual ~MedialAxisUtility() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    /// Print the internal state of this object
    /// @param _os The std::ostream to print to.
    virtual void Print(ostream& _os) const override;

    /// Set to print debug statements or not
    /// @param _debug Print debug statements or not.
    void SetDebug(const bool _debug) { this->m_debug = _debug; }

    ///@}
    ///@name Property Accessors
    ///@{

    /// Get the value of epsilon
    double GetEpsilon() const {return m_epsilon;}

    ///@}
    ///@name Medial Axis Functions
    ///@{

    /// Push a configuration to the medial axis.
    /// @param _cfg The configuration to push.
    /// @param _b The boundary to use.
    /// @return Whether the config was successfully pushed to the medial axis.
    bool PushToMedialAxis(Cfg& _cfg, const Boundary* const _b);

    /// Push a configuration that is known to be inside an obstacle
    /// towards the free-space medial axis. It will be pushed until it is
    /// outside the obstacle.
    /// @param _cfg The configuration to push.
    /// @param _b The boundary to use.
    /// @return Whether the config was successfully pushed out of the obstacle.
    bool PushFromInsideObstacle(Cfg& _cfg, const Boundary* const _b);


    /// Push a configuration to the medial axis by stepping away from the
    /// nearest obstacle at the resolution until a second witness (invalid cfg)
    /// is found and then the midpoint of those two cfgs will be the MA cfg.
    /// @param _cfg The configuration to push.
    /// @param _b The boundary to use.
    /// @return Whether the config was pushed to the medial axis
    bool PushCfgToMedialAxisMidpointRule(Cfg& _cfg, const Boundary* const _b);
    ///@}

  private:

    ///@name Internal State
    ///@{

    // m_epsilon is used only externally right now (not used in pushing to MA)
    double m_epsilon{.1};
    size_t m_historyLength{5};

    ///@}

    /// Test for equality of vectors based on whether they are within
    /// a given tolerance. This is used to assert that two witnesses
    /// found are different.
    /// @param _v1 The vector corresponding to the first configuration
    /// @param _v2 The vector corresponding to the second configuration
    /// @param _tolernace The tolerance for how close _v1 and _v2 can be
    /// @return Whether _v1 and _v2 are sufficiently different.
    bool FuzzyVectorEquality(mathtool::Vector3d _v1, mathtool::Vector3d _v2,
        const double _tolerance = 10.*std::numeric_limits<double>::epsilon());

    /// Test for equality of vectors based on whether the direction of
    /// the vectors are sufficiently different.
    /// @param _v1 The vector corresponding to the first configuration
    /// @param _v2 The vector corresponding to the second configuration
    /// @param _tolernace The tolerance for how close _v1 and _v2 can be
    /// @return Whether _v1 and _v2 are sufficiently different.
    bool WitnessObstacleEquality(Cfg _cfg, CDInfo _firstWitnessInfo, CDInfo _secondWitnessInfo);
};

#endif
