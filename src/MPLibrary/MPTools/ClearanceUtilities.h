#ifndef PMPL_CLEARANCE_UTILITY_H_
#define PMPL_CLEARANCE_UTILITY_H_

#include "MPLibrary/MPBaseObject.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
//#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"


class Robot;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// Return structure for clearance information.
////////////////////////////////////////////////////////////////////////////////
struct ClearanceStats {
  double m_avg{0};
  double m_min{0};
  double m_max{0};
  double m_var{0};
  std::vector<double> m_clearanceAlongPath;
  double m_pathLength{0};
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// A ray in configuration space for approximate methods.
////////////////////////////////////////////////////////////////////////////////
template<class CfgType>
struct Ray {
  CfgType m_incr;
  CfgType m_tick;

  Ray(const CfgType& _i, const CfgType& _t) : m_incr(_i), m_tick(_t) {}
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// Computes clearance and penetration of configurations with respect to
/// obstacles.
////////////////////////////////////////////////////////////////////////////////
class ClearanceUtility : virtual public MPBaseObject {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::WeightType     WeightType;
    typedef typename MPBaseObject::RoadmapType    RoadmapType;
    typedef typename RoadmapType::VID         VID;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a ClearanceUtility object
    /// @param _vcLabel The validity checker to use.
    /// @param _dmLabel The distance metric to use.
    /// @param _exactClearance Use exact clearance or not.
    /// @param _exactPenetration Use exact penetration or not.
    /// @param _clearanceRays The number of clearance rays to use.
    /// @param _penetrationRays The number of penetration rays to use.
    /// @param _approxStepSize The approximate step size to use.
    /// @param _approxResolution The approximate resolution to use.
    /// @param _useBBX Use BBX or not.
    /// @param _positionalDofsOnly Use only positional DOFs.
    /// @param _debug Set to debug mode or not.
    ClearanceUtility(
        string _vcLabel = "", string _dmLabel = "",
        bool _exactClearance = false, bool _exactPenetration = false,
        size_t _clearanceRays = 10, size_t _penetrationRays = 10,
        double _approxStepSize = MAX_DBL, double _approxResolution = MAX_DBL,
        bool _useBBX = true, bool _positionalDofsOnly = true, bool _debug = false);

    /// Construct a ClearanceUtility object from an XML node
    /// @param _node The XML node to use
    ClearanceUtility(XMLNode& _node);

    virtual ~ClearanceUtility() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    /// Print the internal state of this object
    /// @param _os The std::ostream to print to.
    virtual void Print(ostream& _os) const override;

    virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the label of the distance metric
    /// @return The label of the distance metric.
    const string& GetDistanceMetricLabel() const {return m_dmLabel;}

    /// Get the label of the validity checker
    /// @return The label of the validity checker.
    const string& GetValidityCheckerLabel() const {return m_vcLabel;}

    /// Set the label of the validity checker
    /// @param _s The label of the validity checker to use.
    void SetValidityCheckerLabel(const string& _s) {m_vcLabel = _s;}

    /// Get whether exact clearance is used or not
    /// @return Whether exact clearance is being used or not as a bool.
    bool GetExactClearance() const {return m_exactClearance;}

    /// Get the position resolution
    /// @return The position resolution.
    double GetPositionResolution() const {return m_rayTickResolution;}

    /// Get the orientation resolution
    /// @return The orientation resolution.
    double GetOrientationResolution() const {return m_orientationResolution;}

    /// Get whether the object is initialized or not
    /// @return Whether the ClearanceUtility object has been initailized.
    bool IsInitialized() const {return m_initialized;}

    ///@}
    ///@name Clearance Functions
    ///@{

    /// Calculate clearance information as well as the closest witness
    /// approximately or exactly.
    /// @param _cfg The configuration to push
    /// @param _clrCfg Place holder for the clearance configuration
    /// @param _b The boundary
    /// @param _cdInfo Collision detector information reference
    /// @param _useOppValidityWitness Whether the witness should be of
    /// opposite validity to _cfg.
    /// @return Whether a witness was found, if found information will be
    /// stored in _clrCfg and _cdInfo.
    bool CollisionInfo(Cfg& _cfg, Cfg& _clrCfg,
        const Boundary* const _b, CDInfo& _cdInfo,
        const bool& _useOppValidityWitness = true);

    /// Calculate clearance information exactly by taking the validity
    /// checker results against obstacles to the bounding box.
    /// @param _cfg The configuration to push
    /// @param _clrCfg Place holder for the clearance configuration
    /// @param _b The boundary
    /// @param _cdInfo Collision detector information reference
    /// @param _useOppValidityWitness Whether the witness should be of
    /// opposite validity to _cfg.
    /// @return Whether a witness was found, if found information will be
    /// stored in _clrCfg and _cdInfo.
    bool ExactCollisionInfo(Cfg& _cfg, Cfg& _clrCfg,
        const Boundary* const _b, CDInfo& _cdInfo,
        const bool _useOppValidityWitness = true);

    /// Calculate the approximate clearance using a series of rays. The
    /// specified number of rays are pushed outward until they change in
    /// validity. The shortest ray is stored as the witness.
    /// @param _cfg The configuration to push
    /// @param _clrCfg Place holder for the clearance configuration
    /// @param _b The boundary
    /// @param _cdInfo Collision detector information reference
    /// @param _useOppValidityWitness Whether the witness should be of
    /// opposite validity to _cfg.
    /// @return Whether a witness was found, if found information will be
    /// stored in _clrCfg and _cdInfo.
    bool ApproxCollisionInfo(Cfg& _cfg, Cfg& _clrCfg,
        const Boundary* const _b, CDInfo& _cdInfo,
        const bool& _useOppValidityWitness = true);

    /// Helpers for ApproxCollisionInfo (should maybe be protected):
    /// Find the witness of an initial sample of any validity. A witness here is
    /// defined as the nearest point of obstacle boundary.
    ///@TODO Make the user have the option to have same or opposite validity
    /// for witnesses.
    /// @param _numRays The number of rays to search for the nearest witness
    /// @param _rays The container for the rays, if empty will be populated
    /// @param _sampledCfg The sampled configuration to eventually push
    /// @param _initValidity The initial validity of the _sampledCfg
    /// @param _b The boundary to use
    /// @param _candidate Container for candidate witness :
    /// _candidate.first is the index of the candidate ray within _rays
    /// and _candidate.second is the candidate configuration.
    /// @param _useOppValidityWitness Whether the witness should be of
    /// opposite validity to _sampledCfg
    /// @return Whether a witness was found, if successful t he witness
    /// can be found in _candidate.second
    bool FindApproximateWitness(const std::size_t& _numRays,
        std::vector<Ray<Cfg> >& _rays, const Cfg& _sampledCfg,
        const bool& _initValidity, const Boundary* const _b,
        std::pair<size_t, Cfg>& _candidate,
        const bool& _useOppValidityWitness = true);

    /// Generates the rays for the witness finding. This is a helper
    /// function of FindApproximateWitness which will only be called
    /// if the _rays parameter passed into FindapproximateWitness is
    /// empty.
    /// @param _sampledCfg The sampled configuration we wish to push
    /// @param _numRays The number of rays to search for the nearest witness
    /// @param _rays The container for the rays generated.
    /// @return The rays for witness exploration are stored in _rays.
    void MakeRays(const Cfg& _sampledCfg, const std::size_t& _numRays,
        std::vector<Ray<Cfg> >& _rays);

    /// A final sanity check to be performed on the witness candidate.
    /// Compares the candidate cfg with the "previous" ticked configuration.
    /// The appropriate direction of the tick is dependent on whether
    /// opposite validity is true or not.
    /// @param _cand The information for the candidate witness, has two parts.
    /// _cand.first is the index of the ray (within _rays) which lead to the
    /// candidate witness.
    /// _cand.second is the actual configuration of the candidate witness.
    /// @param _rays The vector which stores all rays that were explored.
    /// @param _b The boundary being used.
    /// @param useOppValidityWitness Whether we are asserting that the witness
    /// should be of opposite validity to the sampled configuration.
    bool ValidateCandidate(const std::pair<size_t, Cfg>& _cand,
        const std::vector<Ray<Cfg> >& _rays,
        const Boundary* const _b, const bool& _useOppValidityWitness = true);

    /// Calculate roadmap clearance statistics including averages, mins,
    ///        and maxes for clearance across roadmaps, paths, and edges.
    ClearanceStats RoadmapClearance();

    /// Calculate path clearance statistics including averages, mins,
    ///        and maxes for clearance across the path
    /// @param _path The path to use.
    ClearanceStats PathClearance(vector<VID>& _path);

    /// Calculate path clearance statistics including averages, mins,
    ///        and maxes for clearance across the path
    /// @param _path The path to use.
    ClearanceStats PathClearance(vector<Cfg>& _path);

    /// Calculate path clearance statistics including averages, mins,
    ///        and maxes for clearance across the edge
    /// @param _c1 The source vertex.
    /// @param _c2 The target vertex.
    /// @param _weight The weight of the edge.
    vector<double> EdgeClearance(const Cfg& _c1, const Cfg& _c2,
        const WeightType& _weight);

    /// This function places the nearest vertex of the environment and the
    /// corresponding robot vertex into _cdInfo. This function is only used
    /// by the exact version of collision info.
    /// @param _cfg The sampled configuration to push
    /// @param _cdInfo The container for collision detection information.
    /// The nearest configuration of the environment (witness candidate)
    /// and the point on the robot which corresponds to this config will
    /// be stored in _cdInfo if successful.
    /// @param _b The boundary to use
    /// @return The validity of _cfg provided.
    const bool GetNearestVertexWitness(Cfg& _cfg, CDInfo& _cdInfo,
                                       const Boundary* const _b);

    void SetRobot(Robot* _robot);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    string m_vcLabel{"cd4"};        ///< Validity checker method label.
    string m_dmLabel{"euclidean"};  ///< Distance metric method label.

    bool m_useBBX{true};            ///< Use bounding box as obstacle?

    bool m_positionalDofsOnly{true};        ///< Use only positional dofs?

    bool m_exactClearance{false};   ///< Use exact clearance calculations?
    bool m_exactPenetration{false}; ///< Use exact penetration calculations?

    size_t m_clearanceRays{10};  ///< Number of rays for approximate clearance.
    size_t m_penetrationRays{10};///< Number of rays for approximate penetration.

    // These replaced two similar members from the ClearanceUtility
    // class (m_approxStepSize and m_approxResolution):
    /// The resolution for rayTicks
    double m_rayTickResolution;//no default values as these are required in node
    /// The orientation resolution
    double m_orientationResolution;

    //These two are linearly related based on the resolution for ray ticking.
    // The max ray magnitude is just how far out a ray will be attempted.
    /// How far out a ray will be attempted
    double m_maxRayMagnitude;
    /// The maximum number of ray iterations
    size_t m_maxRayIterations;

    //These are just for getting rayTickResolution and orientationResolution.
    // They get multiplied into their respective values from the environment,
    // and should only really be used there. See Initialize() for more info.
    double m_orientationResFactor{1.}; ///< The orientation resolution factor
    double m_positionalResFactor{1.}; ///< The position resoluiton factor
    double m_maSearchResolutionFactor{1}; ///< The search resolution factor

    /// A boolean to determine whether initialize has been called on the
    /// utility or not yet. This is important, as there are MPLibrary things
    /// we need that are not available at construction time:
    bool m_initialized{false};

    Robot* m_robot;

    ///@}

    /// Adjusts a witness from a nearby witness point to a point of guaranteed
    /// validity, based on _initValidity and _useOppValidityWitness. this is
    /// a helper function called by ExactCollisionInfo.
    /// @param _cfg The sampled configuration to we wish to push
    /// @param _initValidity The initial validity of _cfg
    /// @param _witnessCfg The candidiate witness configuration
    /// @param _b The boundary to use
    /// @param _useOppValidityWitness Whether _cfg and _witness should be the
    /// the same validity or not
    /// @return Whether the witness was successfully adjusted to satisfy the
    /// _useOppValidityWitness condition. If succesfully, the _witnessCfg is
    /// updated.
    bool AdjustWitnessToEnsureValidity(
                                const Cfg& _cfg, const bool _initValidity,
                                Cfg& _witnessCfg, const Boundary* const _b,
                                const bool _useOppValidityWitness = true);

    int abcd = 0;
};

#endif
