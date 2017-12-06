#ifndef DYNAMIC_REGION_SAMPLER_H_
#define DYNAMIC_REGION_SAMPLER_H_

#include "SamplerMethod.h"

#include "MPLibrary/MapEvaluators/RRTQuery.h"
#include "MPLibrary/MPTools/ReebGraphConstruction.h"
#include "MPLibrary/MPTools/RegionKit.h"
#include "MPLibrary/MPTools/SkeletonClearanceUtility.h"
#include "Utilities/MedialAxis2D.h"
#include "Workspace/WorkspaceSkeleton.h"


////////////////////////////////////////////////////////////////////////////////
/// This sampler uses a region kit to produce samples along a workspace
/// skeleton. It is designed as a wrapper for another sampler, where the
/// responsibility of this object is to choose the appropriate sampling boundary
/// according to the region kit.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DynamicRegionSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;

    using typename SamplerMethod<MPTraits>::InputIterator;
    using typename SamplerMethod<MPTraits>::OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    DynamicRegionSampler();

    DynamicRegionSampler(XMLNode& _node);

    virtual ~DynamicRegionSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    /// We use a lazy init for this object. It will not create auxiliary
    /// structures until it is first called.
    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Sampler Interface
    ///@{

    /// This object chooses a sampling region and calls another sampler to
    /// generate configurations within.
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _result,
        OutputIterator _collision) override;

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    /// This object is a wrapper only and does not support the sampler rule.
    /// It cannot generate configurations directly.
    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Bias the velocities of a set of samples along a direction perscribed by
    /// the region kit.
    /// @param _cfgs The samples to bias.
    /// @param _region The dynamic region from which _cfgs were sampled.
    void BiasVelocities(std::vector<CfgType>& _cfgs,
        const Boundary* const _region) const;

    /// Initialize the object just in time by building a workspace skeleton and
    /// region kit.
    void LazyInitialize();

    ///@}
    ///@name Internal State
    ///@{

    WorkspaceSkeleton m_skeleton;     ///< The workspace skeleton.
    RegionKit m_regionKit;            ///< Manages regions following the skeleton.

    std::string m_samplerLabel;       ///< The sampler label.
    std::string m_decompositionLabel; ///< The workspace decomposition label.
    std::string m_scuLabel;           ///< The skeleton clearance utility label.

    bool m_velocityBiasing{false};    ///< Use velocity biasing?
    double m_velocityAlignment{.1};   ///< Strength of velocity biasing.

    bool m_initialized{false};    ///< Have auxiliary structures been initialized?

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
DynamicRegionSampler<MPTraits>::
DynamicRegionSampler() : SamplerMethod<MPTraits>() {
  this->SetName("DynamicRegionSampler");
}


template <typename MPTraits>
DynamicRegionSampler<MPTraits>::
DynamicRegionSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("DynamicRegionSampler");

  m_regionKit = RegionKit(_node);

  m_samplerLabel = _node.Read("samplerLabel", true, "", "The sampler to use "
      "within regions.");

  m_decompositionLabel = _node.Read("decompositionLabel", true, "",
      "The workspace decomposition to use.");

  m_scuLabel = _node.Read("scuLabel", false, "", "The skeleton clearance utility "
      "to use. If not specified, we use the hack-fix from wafr16.");

  m_velocityBiasing = _node.Read("velocityBiasing", false, m_velocityBiasing,
      "Bias nonholonomic samples along the skeleton?");

  m_velocityAlignment = _node.Read("velocityAlignment", false,
      m_velocityAlignment, -1., .99,
      "Minimum dot product for sampled velocity and biasing direction.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
DynamicRegionSampler<MPTraits>::
Print(std::ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  m_regionKit.Print(_os);
  _os << "\tSampler Label: " << m_samplerLabel
      << "\n\tVelocity biasing: " << (m_velocityBiasing ? "enabled" : "diabled")
      << "\n\tVelocity Alignment: " << m_velocityAlignment
      << std::endl;
}


template <typename MPTraits>
void
DynamicRegionSampler<MPTraits>::
Initialize() {
  // Disable velocity biasing if the robot is holonomic.
  m_velocityBiasing &= this->GetTask()->GetRobot()->IsNonholonomic();

  // We use lazy initialization, so the object is not ready.
  m_initialized = false;
  m_regionKit.Clear();
}

/*---------------------------- Sampler Interface -----------------------------*/

template <typename MPTraits>
void
DynamicRegionSampler<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _boundary,
    OutputIterator _result, OutputIterator _collision) {
  LazyInitialize();

  // Choose boundary with region kit. The passed boundary will be the fallback
  // in this case.
  const Boundary* region = m_regionKit.SelectRegion();

  // If we received a null boundary, use the passsed boundary as the whole
  // environment.
  const bool usingDynamicRegion = region;
  if(!usingDynamicRegion)
    region = _boundary;

  // Use sampler with this boundary.
  std::vector<CfgType> result, collision;
  auto sampler = this->GetSampler(m_samplerLabel);
  sampler->Sample(_numNodes, _maxAttempts, region, std::back_inserter(result),
      std::back_inserter(collision));

  // If we are using velocity biasing, apply that here.
  if(usingDynamicRegion and m_velocityBiasing)
    BiasVelocities(result, region);

  // Increment success and attempts if we sampled from a dynamic region.
  if(usingDynamicRegion) {
    m_regionKit.IncrementSuccess(region, result.size());
    m_regionKit.IncrementFailure(region, collision.size());
  }

  _result = copy(result.begin(), result.end(), _result);
  _collision = copy(collision.begin(), collision.end(), _collision);
}

/*------------------------------- Sampler Rule -------------------------------*/

template <typename MPTraits>
bool
DynamicRegionSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {
  throw RunTimeException(WHERE, "Does not make sense for this object");
  return false;
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
void
DynamicRegionSampler<MPTraits>::
BiasVelocities(std::vector<CfgType>& _cfgs, const Boundary* const _region) const {
  MethodTimer mt(this->GetStatClass(), "DynamicRegionSampler::VelocityBiasing");

  // Get the bias from the region kit.
  const Vector3d bias = m_regionKit.GetVelocityBias(_region);
  if(bias.norm() == 0)
    throw RunTimeException(WHERE, "Bias cannot be zero.");

  // Resample each Cfg until its linear velocity aims relatively along the
  // biasing direction.
  for(auto& cfg : _cfgs) {
    Vector3d velocity;
    do {
      cfg.GetRandomVelocity();
      velocity = cfg.GetLinearVelocity().normalize();
      if(this->m_debug)
        std::cout << "\tSampled velocity direction: " << velocity
                  << "\n\t\tDot product with bias: " << velocity * bias
                  << (velocity * bias < m_velocityAlignment ? " < " : " >= ")
                  << m_velocityAlignment
                  << std::endl;
    } while(velocity * bias < m_velocityAlignment);
  }
}


template <typename MPTraits>
void
DynamicRegionSampler<MPTraits>::
LazyInitialize() {
  if(m_initialized)
    return;
  m_initialized = true;

  MethodTimer mt(this->GetStatClass(), "DynamicRegionSampler::BuildSkeleton");

  // Determine if we need a 2d or 3d skeleton.
  auto env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  const bool threeD = robot->GetMultiBody()->GetBaseType() ==
      Body::Type::Volumetric;

  if(threeD) {
    // Create a workspace skeleton using a reeb graph.
    auto decomposition = this->GetMPTools()->GetDecomposition(m_decompositionLabel);
    ReebGraphConstruction reeb;
    reeb.Construct(decomposition, this->GetBaseFilename());

    // Create the workspace skeleton.
    m_skeleton = reeb.GetSkeleton();
  }
  else {
    // Collect the obstacles we want to consider (all in this case).
    std::vector<GMSPolyhedron> polyhedra;
    for(size_t i = 0; i < env->NumObstacles(); ++i) {
      auto obstacle = env->GetObstacle(i);
      polyhedra.emplace_back(obstacle->GetBody(0)->GetWorldPolyhedron());
    }

    // Build a skeleton from a 2D medial axis.
    MedialAxis2D ma(polyhedra, env->GetBoundary());
    ma.BuildMedialAxis();
    m_skeleton = get<0>(ma.GetSkeleton(1)); // 1 for free space.
  }

  // Get the start and goal point from the query.
  /// @TODO Support this for RRT and PRM.
  auto query = static_cast<RRTQuery<MPTraits>*>(this->GetMapEvaluator("RRTQuery").
      get());
  const Point3d start = query->GetQuery()[0].GetPoint(),
                goal  = query->GetQuery()[1].GetPoint();

  // Direct the workspace skeleton outward from the starting point.
  m_skeleton = m_skeleton.Direct(start);

  // Prune the workspace skeleton relative to the goal.
  m_skeleton.Prune(goal);

  // Fix the skelton clearance for 3D.
#if 0 // Broken. Pushing the skeleton should not be done blindly.
  if(threeD) {
    if(!m_scuLabel.empty()) {
      auto util = this->GetMPTools()->GetSkeletonClearanceUtility(m_scuLabel);
      (*util)(m_skeleton);
    }
    else {
      SkeletonClearanceUtility<MPTraits> util;
      util.SetMPLibrary(this->GetMPLibrary());
      util.HackFix(m_skeleton);
    }
  }
#endif

  // Initialize the region kit.
  const double robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->
      GetBoundingSphereRadius();

  m_regionKit.Initialize(&m_skeleton, start, robotRadius, this->GetNameAndLabel(),
      this->GetRoadmap()->GetGraph());

//#ifdef VIZMO
//  GetVizmo().GetEnv()->AddWorkspaceDecompositionModel(this->GetEnvironment()->
//      GetDecomposition());
//  GetMainWindow()->GetModelSelectionWidget()->CallResetLists();
//#endif
}

/*----------------------------------------------------------------------------*/

#endif
