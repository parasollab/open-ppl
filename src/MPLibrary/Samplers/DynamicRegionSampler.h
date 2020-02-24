#ifndef PMPL_DYNAMIC_REGION_SAMPLER_H_
#define PMPL_DYNAMIC_REGION_SAMPLER_H_

#include "SamplerMethod.h"

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
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Local Types
    ///@{

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


    void DirectSkeleton();

    ///@}
    ///@name Internal State
    ///@{

    WorkspaceSkeleton m_originalSkeleton; ///< The original workspace skeleton.
    WorkspaceSkeleton m_skeleton;         ///< The directed/pruned workspace skeleton.
    RegionKit m_regionKit;            ///< Manages regions following the skeleton.

    std::string m_skeletonType{"reeb"}; ///< Type of skeleton to build.
    std::string m_samplerLabel;       ///< The sampler label.
    std::string m_decompositionLabel; ///< The workspace decomposition label.
    std::string m_scuLabel;           ///< The skeleton clearance utility label.

    bool m_velocityBiasing{false};    ///< Use velocity biasing?
    double m_velocityAlignment{.1};   ///< Strength of velocity biasing.

    bool m_initialized{false};    ///< Have auxiliary structures been initialized?

    /// Last pair of points we used to direct the skeleton.
    std::pair<Point3d, Point3d> m_queryPair;

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

  m_skeletonType = _node.Read("skeletonType", true, "",
      "the type of skeleton to use, Available options are reeb and mcs "
      "for 3d, ma for 2d");

  // If using a reeb skeleton, we need a decomposition to build it.
  m_decompositionLabel = _node.Read("decompositionLabel",
      m_skeletonType == "reeb", "",
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
  throw RunTimeException(WHERE) << "Does not make sense for this object";
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
  if(m_initialized) {
    this->DirectSkeleton();
    return;
  }

  m_initialized = true;

  MethodTimer mt(this->GetStatClass(), "DynamicRegionSampler::BuildSkeleton");

  // Determine if we need a 2d or 3d skeleton.
  auto env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  const bool threeD = robot->GetMultiBody()->GetBaseType() ==
      Body::Type::Volumetric;

  if(threeD) {
    if(m_skeletonType == "mcs") {
      if(this->m_debug)
        std::cout << "Building a Mean Curvature skeleton." << std::endl;
      MeanCurvatureSkeleton3D mcs;
      //mcs.SetEnvironment(this->GetEnvironment());
      mcs.BuildSkeleton();

      // Create the workspace skeleton.
      auto sk = mcs.GetSkeleton();
      m_originalSkeleton = sk.first;
      m_originalSkeleton.DoubleEdges();
    }
    else if(m_skeletonType == "reeb") {
      // Create a workspace skeleton using a reeb graph.
      if(this->m_debug)
        std::cout << "Building a Reeb Graph skeleton." << std::endl;
      auto decomposition = this->GetMPTools()->GetDecomposition(
          m_decompositionLabel);
      ReebGraphConstruction reeb;
      reeb.Construct(decomposition);

      // Create the workspace skeleton.
      m_originalSkeleton = reeb.GetSkeleton();
      m_originalSkeleton.DoubleEdges();
    }
    else
      throw ParseException(WHERE) << "Unrecognized skeleton type '"
                                  << m_skeletonType << "', options for 3d "
                                  << "problems are {mcs, reeb}.";
  }
  else {
    // Collect the obstacles we want to consider (all in this case).
    std::vector<GMSPolyhedron> polyhedra;
    for(size_t i = 0; i < env->NumObstacles(); ++i) {
      MultiBody* const obstacle = env->GetObstacle(i);
      for(size_t j = 0; j < obstacle->GetNumBodies(); ++j)
        polyhedra.emplace_back(obstacle->GetBody(j)->GetWorldPolyhedron());
    }

    // Build a skeleton from a 2D medial axis.
    if(this->m_debug)
      std::cout << "Build a skeleton from a 2D medial axis." << endl;
    MedialAxis2D ma(polyhedra, env->GetBoundary());
    ma.BuildMedialAxis();
    m_originalSkeleton = get<0>(ma.GetSkeleton(1)); // 1 for free space.
  }

  if(this->m_debug)
    std::cout << "Direct skeleton" << endl;
  this->DirectSkeleton();
}


template <typename MPTraits>
void
DynamicRegionSampler<MPTraits>::
DirectSkeleton() {
  // Only support single-goal tasks; this is inherent to the method. The problem
  // is solvable but hasn't been solved yet.
  const auto& goalConstraints = this->GetTask()->GetGoalConstraints();
  if(goalConstraints.size() != 1)
    throw RunTimeException(WHERE) << "Only supports single-goal tasks. "
                                  << "Multi-step tasks will need new skeletons "
                                  << "for each sub-component.";

  // Find the workspace points which are nearest to the start and goal.
  auto g = this->GetRoadmap();
  auto goalTracker = this->GetGoalTracker();
  const auto& startVIDs = goalTracker->GetStartVIDs();
  const auto& goalVIDs  = goalTracker->GetGoalVIDs(0);
  Point3d start, goal;
  if(startVIDs.size() == 1) {
    const VID startVID = *startVIDs.begin();
    start = g->GetVertex(startVID).GetPoint();
  }
  else {
    // Probably we can just take the center of the start constraint boundary if
    // applicable, although we have no cases requiring that right now.
    throw RunTimeException(WHERE) << "Exactly one start VID is required, but "
                                  << startVIDs.size() << " were found.";
  }

  if(goalVIDs.size() == 1) {
    const VID goalVID = *goalVIDs.begin();
    goal = g->GetVertex(goalVID).GetPoint();
  }
  else {
    // Check for a goal boundary. We already checked that there is one goal
    // constraint, so it is safe to assume it exists here.
    const Boundary* const boundary = goalConstraints[0]->GetBoundary();
    if(!boundary)
      throw RunTimeException(WHERE) << "Exactly one goal VID is required, but "
                                    << goalVIDs.size() << " were found and no "
                                    << "constraint boundary was available.";

    // Try to sample a configuration in the boundary.
    auto sampler = this->GetSampler(m_samplerLabel);
    const size_t count    = 1,
                 attempts = 100;
    std::vector<CfgType> samples;
    sampler->Sample(count, attempts, boundary, std::back_inserter(samples));

    // If we couldn't generate a configuration here, the goal boundary isn't
    // realistic.
    if(samples.empty())
      throw RunTimeException(WHERE) << "Could not generate a sample within the "
                                    << "goal boundary " << *boundary
                                    << " after " << attempts << " attempts.";

    // We got a sample, take its point as the center point.
    goal = samples.front().GetPoint();
  }

  // If there is a new start and goal pair, redirect the skeleton
  std::pair<Point3d, Point3d> currentQuery{start, goal};
  if(currentQuery == m_queryPair)
    return;

  // Direct the workspace skeleton outward from the starting point.
  m_skeleton = m_originalSkeleton;
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

  m_regionKit.Clear();
  m_regionKit.Initialize(&m_skeleton, start, robotRadius, this->GetNameAndLabel(),
      this->GetRoadmap());

  m_queryPair = std::make_pair(start, goal);
}

/*----------------------------------------------------------------------------*/

#endif
