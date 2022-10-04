#ifndef PPL_DYNAMIC_REGION_RRT_H_
#define PPL_DYNAMIC_REGION_RRT_H_

#include "BasicRRTStrategy.h"

#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "MPLibrary/MPTools/MeanCurvatureSkeleton3D.h"
#include "MPLibrary/MPTools/ReebGraphConstruction.h"
#include "Utilities/MedialAxis2D.h"
#include "Utilities/XMLNode.h"
#include "Utilities/MPUtils.h"
#include "Workspace/WorkspaceSkeleton.h"

////////////////////////////////////////////////////////////////////////////////
/// Dynamic Region-biased RRT algorithm.
///
/// An RRT guided by a workspace skeleton.
///
/// Reference:
///   Jory Denny, Read Sandstrom, Andrew Bregger, and Nancy M. Amato.
///   "Dynamic Region-biased Rapidly-exploring Random Trees." WAFR 2016.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////

template <typename MPTraits>
class DynamicRegionRRT : virtual public BasicRRTStrategy<MPTraits> {
  public:
    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name WorkspaceSkeleton Types
    ///@{

    using SkeletonEdgeDescriptor   = WorkspaceSkeleton::ED;
    using SkeletonEdgeIterator     = WorkspaceSkeleton::adj_edge_iterator;
    using SkeletonVertexDescriptor = WorkspaceSkeleton::vertex_descriptor;
    using SkeletonVertexIterator   = WorkspaceSkeleton::vertex_iterator;

    ///@}
    ///@name Local Types
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// Representation of a sampling region.
    ////////////////////////////////////////////////////////////////////////////
    struct SamplingRegion {

      ///@name Internal State
      ///@{

      SkeletonEdgeIterator edgeIterator; ///< Iterator to region's edge.
      size_t edgeIndex{0};   ///< Which edge point are we at?
      double attempts{1};    ///< Number of attempts to extend into this region.
      double successes{1};   ///< Number of successful attempts.

      ///@}

      SamplingRegion(const SkeletonEdgeIterator& _eit)
          : edgeIterator(_eit) {}

      /// Track the success rate of extending into this region.
      void TrackSuccess(const size_t _success, const size_t _attempts) {
        successes *= .9;
        attempts  *= .9;
        successes += _success;
        attempts  += _attempts;
      }

      /// Compute the weight for this region (i.e. success rate).
      double GetWeight() const noexcept {
        return successes / attempts;
      }

      /// Get the center of this region.
      const Vector3d& GetCenter() const noexcept {
        return (*edgeIterator).property()[edgeIndex];
      }

      /// Check if this region is at the last point on its skeleton edge.
      bool LastPoint() const noexcept {
        return edgeIndex == (*edgeIterator).property().size() - 1;
      }

      /// Advance this region to the next skeleton edge point.
      void Advance() noexcept {
        ++edgeIndex;
      }

      /// Assignment operator
      SamplingRegion& operator=(const SamplingRegion& _region) {
        if(this != &_region) {
          edgeIterator = _region.edgeIterator;
          edgeIndex = _region.edgeIndex;
          attempts = _region.attempts;
          successes = _region.successes;
        }
        return *this;
      }

      /// Equality operator
      bool operator==(const SamplingRegion& _region) const {
        bool eit = edgeIterator == _region.edgeIterator;
        bool idx = edgeIndex == _region.edgeIndex;
        bool att = attempts == _region.attempts;
        bool succ = successes == _region.successes;
        return eit and idx and att and succ;
      }

    };

    ///@}
    ///@ Construction
    ///@{

    DynamicRegionRRT();

    DynamicRegionRRT(XMLNode& _node);

    virtual ~DynamicRegionRRT() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name BasicRRTStrategy Overrides
    ///@{

    /// Get a random configuration to grow towards.
    virtual CfgType SelectTarget() override;

    /// Add a new configuration to the roadmap and current tree.
    /// @param _newCfg The new configuration to add.
    /// @return A pair with the added VID and a bool indicating whether the new
    ///         node was already in the map.
    virtual std::pair<VID, bool> AddNode(const CfgType& _newCfg) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Sample a configuration from within a sampling region using the sampler
    /// given in m_samplerLabel.
    /// @param _region The region to sample from.
    /// @return A configuration with the sampling region.
    CfgType Sample(SamplingRegion* _region);

    /// Sample a configuration from within a given boundary using the sampler
    /// given in _samplerLabel.
    /// @param _region The region to sample from.
    /// @return A configuration with the boundary.
    CfgType Sample(const Boundary* const _boundary, const std::string* _samplerLabel);

    /// Calculate the velocity bias along a region's skeleton edge.
    /// @param _region The region whose skeleton edge to bias the velocity along.
    /// @return The velocity bias.
    const Vector3d GetVelocityBias(SamplingRegion* _region);

    /// Determine if a region is touching a configuration.
    /// @param _cfg The configuration.
    /// @param _region The sampling region.
    bool IsTouching(const Cfg& _cfg, SamplingRegion& _region);

    /// Calculate the boundary around a sampling region.
    /// @param _v The center of the sampling region.
    /// @return The boundary with center _v and radius m_regionRadius.
    CSpaceBoundingSphere MakeBoundary(const Vector3d& _v);
    
    ///@}
    ///@name Skeleton and Workspace
    ///@{

    /// Build topological skeleton
    void BuildSkeleton();

    /// Construct the pruned and directed query skeleton.
    void DirectSkeleton();

    /// Select a region based on weighted success probabilities
    /// @return The sampling region to be expanded
    const size_t SelectSamplingRegion();

    /// Compute probabilities for selecting each sampling region.
    /// @return Probabilities based on extension success
    std::vector<double> ComputeProbabilities();

    /// Bias the velocity of a sample along a direction perscribed by
    /// the region.
    /// @param _cfg The sample to bias.
    /// @param _region The region from which _cfg was sampled.
    void BiasVelocity(CfgType& _cfg, SamplingRegion* _region);

    /// Check if q_new is close enough to an unvisited skeleton vertex to create
    /// new regions on the outgoing edges of that vertex. If so, create those
    /// new regions.
    /// @param _p The new configuration added to the roadmap.
    void CheckRegionProximity(const Point3d& _p);

    /// Create new regions on the outgoing edges of the skeleton vertex.
    /// @param _iter The skeleton vertex iterator.
    /// @return The newly created sampling regions.
    std::vector<SamplingRegion*> 
    CreateRegions(const WorkspaceSkeleton::vertex_iterator _iter);

    /// Advance all sampling regions until they are no longer touching the
    /// newly added configuration.
    /// @param _cfg The newly added configuration, q_new.
    void AdvanceRegions(const Cfg& _cfg);

    /// Advance a region until it is either not longer touching a configuration
    /// or until it reaches the end of its respective skeleton edge.
    /// @param _cfg A configuration possibly touching the region.
    /// @param _region The region to advance along its skeleton edge.
    bool AdvanceRegionToCompletion(const Cfg& _cfg, SamplingRegion& _region);

    ///@}
    ///@name Internal State
    ///@{

    WorkspaceSkeleton m_originalSkeleton; ///< The original workspace skeleton.
    WorkspaceSkeleton m_skeleton;         ///< The directed/pruned workspace skeleton.

    std::string m_skeletonType{"reeb"}; ///< Type of skeleton to build.
    std::string m_decompositionLabel; ///< The workspace decomposition label.
    std::string m_scuLabel;           ///< The skeleton clearance utility label.

    bool m_velocityBiasing{false};    ///< Use velocity biasing?
    double m_velocityAlignment{.1};   ///< Strength of velocity biasing.

    bool m_initialized{false};    ///< Have auxiliary structures been initialized?

    /// Pair of points we use to direct the skeleton.
    std::pair<Point3d, Point3d> m_queryPair;

    /// The set of active dynamic sampling regions and associated metadata.
    std::vector<SamplingRegion> m_regions;

    /// Keep track of which skeleton vertices we've visited.
    std::unordered_map<WorkspaceSkeleton::VD, bool> m_visited;

    /// The dynamic sampling regions will have radius equal to this times the
    /// robot's bounding sphere radius.
    double m_regionFactor{2};

    double m_regionRadius{0}; ///< The region radius.

    /// Weight of explore vs. exploit in region selection probabilities.
    /// Exploring is a uniform chance to select each region, while exploit
    /// favors successful regions.
    double m_explore{.5};

    /// A configuration is considered to be touching a region when this fraction
    /// of its bounding sphere penetrates into the region.
    double m_penetrationFactor{1};

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT() : BasicRRTStrategy<MPTraits>() {
    this->SetName("DynamicRegionRRT");
}

template <typename MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(XMLNode& _node) : BasicRRTStrategy<MPTraits>(_node) {
  this->SetName("DynamicRegionRRT");

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

  m_explore = _node.Read("explore", true, m_explore, 0., 1.,
      "Weight of explore vs. exploit in region selection probabilities");

  m_regionFactor = _node.Read("regionFactor", true,
      m_regionFactor, 1., std::numeric_limits<double>::max(),
      "Regions are this * robot's bounding sphere radius");

  m_penetrationFactor = _node.Read("penetration", true,
      m_penetrationFactor, std::numeric_limits<double>::min(), 1.,
      "Fraction of bounding sphere penetration that is considered touching");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::Print(std::ostream& _os) const {
  BasicRRTStrategy<MPTraits>::Print(_os);

  _os << "\tSkeleton Type:" << m_skeletonType << std::endl;

  if(!m_decompositionLabel.empty())
    _os << "\tWorkspace Decomposition Label:" << m_decompositionLabel << std::endl;

  if(!m_scuLabel.empty())
    _os << "\tSkeleton Clearance Utility:" << m_scuLabel << std::endl;

  _os << "\tVelocity Biasing: " << m_velocityBiasing << std::endl;
  _os << "\tVelocity Alignment: " << m_velocityAlignment << std::endl;

  _os << "\tRegion Factor: " << m_regionFactor << std::endl;
  _os << "\tRegion Radius: " << m_regionRadius << std::endl;
  _os << "\tPenetration Factor: " << m_penetrationFactor << std::endl;
  _os << "\tExploration Factor: " << m_explore << std::endl;
}

/*---------------------------- MPStrategy Overrides --------------------------*/

template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::Initialize(){
  BasicRRTStrategy<MPTraits>::Initialize();

  // Check that only one direction is being extended.
  if (this->m_numDirections > 1)
    throw RunTimeException(WHERE) << "Extending more than one direction "
                                  << "is not supported.";

  // Disable velocity biasing if the robot is holonomic.
  m_velocityBiasing &= this->GetTask()->GetRobot()->IsNonholonomic();

  const double robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->
      GetBoundingSphereRadius();
  m_regionRadius = m_regionFactor * robotRadius;

  // Initialize the skeleton, local components, and regions.
  if(!m_initialized) {
    m_initialized = true;
    BuildSkeleton();

    MethodTimer mt(this->GetStatClass(),
        this->GetNameAndLabel() + "::InitializeRoadmap");
    if(this->m_debug)
      std::cout << "Initializing region at start skeleton vertex."
                << std::endl;

    // Mark all nodes unvisited.
    m_visited.clear();
    for(auto vit = m_skeleton.begin(); vit != m_skeleton.end(); ++vit)
      m_visited[vit->descriptor()] = false;

    // Find the vertex nearest to start and create regions for each outgoing
    // edge.
    auto iter = m_skeleton.FindNearestVertex(m_queryPair.first);
    CreateRegions(iter);
  }
}

/*------------------------ BasicRRTStrategy Overrides ------------------------*/

template <typename MPTraits>
typename MPTraits::CfgType
DynamicRegionRRT<MPTraits>::
SelectTarget() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SelectTarget");

  // Get the sampler.
  const std::string* samplerLabel = &this->m_samplerLabel;

  // Select goal growth with probability m_growthFocus.
  auto goalTracker = this->GetGoalTracker();
  const std::vector<size_t> unreachedGoals = goalTracker->UnreachedGoalIndexes();

  if(unreachedGoals.size() and DRand() < this->m_growthFocus) {
    // Randomly select a goal constraint boundary.
    const auto& goalConstraints = this->GetTask()->GetGoalConstraints();
    const size_t index = unreachedGoals[LRand() % unreachedGoals.size()];
    const Boundary* const b = goalConstraints[index]->GetBoundary();

    // We may eventually support constraints that cannot be described in terms
    // of a boundary, but that is outside the scope of the present
    // implementation.
    if (!b)
      throw NotImplementedException(WHERE) << "Non-boundary constraints are not "
                                           << "yet supported.";
      
    // If there is a query sampler, use that for goal sampling.
    if (!this->m_querySampler.empty())
      samplerLabel = &this->m_querySampler;

    if (this->m_debug)
      std::cout << "Sampling growth target from goal " << index
                << " (sampler '" << *samplerLabel << "'):"
                << std::endl;

    return Sample(b, samplerLabel);
  }

  // Otherwise, use the designated sampler with the region (or environment) 
  // boundary.
  // Select a region for sample generation.
  const size_t regionIdx = SelectSamplingRegion();

  // If we received a null boundary, use the full environment.
  if (regionIdx < m_regions.size())
    return Sample(&m_regions[regionIdx]);
  else
    return Sample(this->GetEnvironment()->GetBoundary(), samplerLabel);
}


template <typename MPTraits>
std::pair<typename DynamicRegionRRT<MPTraits>::VID, bool>
DynamicRegionRRT<MPTraits>::
AddNode(const CfgType& _newCfg) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::AddNode");

  auto g = this->GetRoadmap();

  const VID lastVID = g->GetLastVID();
  const VID newVID  = g->AddVertex(_newCfg);

  const bool nodeIsNew = lastVID != g->GetLastVID();
  if(nodeIsNew) {
    if(this->m_debug)
      std::cout << "\tAdding VID " << newVID << "."
                << std::endl;
    

    // On each new sample, check if we need to advance our regions and generate
    // new ones. Add a roadmap hook to achieve this.
    auto vi = g->find_vertex(newVID);
    CheckRegionProximity(vi->property().GetPoint());
    AdvanceRegions(vi->property());
  }

  return {newVID, nodeIsNew};
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
typename MPTraits::CfgType
DynamicRegionRRT<MPTraits>::
Sample(SamplingRegion* _region) {
  if (this->m_debug)
    std::cout << "\tSampling from region with center at "
              << _region->GetCenter() << ", success rate so far "
              << _region->successes << " / " << _region->attempts
              << ", using sampler '" << this->m_samplerLabel << "'." << std::endl;

  // Get the boundary of the region.
  const auto center = _region->GetCenter();
  auto samplingBoundary = MakeBoundary(center);

  // Get the sampler.
  auto s = this->GetSampler(this->m_samplerLabel);

  std::vector<CfgType> samples, collision;
  while(samples.empty()) {
    s->Sample(1, 100, &samplingBoundary, std::back_inserter(samples),
      std::back_inserter(collision));
    
    // Increment successes as we sample.
    _region->TrackSuccess(samples.size(), samples.size() + collision.size());
  }
  auto target = samples.front();

  if(m_velocityBiasing)
    BiasVelocity(target, _region);

  if(this->m_debug)
    std::cout << "\t" << target.PrettyPrint() << std::endl;
  
  return target;
}


template <typename MPTraits>
typename MPTraits::CfgType
DynamicRegionRRT<MPTraits>::
Sample(const Boundary* const _boundary, const std::string* _samplerLabel) {
  // Get the sampler.
  auto s = this->GetSampler(*_samplerLabel);

  std::vector<CfgType> samples, collision;
  while(samples.empty())
    s->Sample(1, 100, _boundary, std::back_inserter(samples),
      std::back_inserter(collision));

  auto target = samples.front();
  if(this->m_debug)
    std::cout << "\t" << target.PrettyPrint() << std::endl;

  return target;
}


template <typename MPTraits>
const Vector3d
DynamicRegionRRT<MPTraits>::
GetVelocityBias(SamplingRegion* _region) {
  // Get the region data.
  // const auto& regionData = m_regionData.at(region);
  const size_t index = _region->edgeIndex;

  // Find the skeleton edge path the region is traversing.
  auto reit = _region->edgeIterator;
  const auto& path = reit->property();

  // Helper to make the biasing direction and print debug info.
  auto makeBias = [&](const Vector3d& _start, const Vector3d& _end) {
    if(this->m_debug)
      std::cout << "Computed velocity bias: " << (_end - _start).normalize()
                << "\n\tStart: " << _start
                << "\n\tEnd:   " << _end
                << std::endl;
    return (_end - _start).normalize();
  };

  // If there is at least one valid path point after the current path index,
  // then return the direction to the next point.
  if(index < path.size() - 1) {
    if(this->m_debug)
      std::cout << "Biasing velocity along next path step"
                << "\n\tPath index: " << index
                << "\n\tPath size:  " << path.size()
                << std::endl;
    return makeBias(path[index], path[index + 1]);
  }

  // Otherwise, the region has reached a skeleton vertex.
  WorkspaceSkeleton::VD targetVD = reit->target();
  auto vertex = m_skeleton.FindVertex(targetVD);

  // If the vertex has no outgoing edges, this is the end of the skeleton. In
  // that case, use the previous biasing direction. All paths have at least two
  // points so this is safe.
  if(vertex->size() == 0) {
    if(this->m_debug)
      std::cout << "Biasing velocity along previous path step"
                << "\n\tPath index: " << index
                << "\n\tPath size:  " << path.size()
                << std::endl;
    return makeBias(path[index - 1], path[index]);
  }

  // Otherwise, randomly select an outgoing and use it's next point.
  auto eit = vertex->begin();
  const size_t nextEdgeIndex = LRand() % vertex->size();
  std::advance(eit, nextEdgeIndex);
  if(this->m_debug)
    std::cout << "Biasing velocity along next edge (index " << nextEdgeIndex
              << ")\n\tPath index: " << index
              << "\n\tPath size:  " << path.size()
              << "\n\tNext edge path size: " << eit->property().size()
              << std::endl;
  return makeBias(path[index], eit->property()[1]);
}


template <typename MPTraits>
bool
DynamicRegionRRT<MPTraits>::
IsTouching(const Cfg& _cfg, SamplingRegion& _region) {
  // Compute the penetration distance required. We want the robot's bounding
  // sphere to penetrate the region by the fraction m_penetrationThreshold.
  const double robotRadius  = _cfg.GetMultiBody()->GetBoundingSphereRadius(),
               threshold    = 2 * robotRadius * m_penetrationFactor;

  // Get the region boundary.
  const auto center = _region.GetCenter();
  auto boundary = MakeBoundary(center);

  // Compute the penetration distance (maximally enclosed bounding diameter).
  const Point3d robotCenter = _cfg.GetPoint();
  const double penetration = boundary.GetClearance(robotCenter) + robotRadius;

  // The configuration is touching if the penetration exceeds the threshold.
  const bool touching = penetration >= threshold;

  if(this->m_debug)
    std::cout << "\t Touch test: " << (touching ? "passed" : "failed")
              << "\n\t  Bounding sphere: " << robotCenter << " ; " << robotRadius
              << "\n\t  Region:          " << _region.GetCenter() << " ; "
              << m_regionRadius
              << "\n\t  Bounding sphere penetrates by "
              << std::setprecision(4)
              << penetration << (touching ? " >= " : " < ") << threshold
              << " units."
              << std::endl;

  return touching;
}


template <typename MPTraits>
CSpaceBoundingSphere
DynamicRegionRRT<MPTraits>::
MakeBoundary(const Vector3d& _v) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::MakeBoundary");

  const bool threeD = this->GetTask()->GetRobot()->GetMultiBody()->GetBaseType()
                   == Body::Type::Volumetric;

  // I'm not sure what the boundary code might do with a negative radius. Bound
  // it below at zero just in case.
  const double radius = std::max(0., m_regionRadius);

  if (threeD)
    return CSpaceBoundingSphere({_v[0], _v[1], _v[2]}, radius);
  else
    return CSpaceBoundingSphere({_v[0], _v[1]}, radius);
}

/*--------------------------- Skeleton and Workspace -------------------------*/

template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
BuildSkeleton() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::BuildSkeleton");

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
      mcs.SetEnvironment(this->GetEnvironment());
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
  DirectSkeleton();
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
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
    auto sampler = this->GetSampler(this->m_samplerLabel);
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

  m_queryPair = std::make_pair(start, goal);
}


template<typename MPTraits>
const size_t
DynamicRegionRRT<MPTraits>::
SelectSamplingRegion() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::SelectSamplingRegion");

  // Update all region probabilities.
  const std::vector<double> probabilities = ComputeProbabilities();


  // Select a region to sample from. The last region is the whole environment.
  double rand = DRand();
  double lowerBound = 0.0;
  int index;

  for(index = 0; index < (int)probabilities.size(); index++) {
    if((lowerBound < rand) and (rand < lowerBound + probabilities[index]))
      break;
    
    lowerBound += probabilities[index];
  }

  if(this->m_debug) {
    std::cout << "Computed region selection probabilities ("
              << "last is whole env):\n\t";

    for(auto p : probabilities)
      std::cout << std::setprecision(4) << p << " ";

    std::cout << "\n\tSelected index " << index
              << (index != (int)m_regions.size() ? "." : " (whole env).")
              << std::endl;
  }

  return index;
}


template <typename MPTraits>
std::vector<double>
DynamicRegionRRT<MPTraits>::
ComputeProbabilities() {

  // Sum all weights of all current regions.
  double totalWeight = 0.;
  for(auto r : m_regions) {
    totalWeight += r.GetWeight();
  }

  // Compute the probabilities for the current regions.
  std::vector<double> probabilities;
  probabilities.reserve(m_regions.size() + 1);

  const double explore = m_explore / (m_regions.size() + 1);

  for(auto r : m_regions) {
    const double exploit = (1 - m_explore) * r.GetWeight() / totalWeight;

    probabilities.emplace_back(exploit + explore);
  }

  // Get the probability for the whole environment.
  probabilities.emplace_back(explore);

  return probabilities;
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
BiasVelocity(CfgType& _cfg, SamplingRegion* _region) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::BiasVelocity");

  // Get the bias from the region kit.
  const Vector3d bias = GetVelocityBias(_region);
  if(bias.norm() == 0)
    throw RunTimeException(WHERE, "Bias cannot be zero.");

  // Resample the Cfg until its linear velocity aims relatively along the
  // biasing direction.
  Vector3d velocity;
  do {
    _cfg.GetRandomVelocity();
    velocity = _cfg.GetLinearVelocity().normalize();
    if(this->m_debug)
      std::cout << "\tSampled velocity direction: " << velocity
                << "\n\t\tDot product with bias: " << velocity * bias
                << (velocity * bias < m_velocityAlignment ? " < " : " >= ")
                << m_velocityAlignment
                << std::endl;
  } while(velocity * bias < m_velocityAlignment);
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
CheckRegionProximity(const Point3d& _p) {

  // Check each skeleton node to see if a new region should be created.
  for(auto iter = m_skeleton.begin(); iter != m_skeleton.end(); ++iter) {
    // Skip skeleton nodes that are too far away.
    const double dist = (iter->property() - _p).norm();
    if(dist >= m_regionRadius)
      continue;

    CreateRegions(iter);
  }
}


template <typename MPTraits>
std::vector<typename DynamicRegionRRT<MPTraits>::SamplingRegion*>
DynamicRegionRRT<MPTraits>::
CreateRegions(const WorkspaceSkeleton::vertex_iterator _iter) {
  // Skip skeleton nodes that are already visited.
  if(m_visited[_iter->descriptor()])
    return {};
  m_visited[_iter->descriptor()] = true;

  // Save the set of created regions to return.
  std::vector<SamplingRegion*> newRegions;

  // Create a new region for each outgoing edge of this skeleton node.
  for(auto eit = _iter->begin(); eit != _iter->end(); ++eit) {
    m_regions.push_back(SamplingRegion(eit));
    newRegions.push_back(&m_regions[m_regions.size() - 1]);

    if(this->m_debug)
      std::cout << "Created new region with radius "
                << std::setprecision(4) << m_regionRadius
                << " on edge (" << eit->source() << ", "
                << eit->target() << ", " << eit->id() << ") "
                << "at skeleton vertex " << _iter->descriptor()
                << "(" << _iter->property() << ")."
                << std::endl;
  }

  return newRegions;
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
AdvanceRegions(const Cfg& _cfg) {
  if(this->m_debug)
    std::cout << "Checking " << m_regions.size()
              << " regions for contact with new configuration "
              << _cfg.PrettyPrint() << "."
              << std::endl;

  // Keep track of any newly reached vertices to spawn regions on their outbound
  // edges.
  std::queue<WorkspaceSkeleton::VD> newlyReachedVertices;

  // Iterate through all existing regions to see which should be advanced.
  for(auto iter = m_regions.begin(); iter != m_regions.end(); ) {

    // Advance this region until the robot at _cfg is no longer touching it.
    if(!AdvanceRegionToCompletion(_cfg, *iter)) {
      ++iter;
      continue;
    }

    // We have reached the end of this region's edge. Delete it and save the
    // target vertex. to spawn new regions.
    auto eit = iter->edgeIterator;
    const auto target = eit->target();
    if(!m_visited[target])
      newlyReachedVertices.push(target);
    iter = m_regions.erase(iter);
  }

  // Create new regions for each newly reached vertex.
  while(!newlyReachedVertices.empty()) {
    // Pop the next vertex off the queue.
    WorkspaceSkeleton::VD targetVD = newlyReachedVertices.front();
    WorkspaceSkeleton::vertex_iterator target = m_skeleton.FindVertex(targetVD);
    newlyReachedVertices.pop();

    // Create regions at this vertex.
    std::vector<SamplingRegion*> newRegions = CreateRegions(target);

    // Advance each new region.
    for(auto region : newRegions) {
      // Advance this region until the robot at _cfg is no longer touching it.
      if(!AdvanceRegionToCompletion(_cfg, *region))
        continue;

      // We have reached the end of this region's edge. Delete it and save the
      // target vertex. to spawn new regions.
      auto iter = std::find(m_regions.begin(), m_regions.end(), *region);
      auto eit = region->edgeIterator;
      newlyReachedVertices.push(eit->target());
      m_regions.erase(iter);
    }
  }
}


template <typename MPTraits>
bool
DynamicRegionRRT<MPTraits>::
AdvanceRegionToCompletion(const Cfg& _cfg, SamplingRegion& _region) {
  // Find the edge path this region is traversing.
  auto eit = _region.edgeIterator;
  const std::vector<Point3d>& path = eit->property();
  size_t& i = _region.edgeIndex;

  if(this->m_debug)
    std::cout << "\tChecking region at "
              << _region.GetCenter() << "."
              << "\n\t Region is at index " << i << " / " << path.size() - 1
              << std::endl;

  while(IsTouching(_cfg, _region)) {

    // If there are no more points left on this edge, this region is completed.
    if(_region.LastPoint()) {
      if(this->m_debug)
        std::cout << "\t Region has reached the end of its "
                  << "path, erasing it now. " << m_regions.size() - 1
                  << " regions remain."
                  << std::endl;

      return true;
    }

    // Otherwise there are still points left; advance the region and index.
    _region.Advance();

    if(this->m_debug)
      std::cout << "\t Advancing region from index "
                << i - 1 << " to " << i << " / " << path.size() - 1 << "."
                << std::endl;
  }

  if(this->m_debug)
    std::cout << "\t Region is still traversing this edge." << std::endl;

  return false;
}

#endif
