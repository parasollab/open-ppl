#ifndef PPL_COMPOSITE_DYNAMIC_REGION_RRT_LITE_H_
#define PPL_COMPOSITE_DYNAMIC_REGION_RRT_LITE_H_

#include "GroupRRTStrategy.h"

#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "Utilities/XMLNode.h"
#include "Utilities/MPUtils.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "Workspace/CompositeWorkspaceSkeleton.h"

////////////////////////////////////////////////////////////////////////////////
/// A lite version of the Composite Dynamic Region-biased RRT algorithm that
/// grounds an input composite skeleton edge without the frill.
///
/// An RRT guided by a composite workspace skeleton.
///
/// Reference:
///   Coming soon
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////

template <typename MPTraits>
class CDRRRTLite : virtual public GroupRRTStrategy<MPTraits> {
  public:
    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;

    // Add robot group types
    typedef typename MPTraits::GroupCfgType     GroupCfgType;
    typedef typename MPTraits::GroupWeightType  GroupWeightType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename GroupRoadmapType::VID      VID;
    
    typedef typename std::map<Robot*, VID>      VIDMap;

    typedef std::vector<Point3d>                PointSet;
    typedef std::map<Robot*, const Boundary*>   BoundaryMap;
    typedef std::map<Robot*, Vector3d>          VectorMap;

    ///@}
    ///@name WorkspaceSkeleton Types
    ///@{

    typedef typename MPTraits::CompositeSkeletonVertex              CompositeSkeletonVertex;
    typedef typename MPTraits::CompositeSkeletonEdge                CompositeSkeletonEdge;
    typedef typename MPTraits::CompositeSkeletonType                CompositeSkeletonType;
    typedef typename CompositeSkeletonType::ED                      SkeletonEdgeDescriptor;
    typedef typename CompositeSkeletonType::adj_edge_iterator       SkeletonEdgeIterator;
    typedef typename CompositeSkeletonType::vertex_descriptor       SkeletonVertexDescriptor;
    typedef typename CompositeSkeletonType::vertex_iterator         SkeletonVertexIterator;

    ///@}
    ///@name Local Types
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// Representation of a sampling region.
    ////////////////////////////////////////////////////////////////////////////
    struct SamplingRegion {

      ///@name Internal State
      ///@{
      
      CompositeSkeletonEdge edge;

      size_t edgeIndex{0};   ///< Which edge point are we at?
      double attempts{1};    ///< Number of attempts to extend into this region.
      double successes{1};   ///< Number of successful attempts.

      std::unordered_set<Robot*> activeRobots; ///< The robots that move on the edge.

      ///@}

      SamplingRegion() {}

      SamplingRegion(const CompositeSkeletonEdge& _edge) : 
            edge(_edge) {
        activeRobots = _edge.GetActiveRobots();
      }

      /// Track the success rate of extending into this region.
      void TrackSuccess(const size_t _success, const size_t _attempts) {
        successes *= .9;
        attempts  *= .9;
        successes += _success;
        attempts  += _attempts;
      }

      void IncrementSuccess() {
        successes += 1;
      }

      void IncrementAttempts() {
        attempts += 1;
      }

      /// Get the center of this region.
      const VectorMap GetCenter() const noexcept {
        VectorMap center;
        auto compState = edge.GetIntermediates()[edgeIndex];

        for(auto r : compState.GetRobots()) {
          center.insert(std::pair<Robot*, Vector3d>(r, compState.GetRobotCfg(r)));
        }

        return center;
      }

      /// Check if this region is at the last point on its skeleton edge.
      bool LastPoint() const noexcept {
        return edgeIndex == edge.GetNumIntermediates() - 1;
      }

      /// Advance this region to the next skeleton edge point.
      void Advance() noexcept {
        ++edgeIndex;
      }

      /// Assignment operator
      SamplingRegion& operator=(const SamplingRegion& _region) {
        if(this != &_region) {
          edge = _region.edge;
          edgeIndex = _region.edgeIndex;
          attempts = _region.attempts;
          successes = _region.successes;
          activeRobots = _region.activeRobots;
        }
        return *this;
      }

      /// Equality operator
      bool operator==(const SamplingRegion& _region) const {
        // const bool eit = edgeIterator == _region.edgeIterator;
        // const bool idx = edgeIndex == _region.edgeIndex;
        // const bool att = attempts == _region.attempts;
        // const bool succ = successes == _region.successes;
        // return eit and idx and att and succ;
        return edge == _region.edge;
      }

    };

    ///@}
    ///@name Construction
    ///@{

    CDRRRTLite();

    CDRRRTLite(XMLNode& _node);

    virtual ~CDRRRTLite() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Hack for WoDaSH
    ///@{

    virtual void GroundEdge(const CompositeSkeletonEdge& _edge) override;

    ///@}

  protected:
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;

    ///@}
    ///@name GroupRRTStrategy Overrides
    ///@{

    /// Get a random configuration to grow towards.
    // virtual CfgType SelectIndividualTarget(SamplingRegion* _region);
    virtual GroupCfgType SelectTarget() override;

    /// Add a new configuration to the roadmap and current tree.
    /// @param _newCfg The new configuration to add.
    /// @return A pair with the added VID and a bool indicating whether the new
    ///         node was already in the map.
    virtual std::pair<VID, bool> AddNode(GroupCfgType& _newCfg) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Sample a configuration from within a sampling region using the sampler
    /// given in m_samplerLabel.
    /// @param _region The region to sample from.
    /// @return A configuration with the sampling region.
    std::pair<bool, GroupCfgType> Sample(SamplingRegion* _region);

    /// Determine if a region is touching a configuration.
    /// @param _cfg The configuration.
    /// @param _region The sampling region.
    bool IsTouching(const GroupCfgType& _cfg, SamplingRegion& _region);

    /// Calculate the boundary around a sampling region.
    /// @param _v The center of the sampling region.
    /// @return The boundary with center _v and radius m_regionRadius.
    CSpaceBoundingSphere MakeBoundary(Robot* _robot, const VectorMap _v);

    ///@}
    ///@name Skeleton and Workspace
    ///@{

    /// Advance a region until it is either not longer touching a configuration
    /// or until it reaches the end of its respective skeleton edge.
    /// @param _cfg A configuration possibly touching the region.
    /// @param _region The region to advance along its skeleton edge.
    bool AdvanceRegionToCompletion(const GroupCfgType& _cfg, SamplingRegion* _region);

    ///@}
    ///@name Internal State
    ///@{

    SamplingRegion m_region;

    size_t m_maxSampleFails{20};

    /// The dynamic sampling regions will have radius equal to this times the
    /// robot group's bounding sphere radius.
    double m_regionFactor{2};

    /// A configuration is considered to be touching a region when this fraction
    /// of its bounding sphere penetrates into the region.
    double m_penetrationFactor{1};

    bool m_done{false}; // finished grounding edge

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
CDRRRTLite<MPTraits>::
CDRRRTLite() : GroupRRTStrategy<MPTraits>() {
    this->SetName("CDRRRTLite");
}

template <typename MPTraits>
CDRRRTLite<MPTraits>::
CDRRRTLite(XMLNode& _node) : GroupRRTStrategy<MPTraits>(_node) {
  this->SetName("CDRRRTLite");

  m_maxSampleFails = _node.Read("maxSampleFails", false, m_maxSampleFails, 
      (size_t)1, SIZE_MAX, 
      "Maximum number of failures before abandoning a region");

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
CDRRRTLite<MPTraits>::Print(std::ostream& _os) const {
  GroupRRTStrategy<MPTraits>::Print(_os);

  _os << "\tMaximum Number of Region Failures: " << m_maxSampleFails << std::endl;
  _os << "\tRegion Factor: " << m_regionFactor << std::endl;
  _os << "\tPenetration Factor: " << m_penetrationFactor << std::endl;
}

/*---------------------------- MPStrategy Overrides --------------------------*/

template <typename MPTraits>
void
CDRRRTLite<MPTraits>::Initialize(){
  GroupRRTStrategy<MPTraits>::Initialize();

  MethodTimer mt(this->GetStatClass(),
        this->GetNameAndLabel() + "::InitializeRoadmap");

  // Check that only one direction is being extended.
  if(this->m_numDirections > 1)
    throw RunTimeException(WHERE) << "Extending more than one direction "
                                  << "is not supported.";
  if(this->m_growGoals)
    throw RunTimeException(WHERE) << "Bidirectional growth is not supported.";

  if(this->m_goalDmLabel.empty())
    throw RunTimeException(WHERE) << "Goal distance metric label is required.";
}


template <typename MPTraits>
void
CDRRRTLite<MPTraits>::
Iterate() {
  // Find growth target.
  const GroupCfgType target = this->SelectTarget();

  auto t = target;
  if(t.GetGroup() == nullptr)
    return;

  auto vid = this->ExpandTree(target);
  if(vid != INVALID_VID and this->m_restrictGrowth)
    this->m_vids.insert(vid);
}

/*------------------------ GroupRRTStrategy Overrides ------------------------*/

template <typename MPTraits>
typename MPTraits::GroupCfgType
CDRRRTLite<MPTraits>::
SelectTarget() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SelectTarget");

  GroupCfgType gcfg;

  if(m_region.attempts - m_region.successes > m_maxSampleFails) {
    m_done = true;
    return gcfg;
  }

  bool notFound = true;
  size_t attempts = 0;
  while(notFound and attempts < 1000) {
    notFound = false;
    attempts++;

    auto result = Sample(&m_region);
    notFound = not result.first;
    m_region.IncrementAttempts();
    
    if(notFound)
        continue;

    return result.second;
  }

  return gcfg;
}

template <typename MPTraits>
std::pair<typename CDRRRTLite<MPTraits>::VID, bool>
CDRRRTLite<MPTraits>::
AddNode(GroupCfgType& _newCfg) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::AddNode");

  auto g = this->GetGroupRoadmap();

  const VID lastVID = g->GetLastVID();
  const VID newVID  = g->AddVertex(_newCfg);

  const bool nodeIsNew = lastVID != g->GetLastVID();
  if(nodeIsNew) {
    if(this->m_debug)
      std::cout << "\tAdding VID " << newVID << "."
                << std::endl;
  }

  if(nodeIsNew) {

    m_region.IncrementSuccess();

    // On each new sample, check if we need to advance our region.
    auto vi = g->find_vertex(newVID);

    auto compState = vi->property();
    m_done = AdvanceRegionToCompletion(compState, &m_region);
  }

  return {newVID, nodeIsNew};
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
std::pair<bool, typename MPTraits::GroupCfgType>
CDRRRTLite<MPTraits>::
Sample(SamplingRegion* _region) {
  if (this->m_debug) {
    std::cout << "\tSampling from region"
              << " (intermediate "
              << _region->edgeIndex << "/"
              << _region->edge.GetNumIntermediates()
              << "), success rate so far "
              << _region->successes << " / " << _region->attempts
              << ", using sampler '" << this->m_samplerLabel << "'." << std::endl;

    for(auto r : _region->activeRobots) {
      std::cout << "\t Region centered at " << _region->GetCenter().at(r)
                << "for robot " << r->GetLabel() << "." << std::endl;
    }
  }

  // Get a boundary map for the composite region.
  auto robots = this->GetGroupTask()->GetRobotGroup()->GetRobots();
  const auto center = _region->GetCenter();

  std::vector<CSpaceBoundingSphere> bounds;
  for(auto robot : robots) {
    bounds.push_back(MakeBoundary(robot, center));
  }

  BoundaryMap boundMap;
  for(size_t i = 0; i < robots.size(); i++) {
    boundMap.emplace(std::make_pair(robots.at(i), &bounds.at(i)));
  }

  // Get the sampler.
  auto s = this->GetSampler(this->m_samplerLabel);

  // Try to sample 10 times before we consider it a failure
  std::vector<GroupCfgType> samples, collision;
  int tries = 0;

  while(tries < 10 and samples.empty()) {
    ++tries;
    s->Sample(1, 100, boundMap, std::back_inserter(samples),
      std::back_inserter(collision));
  }

  if(samples.empty()) {
    return std::make_pair<bool, GroupCfgType>(false, GroupCfgType());
  }

  auto& target = samples.front();

  if(this->m_debug)
    std::cout << "\t" << target.PrettyPrint() << std::endl;

  return std::make_pair<bool,GroupCfgType>(true, std::move(target));
}


template <typename MPTraits>
bool
CDRRRTLite<MPTraits>::
IsTouching(const GroupCfgType& _groupCfg, SamplingRegion& _region) {
  const auto center = _region.GetCenter();

  bool touching = true;
  for (auto r : _region.activeRobots) {
    auto cfg = _groupCfg.GetRobotCfg(r);

    // Compute the penetration distance required. We want the robot's bounding
    // sphere to penetrate the region by the fraction m_penetrationThreshold.
    const double robotRadius  = cfg.GetMultiBody()->GetBoundingSphereRadius(),
                 threshold    = 2 * robotRadius * m_penetrationFactor;

    // Get the region boundary.
    auto boundary = MakeBoundary(r, center);

    // Compute the penetration distance (maximally enclosed bounding diameter).
    const Point3d robotCenter = cfg.GetPoint();
    const double penetration = boundary.GetClearance(robotCenter) + robotRadius;

    // The configuration is touching if the penetration exceeds the threshold.
    if(penetration < threshold) {
      touching = false;
      break;
    }
  }

  if(this->m_debug)
    std::cout << "\t Touch test: " << (touching ? "passed" : "failed") << std::endl;

  return touching;
}


template <typename MPTraits>
CSpaceBoundingSphere
CDRRRTLite<MPTraits>::
MakeBoundary(Robot* _robot, const VectorMap _v) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::MakeBoundary");

  const bool threeD = _robot->GetMultiBody()->GetBaseType()
                   == Body::Type::Volumetric;

  auto indV = _v.at(_robot);

  // I'm not sure what the boundary code might do with a negative radius. Bound
  // it below at zero just in case.
  auto regRadius = _robot->GetMultiBody()->GetBoundingSphereRadius();
  const double radius = std::max(0., m_regionFactor * regRadius);

  if (threeD)
    return CSpaceBoundingSphere({indV[0], indV[1], indV[2]}, radius);
  else
    return CSpaceBoundingSphere({indV[0], indV[1]}, radius);
}

/*--------------------------- Skeleton and Workspace -------------------------*/

template <typename MPTraits>
bool
CDRRRTLite<MPTraits>::
AdvanceRegionToCompletion(const GroupCfgType& _cfg, SamplingRegion* _region) {
  // Find the edge path this region is traversing.
  const auto path = _region->edge.GetIntermediates();
  size_t& i = _region->edgeIndex;

  if(this->m_debug)
    std::cout << "\tChecking region at "
              << _region->GetCenter() << "."
              << "\n\t Region is at index " << i << " / " << path.size() - 1
              << std::endl;

  while(IsTouching(_cfg, *_region)) {

    // If there are no more points left on this edge, this region is completed.
    if(_region->LastPoint()) {
      if(this->m_debug)
        std::cout << "\t Region has reached the end of its path." << std::endl;

      return true;
    }

    // Otherwise there are still points left; advance the region and index.
    _region->Advance();

    if(this->m_debug)
      std::cout << "\t Advancing region from index "
                << i - 1 << " to " << i << " / " << path.size() - 1 << "."
                << std::endl;
  }

  if(this->m_debug)
    std::cout << "\t Region is still traversing this edge." << std::endl;

  return false;
}


template <typename MPTraits>
void
CDRRRTLite<MPTraits>::
GroundEdge(const CompositeSkeletonEdge& _edge){
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::GroundEdge");

  m_done = false;
  m_region = SamplingRegion(_edge);

  while(!m_done)
    this->Iterate();
}

#endif
