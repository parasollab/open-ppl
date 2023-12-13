#ifndef PPL_DYNAMIC_REGION_RRT_H_
#define PPL_DYNAMIC_REGION_RRT_H_

#include "BasicRRTStrategy.h"

#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "Utilities/XMLNode.h"
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
class DynamicRegionRRT : virtual public BasicRRTStrategy {
  public:
    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::WeightType   WeightType;
    typedef typename MPBaseObject::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID           VID;

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
    virtual Cfg SelectTarget() override;

    /// Add a new configuration to the roadmap and current tree.
    /// @param _newCfg The new configuration to add.
    /// @return A pair with the added VID and a bool indicating whether the new
    ///         node was already in the map.
    virtual std::pair<VID, bool> AddNode(const Cfg& _newCfg) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Sample a configuration from within a sampling region using the sampler
    /// given in m_samplerLabel.
    /// @param _region The region to sample from.
    /// @return A configuration with the sampling region.
    Cfg Sample(SamplingRegion* _region);

    /// Sample a configuration from within a given boundary using the sampler
    /// given in _samplerLabel.
    /// @param _region The region to sample from.
    /// @return A configuration with the boundary.
    Cfg Sample(const Boundary* const _boundary, const std::string* _samplerLabel);

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
    void BiasVelocity(Cfg& _cfg, SamplingRegion* _region);

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

#endif
