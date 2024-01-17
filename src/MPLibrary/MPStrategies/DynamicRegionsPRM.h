#ifndef PMPL_DYNAMIC_REGIONS_PRM_H_
#define PMPL_DYNAMIC_REGIONS_PRM_H_

#include "MPStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/NeighborhoodFinders/Neighbors.h"
#include "Utilities/XMLNode.h"
#include "Workspace/WorkspaceSkeleton.h"


////////////////////////////////////////////////////////////////////////////////
/// Dynamic Regions PRM algorithm.
///
/// A PRM guided by a workspace skeleton.
///
/// Reference:
///   Read Sandstrom. "Approximating Configuration Space Topology with Workspace
///   Models". PhD Thesis, Spring 2020.
///   -and-
///   Read Sandstrom, Diane Uwacu, Jory Denny, and Nancy M. Amato.
///   "Topology-Guided Roadmap Construction with Dynamic Region Sampling".
///   Under review for RA-L @ IROS 20.
///
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class DynamicRegionsPRM : public MPStrategyMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::WeightType   WeightType;
    typedef typename MPBaseObject::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID           VID;
    typedef typename RoadmapType::VertexSet     VertexSet;

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
    /// Settings for a specific sampler.
    ////////////////////////////////////////////////////////////////////////////
    struct SamplerSetting {
      std::string label;  ///< The sampler label.
      size_t count;       ///< The number of samples to generate.
      size_t attempts;    ///< The number of attempts to generate each sample.
    };

    ////////////////////////////////////////////////////////////////////////////
    /// Output for connection attempt.
    ////////////////////////////////////////////////////////////////////////////
    struct EdgeOutput {
      VID source;
      VID target;
      std::pair<WeightType, WeightType> weights;
    };

    ////////////////////////////////////////////////////////////////////////////
    /// A descriptor for a local connected component.
    ////////////////////////////////////////////////////////////////////////////
    struct LocalComponentDescriptor {
      SkeletonEdgeDescriptor edgeDescriptor; ///< The edge this component is on.
      VID representative;                    ///< Representative roadmap vertex.
      bool bridge;                           ///< Does this span the edge?

      std::ostream& Print(std::ostream& _os) const {
        return _os << "(" << edgeDescriptor.id()
                   << "|"
                   << edgeDescriptor.source() << "," << edgeDescriptor.target()
                   << "|"
                   << representative
                   << "|" << (bridge ? "+" : "-") << ")";
      }

      friend std::ostream& operator<<(std::ostream& _os,
          const LocalComponentDescriptor& _d) {
        return _d.Print(_os);
      }
    };

    ////////////////////////////////////////////////////////////////////////////
    /// Representation of an expansion region.
    ////////////////////////////////////////////////////////////////////////////
    struct ExpansionRegion {

      ///@name Internal State
      ///@{

      const SkeletonEdgeIterator edgeIterator; ///< Iterator to region's edge.
      VID representative{INVALID_VID};       ///< Representative roadmap vertex.
      size_t edgeIndex{0};   ///< Which edge point are we at?
      double attempts{1};    ///< Number of attempts to extend into this region.
      double successes{0};   ///< Number of successful attempts.

      ///@}

      ExpansionRegion(const SkeletonEdgeIterator& _eit, const VID _representative)
          : edgeIterator(_eit), representative(_representative) {}

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

      /// Get the descriptor for the local component this region is expanding.
      LocalComponentDescriptor GetLocalComponentDescriptor() const noexcept {
        return {(*edgeIterator).descriptor(), representative, false};
      }

    };

    /// Ordering operator for stapl edge descriptors, which includes skeleton
    /// edges. GCC refuses to acknowledge a global definition of operator< for
    /// this purpose despite a perfect signature match, I assume it's related
    /// to some stapl stupidity and falling back to a dedicated object.
    struct EdgeCompare {
      bool
      operator()(const SkeletonEdgeDescriptor& _d1,
          const SkeletonEdgeDescriptor& _d2) const noexcept {
        // Test ID first since we should only ever have two edges on the same ID (one
        // for each direction). If ID are equal, sort by source, then target.
        return _d1.id() < _d2.id()
            or (!(_d1.id() > _d2.id())
                and (_d1.source() < _d2.source()
                     or (!(_d1.source() > _d2.source())
                         and _d1.target() < _d2.target()
                        )
                    )
               );
      }
    };

    /// Ordering operator for stapl edge descriptors for bridges. Only cares
    /// about the edge ID.
    struct EdgeIDCompare {
      bool
      operator()(const SkeletonEdgeDescriptor& _d1,
          const SkeletonEdgeDescriptor& _d2) const noexcept {
        return _d1.id() < _d2.id();
      }
    };

    /// Map for local connected components. Maps skeleton edge to representative
    /// to vids.
    typedef std::map<Robot*, std::map<SkeletonEdgeDescriptor, std::map<VID, VertexSet>, EdgeCompare>>
        LocalConnectivityMap;

    /// Bridges are non-directional and tied to edge IDs.
    typedef std::map<Robot*, std::map<SkeletonEdgeDescriptor, std::map<VID, VertexSet>, EdgeIDCompare>>
        BridgeMap;

    /// Map for regions.
    typedef std::map<Robot*, std::map<SkeletonEdgeDescriptor, std::map<VID, ExpansionRegion>, EdgeCompare>>
        RegionMap;

    /// Map for low clearance.
    typedef std::map<Robot*, std::set<size_t>> LowClearanceMap;

    /// Map for regions with unconnected local components.
    typedef std::map<Robot*, std::set<SkeletonEdgeDescriptor, EdgeCompare>> UnconnectedEdgeMap;

    ///@}
    ///@name Construction
    ///@{

    DynamicRegionsPRM();

    DynamicRegionsPRM(XMLNode& _node);

    virtual ~DynamicRegionsPRM() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Iterate() override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Add start and goals to the roadmap
    void AddQuery();

    /// Hax.
    bool CheckReachedRegion(const VID _query, const VertexSet& _component);

    /// Grow a tree from a vertex q
    /// Stop when q connects to the rest of the roadmap or is within the
    /// boundary of a skeleton vertex
    void GrowRRT(const VID _q);

    ///@}

    /// Expand a component in region r
    /// @return the list of new cfgs added to the component
    VertexSet ExpandComponent(ExpansionRegion* const _r);

    /// Connect local components within an edge segment.
    /// @param _ed The edge segment to connect.
    void ConnectEdgeSegment(const SkeletonEdgeDescriptor _ed);

    // Initialize the unconnected edge map
    void InitializeLUnconnectedEdgeMap();

    ///@name Local Connected Components
    ///@{

    /// Initialize tracking for a local connected component.
    /// @param _d The descriptor for the local component.
    /// @param _vids All roadmap vertices initially in the component.
    void MakeLocalComponent(const LocalComponentDescriptor& _d,
        const VertexSet& _vids);

    /// Get a local component's vertex set.
    /// @param _d The descriptor for the local component.
    /// @return The set of VIDs in this component.
    VertexSet& GetLocalComponent(const LocalComponentDescriptor& _d) noexcept;

    /// Get the expansion region for a local component.
    /// @param _d The descriptor for the local component.
    /// @return The expansion region for this component.
    ExpansionRegion* GetExpansionRegion(const LocalComponentDescriptor& _d)
        noexcept;

    /// Get a bridge's vertex set.
    /// @param _d The descriptor for the bridge.
    /// @return The set of VIDs in this bridge.
    VertexSet& GetBridge(const LocalComponentDescriptor& _d) noexcept;

    /// Get the local component descriptor for a vertex.
    /// @warning This is a linear scan over all local CCs, use very sparingly.
    LocalComponentDescriptor FindLocalComponent(const VID _vid) const noexcept;

    /// Promote a local connected component to a bridge.
    /// @param _d The descriptor for the local component.
    LocalComponentDescriptor PromoteLocalComponent(LocalComponentDescriptor _d);

    /// Merge two local connected components. If they are both incomplete and
    /// from the same side of the edge, the one whos region is further ahead
    /// will be preserved. If they are from different sides of the edge, a
    /// bridge will be formed.
    /// @param _d1 The descriptor for the first component.
    /// @param _d2 The descriptor for the second component.
    /// @return The descriptor of the surviving component.
    LocalComponentDescriptor MergeLocalComponents(
        const LocalComponentDescriptor& _d1,
        const LocalComponentDescriptor& _d2);

    /// Update the local connectivity for an edge.
    void UpdateEdgeConnectivity(const SkeletonEdgeDescriptor& _ed);

    /// Check if an edge is considered locally connected.
    bool IsEdgeConnected(const SkeletonEdgeDescriptor& _ed);

    ///@}
    ///@name Region Functions
    ///@{

    /// Select a region based weighted success probabilities
    /// @return expansion region to be expanded
    ExpansionRegion* SelectExpansionRegion();

    /// Compute probabilities for selecting each expansion region.
    /// @return probabiliities based on expansion success
    std::pair<std::vector<double>, std::vector<ExpansionRegion*>>
        ComputeProbabilities();

    /// Advance a region along an edge.
    /// @param _r The region to advance.
    /// @param _newVIDs The recently added VIDs from sampling in this region.
    /// @return True if the region is successfully advanced, false if the region
    ///         reaches end of the edge or an low clearance area.
    bool AdvanceRegion(ExpansionRegion* const _r,
        const VertexSet& _newVIDs);

    /// Check if region still covers a subset of the given samples set
    /// @param _region The region to check.
    /// @param _samples The newly generated samples to check.
    /// @return True if any of _samples is contained by _region.
    bool AreSamplesCovered(const ExpansionRegion* const _region,
        const VertexSet& _samples);

    ///@}
    ///@name General Planning
    ///@{

    /// Sample and add configurations to the roadmap.
    /// @return The generated VIDs for the successful samples.
    std::vector<Cfg> Sample(const Boundary* _b = nullptr);

    /// Return K nearest neighors from a set of candidates
    std::vector<Neighbor> FindNearestNeighbors(const Cfg& _cfg,
        const VertexSet* const _candidates);

    /// Attempt connections between a configuration and its neighbors
    /// @return the set of neighbors that successfully connect to _c
    bool AttemptConnection(const Cfg& _c1, const Cfg& _c2,
        LPOutput& _lpOuptut);

    /// Try to connect a configuration to a local component.
    /// @param _cfg The joining sample.
    /// @param _d The component descriptor.
    /// @return The generated edges.
    std::vector<EdgeOutput> ConnectToComponent(const Cfg& _cfg,
        const LocalComponentDescriptor& _d);

    /// Extend a tree node towards a direction
    VID Extend(const VID _nearVID, const Cfg& _target,
        LPOutput& _lp);

    ///@}
    ///@name Workspace and Clearance
    ///@{

    /// Make a boundary
    /// @return A boundary centered at _v
    CSpaceBoundingSphere MakeBoundary(const Vector3d& _v);

    /// Get region radius
    /// return radius = robot clearance - robot radius
    double GetRegionRadius(const Vector3d& _v);

    /// Get clearance
    /// @return robot clearance from closest obstacle
    double GetClearance(const Vector3d& _v);

    /// Build topological skeleton
    void BuildSkeleton();

    // Initialize the low clearance map
    void InitializeLowClearanceMap();

    ///@}
    ///@name Internal State
    ///@{

    std::vector<SamplerSetting> m_samplers;     ///< Samplers to generate nodes.
    std::string m_nfLabel;            ///< The neighborhood finder label.
    std::string m_lpLabel;            ///< The neighborhood finder label.
    std::string m_exLabel;            ///< The extender label.
    std::string m_decompositionLabel; ///< The workspace decomposition label.

    std::string m_skeletonType{"reeb"}; ///< Type of skeleton to build.
    std::string m_skeletonFilename;  ///< The output file for the skeleton graph
    std::string m_skeletonIO;        ///< Option to read or write the skeleton

    /// Weight of explore vs. exploit in region selection probabilities.
    /// Exploring is a uniform chance to select each region, while exploit
    /// favors successful regions.
    double m_explore{.5};

    /// Optional minimum radius for regions. -1 indicates no limit, allowing
    /// low-clearance regions to be pruned.
    double m_minRegionRadius{-1};

    /// After extending a local component, aggressively try to bridge across to
    /// the other side from the newly added configurations.
    bool m_aggressiveBridging{true};

    ///@}
    ///@name Shared State
    ///@{

    static bool m_initialized; ///< Is the shared state initialized?

    static WorkspaceSkeleton m_skeleton;     ///< The workspace skeleton.

    static LocalConnectivityMap m_localComponents; ///< Local components in progress.
    static BridgeMap m_bridges;                    ///< Completed local components.

    static RegionMap m_regions; ///< Expansion regions for local components.

    static LowClearanceMap m_lowClearanceMap; ///< Track low-clearance edges.

    static UnconnectedEdgeMap m_unconnectedEdges; ///< Track unconnected edges.

    ///@}

};

#endif
