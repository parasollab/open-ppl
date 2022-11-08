#ifndef PMPL_DYNAMIC_REGIONS_PRM_H_
#define PMPL_DYNAMIC_REGIONS_PRM_H_

#include "MPStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "MPLibrary/MPTools/MeanCurvatureSkeleton3D.h"
#include "MPLibrary/MPTools/ReebGraphConstruction.h"
#include "Utilities/MedialAxis2D.h"
#include "Utilities/XMLNode.h"
#include "Workspace/WorkspaceSkeleton.h"

#include <unordered_map>
#include <unordered_set>
#include <utility>




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
template <typename MPTraits>
class DynamicRegionsPRM : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::VertexSet VertexSet;

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
    std::vector<CfgType> Sample(const Boundary* _b = nullptr);

    /// Return K nearest neighors from a set of candidates
    std::vector<Neighbor> FindNearestNeighbors(const CfgType& _cfg,
        const VertexSet* const _candidates);

    /// Attempt connections between a configuration and its neighbors
    /// @return the set of neighbors that successfully connect to _c
    bool AttemptConnection(const CfgType& _c1, const CfgType& _c2,
        LPOutput<MPTraits>& _lpOuptut);

    /// Try to connect a configuration to a local component.
    /// @param _cfg The joining sample.
    /// @param _d The component descriptor.
    /// @return The generated edges.
    std::vector<EdgeOutput> ConnectToComponent(const CfgType& _cfg,
        const LocalComponentDescriptor& _d);

    /// Extend a tree node towards a direction
    VID Extend(const VID _nearVID, const CfgType& _target,
        LPOutput<MPTraits>& _lp);

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

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::m_initialized = false;

template <typename MPTraits>
WorkspaceSkeleton
DynamicRegionsPRM<MPTraits>::m_skeleton;

template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::LocalConnectivityMap
DynamicRegionsPRM<MPTraits>::m_localComponents;

template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::BridgeMap
DynamicRegionsPRM<MPTraits>::m_bridges;

template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::RegionMap
DynamicRegionsPRM<MPTraits>::m_regions;

template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::LowClearanceMap
DynamicRegionsPRM<MPTraits>::m_lowClearanceMap;

template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::UnconnectedEdgeMap
DynamicRegionsPRM<MPTraits>::m_unconnectedEdges;


template <typename MPTraits>
DynamicRegionsPRM<MPTraits>::
DynamicRegionsPRM() {
  this->SetName("DynamicRegionsPRM");
}


template <typename MPTraits>
DynamicRegionsPRM<MPTraits>::
DynamicRegionsPRM(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("DynamicRegionsPRM");

  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local Planner");
  m_exLabel = _node.Read("extenderLabel", true, "", "Extender label");

  for(auto& child : _node) {
    if(child.Name() == "Sampler") {
      SamplerSetting s;
      s.label = child.Read("label", true, "", "Sampler Label");
      s.count = child.Read("number", true,
          1, 0, MAX_INT, "Number of samples");
      s.attempts = child.Read("attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
      m_samplers.push_back(s);
    }
  }

  m_skeletonType = _node.Read("skeletonType", true, "",
      "the type of skeleton to use, Available options are reeb and mcs "
      "for 3d, ma for 2d");

  m_skeletonIO = _node.Read("skeletonIO", false, "", "read of write the "
      "skeleton file");

  m_skeletonFilename = _node.Read("skeletonFile", m_skeletonIO != "", "",
      "the skeleton file to read from or write to");

  // If using a reeb skeleton, we need a decomposition to build it.
  m_decompositionLabel = _node.Read("decompositionLabel",
      m_skeletonType == "reeb", "",
      "The workspace decomposition to use.");

  m_explore = _node.Read("exploreBias", false, m_explore, 0., 1.,
      "Preference for exploring vs. exploiting successful regions. "
      "1 indicates pure explore, 0 indicates pure exploit.");

  m_minRegionRadius = _node.Read("minRegionRadius", false, m_minRegionRadius,
      0., std::numeric_limits<double>::max(), "Optional minimum radius for "
      "regions. Leaving it unset indicates no limit, allowing low-clearance "
      "regions to be pruned. This likely will not work well without either "
      "a high-clearance environment or a medial axis skeleton.");

  m_aggressiveBridging = _node.Read("aggressiveBridging", false,
      m_aggressiveBridging, "After extending a local component, aggressively "
      "try to bridge across to the other side from the newly added "
      "configurations.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
Print(std::ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);

  _os << "\tSkeleton IO: " << m_skeletonIO << std::endl;

  _os << "\tSkeleton File: " << m_skeletonFilename << std::endl;

  _os << "\tSamplers" << std::endl;
  for(const auto& sampler : m_samplers)
    _os << "\t\t" << sampler.label
        << "\n\t\t  Number:   " << sampler.count
        << "\n\t\t  Attempts: " << sampler.attempts
        << std::endl;
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
Initialize() {
  // Initialize the skeleton, local components, and regions.
  if(!m_initialized) {
    m_initialized = true;
    BuildSkeleton();

    MethodTimer mt(this->GetStatClass(),
        this->GetNameAndLabel() + "::InitializeRoadmap");
    if(this->m_debug)
      std::cout << "Initializing regions at skeleton vertices."
                << std::endl;

    auto r = this->GetRoadmap();
    LPOutput<MPTraits> lpOutput;
    size_t componentCount = 0;
    size_t regionCount = 0;

    // Check each skeleton node to see if expansion regions should be created.
    auto& skeletonGraph = m_skeleton;
    for(auto iter = skeletonGraph.begin(); iter != skeletonGraph.end(); ++iter) {
      if(this->m_debug)
        std::cout << "\tVertex " << iter->descriptor()
                  << " at " << iter->property() << ":"
                  << std::endl;

      // Skip vertices with insufficient clearance.
      const double regionSize = GetRegionRadius(iter->property());
      if(regionSize < 0) {
        if(this->m_debug)
          std::cout << "\t\tRegion size " << regionSize << " < 0 too small."
                    << std::endl;
        continue;
      }

      // Generate a valid configuration in the boundary centered at the skeleton.
      const CSpaceBoundingSphere boundary = MakeBoundary(iter->property());
      const std::vector<CfgType> samples = Sample(&boundary);

      // Skip vertices where we fail to generate a sample.
      if(samples.empty()) {
        if(this->m_debug)
          std::cout << "\t\tCould not find a valid sample at this vertex."
                    << std::endl;
        continue;
      }

      // Add the samples to the roadmap and initialize each as its own
      // component.
      std::vector<VID> vids;
      std::vector<VertexSet> components;
      std::unordered_map<VID, size_t> componentIndex;
      for(const CfgType& cfg : samples) {
        const VID vid = r->AddVertex(cfg);
        vids.push_back(vid);
        components.push_back({vid});
        componentIndex[vid] = components.size() - 1;
      }

      // Attempt to connect the samples to form components.
      for(size_t i = 0; i < vids.size(); ++i) {
        for(size_t j = i + 1; j < vids.size(); ++j) {
          // Try to connect sample i and j.
          const VID v1 = vids[i],
                    v2 = vids[j];
          const bool connected = AttemptConnection(r->GetVertex(v1),
              r->GetVertex(v2), lpOutput);

          // Skip failures.
          if(!connected)
            continue;

          // Add the edges to the roadmap.
          r->AddEdge(v1, v2, lpOutput.m_edge);

          // If the vertices are already in the same component, we're done.
          const size_t index1 = componentIndex[v1],
                       index2 = componentIndex[v2];
          if(index1 == index2)
            continue;

          // Merge v2's component into v1's.
          components[index1].insert(components[index2].begin(),
              components[index2].end());

          // Retarget all vertices in the index2 component on index1.
          for(const VID vid : components[index2])
            componentIndex[vid] = index1;

          // Clear the index2 component but DONT REMOVE IT because that will
          // screw up the indexes.
          components[index2].clear();
        }
      }

      if(this->m_debug)
        std::cout << "\tComponents: " << components
                  << std::endl;

      // Each non-empty component will now be a local component rooted at this
      // skeleton vertex.
      size_t newComponents = 0;
      for(const VertexSet& component : components) {
        // Skip empty components.
        if(component.empty())
          continue;
        ++newComponents;

        for(auto eit = iter->begin(); eit != iter->end(); ++eit) {
          const VID representative = *component.begin();
          LocalComponentDescriptor d{eit->descriptor(), representative, false};
          MakeLocalComponent(d, component);
        }
      }

      componentCount += newComponents;
      regionCount += newComponents * iter->size();
      if(this->m_debug)
        std::cout << "\t\tInitialized " << newComponents << " new components "
                  << "and " << newComponents * iter->size() << " regions "
                  << "at skeleton vertex " << iter->descriptor() << "."
                  << std::endl;
    }

    if(this->m_debug)
      std::cout << "Initialized " << componentCount << " local components "
                << "and " << regionCount << " expansion regions."
                << std::endl;
  }

  // If we have a start and goal, add them to the roadmap.
  AddQuery();
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
Iterate() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Iterate");

  // Select a region for sample generation.
  auto region = SelectExpansionRegion();

  // Check for selecting the whole environment. In that case, we'll do something
  // different.
  const bool wholeEnvironment = region == nullptr;
  InitializeLUnconnectedEdgeMap();
  if(wholeEnvironment) {
    if(m_unconnectedEdges.at(this->GetTask()->GetRobot()).empty())
      return;
    // Select a random unconnected edge and attempt to connect it.
    const size_t index = LRand() % m_unconnectedEdges.at(this->GetTask()->GetRobot()).size();
    auto iter = m_unconnectedEdges.at(this->GetTask()->GetRobot()).begin();
    std::advance(iter, index);
    ConnectEdgeSegment(*iter);
    return;
  }

  // Try to expand the component associated with this region. If we succeed,
  // advance the region until it no longer contains the new samples.
  const SkeletonEdgeDescriptor ed = (*region->edgeIterator).descriptor();
  const VertexSet samples = ExpandComponent(region);
  if(!samples.empty())
    while(AdvanceRegion(region, samples) and AreSamplesCovered(region, samples))
      continue;

  // Try to connect the selected component to the others if needed.
  if(!IsEdgeConnected(ed))
    ConnectEdgeSegment(ed);
}

/*-------------------- Roadmap builders -------------------------------------*/

template<typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
AddQuery() {
  // Do nothing if we have no query.
  if(this->GetTask()->Empty())
    return;

  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::AddQuery");

  std::vector<VID> queryPoints;
  const VID start = this->GenerateStart(m_samplers.front().label);
  const std::vector<VID> goals = this->GenerateGoals(m_samplers.front().label);
  queryPoints.push_back(start);
  queryPoints.insert(queryPoints.end(), goals.begin(), goals.end());

  auto r = this->GetRoadmap();

  // Try to connect each query point to the full roadmap.
  for(const VID vid : queryPoints) {
    // Get the nearest neighbors.
    const CfgType& cfg = r->GetVertex(vid);
    const std::vector<Neighbor> neighbors = FindNearestNeighbors(cfg, nullptr);

    // Try to connect to each.
    bool connected = false;
    LPOutput<MPTraits> lp;
    for(const Neighbor& n : neighbors) {
      // Try connection.
      const bool success = AttemptConnection(cfg, r->GetVertex(n.target), lp);

      // Skip failures.
      if(!success)
        continue;
      connected = true;
      if(this->m_debug)
        std::cout << "\tConnected to " << n.target << "."
                  << std::endl;

      // Add the edges to the roadmap.
      r->AddEdge(vid, n.target, lp.m_edge);

      // Find n.target's local component and add vid to it.
      const LocalComponentDescriptor d = FindLocalComponent(n.target);
      VertexSet& vids = d.bridge
                      ? GetBridge(d)
                      : GetLocalComponent(d);
      vids.insert(vid);

      if(this->m_debug)
        std::cout << "\t\tMerged with local component " << d << "."
                  << std::endl;
      break;
    }

    // If we connected, all is well. Move on to the next one.
    if(connected)
      continue;

    // Rut roh. Check if this query is within the boundary of any skeleton edge.
    // If so, start a new local component for it.
    const bool inRegion = CheckReachedRegion(vid, {vid});
    if(inRegion)
      continue;

    // Double rut roh. RRT outta here until we hit the nearest skeleton vertex
    // or connect.
    GrowRRT(vid);
  }
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
CheckReachedRegion(const VID _query, const VertexSet& _component) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::CheckReachedRegion");
  auto r = this->GetRoadmap();
  const CfgType& cfg = r->GetVertex(_query);
  auto& skeletonGraph = m_skeleton;

  for(auto vi = skeletonGraph.begin(); vi != skeletonGraph.end(); ++vi) {
    // Check if _query is in this region.
    const CSpaceBoundingSphere boundary = MakeBoundary(vi->property());
    const bool inRegion = boundary.InBoundary(cfg);

    // Skip failures.
    if(!inRegion)
      continue;

    // Initalize local components on the edges.
    for(auto ei = vi->begin(); ei != vi->end(); ++ei)
      MakeLocalComponent({ei->descriptor(), _query, false}, _component);
    return true;
  }
  return false;
}


template<typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
GrowRRT(const VID _q) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::GrowRRT");

  if(this->m_debug)
    std::cout << "Growing RRT from node " << _q << " until connection or "
              << "locating a skeleton vertex."
              << std::endl;

  auto r = this->GetRoadmap();
  auto ccTracker = r->GetCCTracker();

  std::vector<CfgType> samples;
  LPOutput<MPTraits> lp;

  // Get the set of roadmap VIDs before growing the RRT and not counting _q.
  VertexSet roadmapVIDs;
  for(auto vi = r->begin(); vi != r->end(); ++vi)
    roadmapVIDs.insert(vi->descriptor());
  roadmapVIDs.erase(_q);

  while(true) {
    // Load up more samples if needed.
    while(samples.empty())
      samples = Sample();

    if(this->m_debug)
      std::cout << "\tPopping next sample." << std::endl;

    // Pop the next sample.
    CfgType qRand = samples.back();
    samples.pop_back();

    // Find nearest neighbors.
    const std::vector<Neighbor> treeNeighbors = FindNearestNeighbors(qRand,
        ccTracker->GetCC(_q));
    if(treeNeighbors.empty())
      continue;

    // Extend from the nearest neighbor to qRand.
    const VID newVID = Extend(treeNeighbors[0].target, qRand, lp);
    if(newVID == INVALID_VID)
      continue;

    if(this->m_debug)
      std::cout << "\tAttempting to connect " << newVID << " to pre-RRT roadmap."
                << std::endl;

    // Try to connect newVID to the pre-RRT roadmap.
    const std::vector<Neighbor> mapNeighbors = FindNearestNeighbors(
        r->GetVertex(newVID), &roadmapVIDs);
    for(const Neighbor& n : mapNeighbors) {
      // Try connection.
      const bool success = AttemptConnection(r->GetVertex(newVID),
          r->GetVertex(n.target), lp);
      // Skip failures.
      if(!success)
        continue;

      // Add edge.
      r->AddEdge(newVID, n.target, lp.m_edge);

      // Figure out what local component we connected to and add this tree
      // to it.
      const LocalComponentDescriptor d = FindLocalComponent(n.target);
      const VertexSet* const treeCC = ccTracker->GetCC(newVID);
      VertexSet& vids = d.bridge
                      ? GetBridge(d)
                      : GetLocalComponent(d);
      vids.insert(treeCC->begin(), treeCC->end());

      if(this->m_debug)
        std::cout << "\tEnding RRT because we connected to the roadmap at "
                  << n.target << "."
                  << "\n\tMerged with local component " << d << "."
                  << std::endl;

      // We're done with the RRT now.
      return;
    }

    // Check if we reached skeleton node. If so, make this a new local component
    // for all of its outgoing edges.
    const bool inRegion = CheckReachedRegion(newVID, *ccTracker->GetCC(newVID));
    if(inRegion) {
      if(this->m_debug)
        std::cout << "\tEnding RRT because we reached a skeleton vertex region."
                  << std::endl;
      return;
    }
  }

  throw RunTimeException(WHERE) << "Unreachable state.";
}

/*------------------------- Connected Component Methods -------------------*/

template <typename MPTraits>
typename MPTraits::RoadmapType::VertexSet
DynamicRegionsPRM<MPTraits>::
ExpandComponent(ExpansionRegion* const _r) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::ExpandComponent");

  if(this->m_debug)
    std::cout << "Attempting to expand component "
              << _r->GetLocalComponentDescriptor()
              << " at index " << _r->edgeIndex
              << " with center at (" << _r->GetCenter()
              << ")."
              << "\n\tRegion success rate is " << _r->GetWeight() << "."
              << std::endl;

  // Ensure we got a valid expansion region.
  if(!_r)
    throw RunTimeException(WHERE) << "Non-null region expected.";

  // Generate samples in the region.
  const auto boundary = MakeBoundary(_r->GetCenter());
  const std::vector<CfgType> samples = Sample(&boundary);

  auto r = this->GetRoadmap();
  const size_t initialEdgeCount = r->get_num_edges();

  // Try to connect each sample to the component.
  const LocalComponentDescriptor d = _r->GetLocalComponentDescriptor();
  VertexSet newVIDs;
  for(const CfgType& cfg : samples) {
    // Make edges.
    const std::vector<EdgeOutput> edges = ConnectToComponent(cfg, d);

    // Check for success.
    const bool success = !edges.empty();
    if(!success)
      continue;

    // Update the roadmap.
    const VID newVID = r->AddVertex(cfg);
    newVIDs.insert(newVID);
    for(const auto& edge : edges)
      r->AddEdge(newVID, edge.target, edge.weights);
  }

  // Update the success rate for this region.
  _r->TrackSuccess(newVIDs.size(), samples.size());

  // Add the new vertices to their local component.
  VertexSet& cc = GetLocalComponent(d);
  cc.insert(newVIDs.begin(), newVIDs.end());

  if(this->m_debug)
    std::cout << "\tGenerated " << newVIDs.size() << " new vertices "
              << newVIDs << "."
              << "\n\tGenerated " << (r->get_num_edges() - initialEdgeCount) / 2.
              << " edges."
              << "\n\tRegion success rate is now " << _r->GetWeight() << "."
              << "\n\tComponent VIDs: " << GetLocalComponent(d)
              << std::endl;

  // Aggressive bridging.
  if(m_aggressiveBridging) {
    // If there are any local components going the other way, try to connect.
    bool bridged = false;
    const SkeletonEdgeDescriptor reverseEd = reverse((*_r->edgeIterator).descriptor());
    const auto& reverseComponents = m_localComponents[this->GetTask()->GetRobot()][reverseEd];
    if(reverseComponents.size()) {
      if(this->m_debug)
        std::cout << "\tChecking for connection to " << reverseComponents.size()
                  << " components from the other side of this edge."
                  << std::endl;

      // Copy the descriptors because any merges will invalidate iteration over
      // reverseComponents.
      std::vector<LocalComponentDescriptor> reverseDescriptors;
      for(const auto& repAndVIDs : reverseComponents)
        reverseDescriptors.emplace_back(LocalComponentDescriptor{reverseEd, repAndVIDs.first, false});

      LocalComponentDescriptor myComponent = d;
      for(const auto& rd : reverseDescriptors) {
        if(this->m_debug)
          std::cout << "\tChecking connection to " << rd << "."
                    << std::endl;

        for(const VID newVID : newVIDs) {
          if(this->m_debug)
            std::cout << "\tChecking connection from VID " << newVID << "."
                      << std::endl;

          // Make edges.
          const std::vector<EdgeOutput> edges = this->ConnectToComponent(
              r->GetVertex(newVID), rd);
          if(!edges.size())
            continue;

          // We made a bridge.
          bridged = true;
          for(auto& edge : edges)
            r->AddEdge(newVID, edge.target, std::move(edge.weights));

          myComponent = MergeLocalComponents(myComponent, rd);

          if(this->m_debug)
            std::cout << "\t\tMerged with component " << rd << " to form a bridge."
                      << "\n\t\tNew component descriptor is " << myComponent << "."
                      << std::endl;

          // Once we've connected, we don't need to keep trying the other vids.
          break;
        }
      }
    }

    if(bridged)
      newVIDs.clear();
  }
  return newVIDs;
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
ConnectEdgeSegment(const SkeletonEdgeDescriptor _ed) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::ConnectEdgeSegment");
  auto r = this->GetRoadmap();

  if(this->m_debug) {
    std::cout << "Trying to connect components in edge (" << _ed.id() << "|"
              << _ed.source() << "," << _ed.target() << ").";

    std::cout << "\n\tLeft components:";
    const auto& left = m_localComponents[this->GetTask()->GetRobot()][_ed];
    for(const auto& repAndVIDs : left)
      std::cout << "\n\t\t"
                << LocalComponentDescriptor{_ed, repAndVIDs.first, false}
                << " " << repAndVIDs.second;

    std::cout << "\n\tRight components:";
    const SkeletonEdgeDescriptor rd = reverse(_ed);
    const auto& right = m_localComponents[this->GetTask()->GetRobot()][rd];
    for(const auto& repAndVIDs : right)
      std::cout << "\n\t\t"
                << LocalComponentDescriptor{rd, repAndVIDs.first, false}
                << " " << repAndVIDs.second;

    std::cout << "\n\tBridge components:";
    const auto& bridge = m_bridges[this->GetTask()->GetRobot()][_ed];
    for(const auto& repAndVIDs : bridge)
      std::cout << "\n\t\t"
                << LocalComponentDescriptor{_ed, repAndVIDs.first, true}
                << " " << repAndVIDs.second;

    std::cout << std::endl;
  }

  using CCMap   = std::map<VID, VertexSet>;
  using EdgeMap = std::unordered_map<VID, std::vector<EdgeOutput>>;
  EdgeMap leftEdges, rightEdges, bridgeEdges;

  // Define a function for building connections to local components in a CCMap.
  auto connector = [this](const CfgType& _cfg,
      const SkeletonEdgeDescriptor& _ed,
      EdgeMap& _edgeMap, const bool _bridges) {
    // Find the cc map.
    const auto& ccMap = _bridges ? m_bridges.at(this->GetTask()->GetRobot()).at(_ed)
                                 : m_localComponents[this->GetTask()->GetRobot()].at(_ed);

    // Try to make connection from _cfg to each component in the map.
    for(const auto& repAndVIDs : ccMap) {
      const VID representative = repAndVIDs.first;
      const LocalComponentDescriptor d{_ed, representative, _bridges};

      // Make edges.
      const std::vector<EdgeOutput> edges = this->ConnectToComponent(_cfg, d);
      if(!edges.size())
        continue;

      auto& mappedEdges = _edgeMap[representative];
      mappedEdges.insert(mappedEdges.end(), edges.begin(), edges.end());
    }
  };

  // Define a function for adding new connections to the roadmap and returning
  // the affected component descriptors.
  auto addConnections = [this, r](
      const VID _newVID,
      EdgeMap& _edges,
      std::vector<LocalComponentDescriptor>& _descriptors,
      const SkeletonEdgeDescriptor _ed,
      const bool _bridge)
  {
    for(auto& representativeAndEdges : _edges) {
      // Add this local component to the descriptors.
      const VID representative = representativeAndEdges.first;
      _descriptors.push_back({_ed, representative, _bridge});

      // Add the new edges to the roadmap.
      std::vector<EdgeOutput>& edges = representativeAndEdges.second;
      for(auto& edge : edges)
        r->AddEdge(_newVID, edge.target, std::move(edge.weights));
    }
  };

  // Pick a random point along the edge.
  const std::vector<Point3d>& path = m_skeleton.FindEdge(_ed)->property();
  if(path.empty())
    throw RunTimeException(WHERE) << "Ill-formed skeleton. Edge descriptor ("
                                  << _ed.id() << "|" << _ed.source() << ","
                                  << _ed.target() << ") has empty path.";

  const size_t index = LRand() % path.size();
  const Point3d& point = path[index];

  // Make a sampling region at the point and draw samples.
  const CSpaceBoundingSphere boundary = MakeBoundary(point);
  const std::vector<CfgType> samples = Sample(&boundary);

  // Try to connect each sample to the local components in this edge.
  for(const CfgType& cfg : samples) {
    const SkeletonEdgeDescriptor rightEd = reverse(_ed);
    // Try to connect to each set of components.
    if(this->m_debug)
      std::cout << "\tCfg: " << cfg.PrettyPrint()
                << "\n\tTrying to connect left components."
                << std::endl;
    leftEdges.clear();
    connector(cfg, _ed, leftEdges, false);
    if(this->m_debug)
      std::cout << "\tTrying to connect right components." << std::endl;
    rightEdges.clear();
    connector(cfg, rightEd, rightEdges, false);
    if(this->m_debug)
      std::cout << "\tTrying to connect bridges." << std::endl;
    bridgeEdges.clear();
    connector(cfg, _ed, bridgeEdges, true);

    if(this->m_debug)
      std::cout << "\tConnected to:"
                << "\n\t\t" << leftEdges.size() << "/"
                << m_localComponents[this->GetTask()->GetRobot()][_ed].size() << " left components"
                << "\n\t\t" << rightEdges.size() << "/"
                << m_localComponents[this->GetTask()->GetRobot()][rightEd].size() << " right components"
                << "\n\t\t" << bridgeEdges.size() << "/"
                << m_bridges[this->GetTask()->GetRobot()][_ed].size() << " bridges"
                << std::endl;

    // Determine what kind of connection(s) we built to decide whether to keep
    // them.

    // We merged bridges if there are only edges to two or more bridges.
    const bool mergedBridges = bridgeEdges.size() > 1;
    // If we connected to a bridge and anything else, we extended the bridge.
    const bool extendedBridge = bridgeEdges.size()
                            and (leftEdges.size() or rightEdges.size());
    // We built a new bridge if the two sides connected.
    const bool newBridge = leftEdges.size() and rightEdges.size();
    // We merged left- or right- components if we produced edges to more than
    // one on the same side.
    const bool leftMerge  = leftEdges.size() > 1,
               rightMerge = rightEdges.size() > 1;

    // If none of the above happened, we didn't complete any merges. Discard the
    // edges.
    if(!(mergedBridges or extendedBridge or newBridge or leftMerge or rightMerge)) {
      if(this->m_debug)
        std::cout << "\tNo merges occurred." << std::endl;
      continue;
    }

    // We made a merge. Add the new components to the roadmap and collect the
    // component descriptors.
    const VID newVID = r->AddVertex(cfg);
    std::vector<LocalComponentDescriptor> descriptors;
    addConnections(newVID, leftEdges, descriptors, _ed, false);
    addConnections(newVID, rightEdges, descriptors, rightEd, false);
    addConnections(newVID, bridgeEdges, descriptors, _ed, true);

    // We had better have more than one descriptor at this point.
    if(descriptors.size() < 2)
      throw RunTimeException(WHERE) << "We merged components but got < 2 "
                                    << "descriptors.";

    if(this->m_debug)
      std::cout << "\tOne or more merges occured ("
                << (mergedBridges ? "merged bridges," : "")
                << (extendedBridge ? "extended bridges," : "")
                << (newBridge ? "new bridge," : "")
                << (leftMerge ? "left merge," : "")
                << (rightMerge ? "right merge," : "")
                << ")."
                << std::endl;

    // Merge each of the descriptors together. MergeLocalComponents will handle
    // the remaining details.
    LocalComponentDescriptor d = descriptors[0];
    for(size_t i = 1; i < descriptors.size(); ++i)
      d = MergeLocalComponents(d, descriptors[i]);

    // Add the new VID to the surviving local component.
    VertexSet& vids = d.bridge ? GetBridge(d) : GetLocalComponent(d);
    vids.insert(newVID);
  }

  UpdateEdgeConnectivity(_ed);
}

/*------------------------ Local Connected Components ------------------------*/

template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
MakeLocalComponent(const LocalComponentDescriptor& _d, const VertexSet& _vids) {
  if(this->m_debug)
    std::cout << "\t\tInitializing new component with descriptor " << _d << "."
              << "\n\t\t\tVIDs: " << _vids
              << std::endl;

  // Make sure this component isn't already a bridge.
  const bool isBridge = m_bridges[this->GetTask()->GetRobot()][_d.edgeDescriptor].count(
      _d.representative);
  if(isBridge)
    throw RunTimeException(WHERE) << "Local component " << _d
                                  << " is an existing bridge.";

  // Try to construct a new component.
  {
    auto iterBool = m_localComponents[this->GetTask()->GetRobot()][_d.edgeDescriptor].emplace(
        _d.representative, _vids);

    // Make sure we constructed a new element.
    const bool isNew = iterBool.second;
    if(!isNew)
      throw RunTimeException(WHERE) << "Local component " << _d
                                    << " already exists.";
  }

  // Try to construct an expansion region for this component.
  {
    SkeletonEdgeIterator iter = m_skeleton.FindEdge(_d.edgeDescriptor);
    auto iterBool = m_regions[this->GetTask()->GetRobot()][_d.edgeDescriptor].emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_d.representative),
        std::forward_as_tuple(iter, _d.representative));

    // Make sure we constructed a new element.
    const bool isNew = iterBool.second;
    if(!isNew)
      throw RunTimeException(WHERE) << "Region for component " << _d
                                    << " already exists.";
  }

  // Update this edge's connectivity.
  UpdateEdgeConnectivity(_d.edgeDescriptor);
}


template <typename MPTraits>
typename MPTraits::RoadmapType::VertexSet&
DynamicRegionsPRM<MPTraits>::
GetLocalComponent(const LocalComponentDescriptor& _d) noexcept {
  try {
    return m_localComponents.at(this->GetTask()->GetRobot()).at(_d.edgeDescriptor).at(_d.representative);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Local component " << _d
                                  << " does not exist.";
  }
}


template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::ExpansionRegion*
DynamicRegionsPRM<MPTraits>::
GetExpansionRegion(const LocalComponentDescriptor& _d) noexcept {
  // If this edge has low clearance, we may not have a region for this
  // component.
  InitializeLowClearanceMap();
  const bool lowClearance = m_lowClearanceMap.at(this->GetTask()->GetRobot()).count(_d.edgeDescriptor.id());
  if(lowClearance and !m_regions.at(this->GetTask()->GetRobot()).at(_d.edgeDescriptor).count(_d.representative))
    return nullptr;

  try {
    return &m_regions.at(this->GetTask()->GetRobot()).at(_d.edgeDescriptor).at(_d.representative);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Expansion region for component " << _d
                                  << " does not exist.";
  }
}


template <typename MPTraits>
typename MPTraits::RoadmapType::VertexSet&
DynamicRegionsPRM<MPTraits>::
GetBridge(const LocalComponentDescriptor& _d) noexcept {
  try {
    return m_bridges.at(this->GetTask()->GetRobot()).at(_d.edgeDescriptor).at(_d.representative);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Bridge " << _d << " does not exist.";
  }
}


template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::LocalComponentDescriptor
DynamicRegionsPRM<MPTraits>::
FindLocalComponent(const VID _vid) const noexcept {
  // Search the local components.
  for(const auto& edAndMap : m_localComponents.at(this->GetTask()->GetRobot())) {
    for(const auto& vidAndVIDs : edAndMap.second) {
      // Check for the vid in this set.
      const VertexSet& vids = vidAndVIDs.second;
      if(!vids.count(_vid))
        continue;

      // This is the component we joined.
      return LocalComponentDescriptor{edAndMap.first, vidAndVIDs.first, false};
    }
  }

  // Search the bridges.
  for(const auto& edAndMap : m_bridges.at(this->GetTask()->GetRobot())) {
    for(const auto& vidAndVIDs : edAndMap.second) {
      // Check for the vid in this set.
      const VertexSet& vids = vidAndVIDs.second;
      if(!vids.count(_vid))
        continue;

      // This is the component we joined.
      return LocalComponentDescriptor{edAndMap.first, vidAndVIDs.first, true};
    }
  }

  throw RunTimeException(WHERE) << "VID " << _vid
                                << " is not in any local component or bridge.";
}


template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::LocalComponentDescriptor
DynamicRegionsPRM<MPTraits>::
PromoteLocalComponent(LocalComponentDescriptor _d) {
  if(this->m_debug)
    std::cout << "\t\tPromoted local component " << _d << " to bridge."
              << std::endl;

  // Get the vids, which also ensures the component exists.
  VertexSet& vids = GetLocalComponent(_d);

  // Try to construct a new bridge.
  auto iterBool = m_bridges[this->GetTask()->GetRobot()][_d.edgeDescriptor].emplace(
      std::piecewise_construct,
      std::forward_as_tuple(_d.representative),
      std::forward_as_tuple(std::move(vids)));
  _d.bridge = true;

  // Make sure we created a new element.
  const bool isNew = iterBool.second;
  if(!isNew)
    throw RunTimeException(WHERE) << "Bridge " << _d << " already exists.";

  // Erase from the local component and region maps.
  m_localComponents[this->GetTask()->GetRobot()][_d.edgeDescriptor].erase(_d.representative);
  m_regions[this->GetTask()->GetRobot()][_d.edgeDescriptor].erase(_d.representative);

  return _d;
}


template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::LocalComponentDescriptor
DynamicRegionsPRM<MPTraits>::
MergeLocalComponents(const LocalComponentDescriptor& _d1,
    const LocalComponentDescriptor& _d2) {
  // Assert that the edge descriptor IDs match, or we're looking at local ccs
  // covering different edges.
  const bool edgeIdsOk = _d1.edgeDescriptor.id() == _d2.edgeDescriptor.id();
  if(!edgeIdsOk)
    throw RunTimeException(WHERE) << "Descriptors point to different edge IDs "
                                  << _d1 << ", " << _d2 << ".";

  // Define a function for merging bridges.
  auto mergeBridges = [this](const LocalComponentDescriptor& _d1,
                             const LocalComponentDescriptor& _d2) {
    // Get both vertex sets.
    VertexSet& v1 = this->GetBridge(_d1),
             & v2 = this->GetBridge(_d2);

    // Merge the smaller bridge into the larger one.
    if(v1.size() > v2.size()) {
      v1.insert(v2.begin(), v2.end());
      this->m_bridges[this->GetTask()->GetRobot()][_d2.edgeDescriptor].erase(_d2.representative);
      return _d1;
    }
    else {
      v2.insert(v1.begin(), v1.end());
      this->m_bridges[this->GetTask()->GetRobot()][_d1.edgeDescriptor].erase(_d1.representative);
      return _d2;
    }
  };

  // Define a function for same-side merges.
  auto sameSideMerge = [this](const LocalComponentDescriptor& _save,
                              const LocalComponentDescriptor& _merge) {
    // Get the vertex sets.
    VertexSet& save  = this->GetLocalComponent(_save),
             & merge = this->GetLocalComponent(_merge);

    // Merge the smaller set into the larger, move result to save if necessary.
    if(save.size() >= merge.size())
      save.insert(merge.begin(), merge.end());
    else {
      merge.insert(save.begin(), save.end());
      save = std::move(merge);
    }

    this->m_localComponents[this->GetTask()->GetRobot()][_merge.edgeDescriptor].erase(_merge.representative);
    this->m_regions[this->GetTask()->GetRobot()][_merge.edgeDescriptor].erase(_merge.representative);
    return _save;
  };

  // Check if we are merging any bridges.
  const size_t numBridges = _d1.bridge + _d2.bridge;
  switch(numBridges) {
    // If both are bridges, merge into the larger one.
    case 2:
    {
      if(this->m_debug)
        std::cout << "\t\tMerging bridges " << _d1 << " and " << _d2 << "."
                  << std::endl;
      return mergeBridges(_d1, _d2);
    }
    // If one is a bridge, promote the other and then merge bridges.
    case 1:
    {
      if(this->m_debug)
        std::cout << "\t\tMerging bridge " << (_d1.bridge ? _d1 : _d2)
                  << " and component " << (_d1.bridge ? _d2 : _d1) << "."
                  << std::endl;
      return _d1.bridge ? mergeBridges(_d1, PromoteLocalComponent(_d2))
                        : mergeBridges(_d2, PromoteLocalComponent(_d1));
    }
    // Handle merging two non-bridges.
    case 0:
    {
      // If the components are from either side, we've made a bridge. Promote both
      // components and merge.
      const bool sameSide = _d1.edgeDescriptor == _d2.edgeDescriptor;
      if(!sameSide) {
        if(this->m_debug)
          std::cout << "\t\tMerging components " << _d1 << " and " << _d2
                    << " into a new bridge."
                    << std::endl;
        return mergeBridges(PromoteLocalComponent(_d1),
                            PromoteLocalComponent(_d2));
      }

      if(this->m_debug)
        std::cout << "\t\tMerging components " << _d1 << " and " << _d2
                  << " on the same side."
                  << std::endl;

      // The components are from the same side. Find which component's region is
      // further along and keep that one.
      const ExpansionRegion* r1 = GetExpansionRegion(_d1),
                           * r2 = GetExpansionRegion(_d2);
      // We may get null regions if the component is on a low-clearance edge. In
      // that case, we'll save the component with no region because the other
      // one will eventually hit the low-clearance point.
      const bool saveR1 = (r1 and r2 and r1->edgeIndex > r2->edgeIndex)
                       or (!r1 and r2);
      return saveR1 ? sameSideMerge(_d1, _d2)
                    : sameSideMerge(_d2, _d1);
    }
    // We really can't get here...
    default:
      throw RunTimeException(WHERE) << "Unreachable state.";
  }
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
UpdateEdgeConnectivity(const SkeletonEdgeDescriptor& _ed) {
  // Get the component information.
  const SkeletonEdgeDescriptor right = reverse(_ed);
  const auto& leftCCs   = m_localComponents[this->GetTask()->GetRobot()][_ed],
            & rightCCs  = m_localComponents[this->GetTask()->GetRobot()][right],
            & bridgeCCs = m_bridges[this->GetTask()->GetRobot()][_ed];

  bool connected;

  // If the edge is marked as low clearnce, it is connected if there is at most
  // one left and one right component.
  InitializeLowClearanceMap();
  const bool lowClearance = m_lowClearanceMap.at(this->GetTask()->GetRobot()).count(_ed.id());
  if(lowClearance)
     connected = leftCCs.size() < 2 and rightCCs.size() < 2;
  // Otherwise, it is connected if there is only one component.
  else
    connected = (leftCCs.size() + rightCCs.size() + bridgeCCs.size()) < 2;

  const SkeletonEdgeDescriptor& canonical = _ed.source() < _ed.target()
                                          ? _ed
                                          : right;
  
  InitializeLUnconnectedEdgeMap();
  if(connected)
    m_unconnectedEdges.at(this->GetTask()->GetRobot()).erase(canonical);
  else
    m_unconnectedEdges.at(this->GetTask()->GetRobot()).insert(canonical);
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
IsEdgeConnected(const SkeletonEdgeDescriptor& _ed) {
  const SkeletonEdgeDescriptor right = reverse(_ed);
  const SkeletonEdgeDescriptor& canonical = _ed.source() < _ed.target()
                                          ? _ed
                                          : right;
  InitializeLUnconnectedEdgeMap();
  return !m_unconnectedEdges.at(this->GetTask()->GetRobot()).count(canonical);
}

/*----------------------------- Region Functions -----------------------------*/

template<typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::ExpansionRegion*
DynamicRegionsPRM<MPTraits>::
SelectExpansionRegion() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::SelectExpansionRegion");

  // Update all region probabilities.
  auto probabilitiesAndRegions = ComputeProbabilities();
  const std::vector<double>& probabilities     = probabilitiesAndRegions.first;
  const std::vector<ExpansionRegion*>& regions = probabilitiesAndRegions.second;

  // Construct with random number generator with the region probabilities.
  static std::default_random_engine generator(0);
  std::discrete_distribution<size_t> distribution(probabilities.begin(),
      probabilities.end());

  const size_t index = distribution(generator);

  if(this->m_debug) {
    std::cout << "Computed region selection probabilities ("
              << "last is whole env):\n\t";

    for(auto p : distribution.probabilities())
      std::cout << std::setprecision(4) << p << " ";

    const ExpansionRegion* const r = regions.at(index);
    std::cout << "\n\tSelected index " << index
              << (r ? "." : " (whole env).")
              << std::endl;
  }

  return regions[index];
}


template <typename MPTraits>
std::pair<std::vector<double>,
          std::vector<typename DynamicRegionsPRM<MPTraits>::ExpansionRegion*>>
DynamicRegionsPRM<MPTraits>::
ComputeProbabilities() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::ComputeProbabilities");

  // Sum all weights of all current regions and collect them into a vector.
  double totalWeight = 0.;
  std::vector<ExpansionRegion*> regions;
  for(auto& edAndMap : m_regions.at(this->GetTask()->GetRobot())) {
    for(auto& representativeAndRegion : edAndMap.second) {
      ExpansionRegion* const region = &representativeAndRegion.second;
      totalWeight += region->GetWeight();
      regions.push_back(region);
    }
  }

  // Compute the probabilities for the current regions.
  std::vector<double> probabilities;
  probabilities.reserve(regions.size() + 1);

  // Compute the explore probability (added to all regions).
  const double explore = m_explore / (regions.size() + 1);

  // Compute the total probability (exploit + explore).
  for(const ExpansionRegion* const region : regions) {
    const double exploit = totalWeight > 100 * std::numeric_limits<double>::epsilon()
                         ? (1 - m_explore) * region->GetWeight() / totalWeight
                         : 0;

    probabilities.emplace_back(exploit + explore);
  }

  // Add a null pointer and probability for the full environment.
  regions.push_back(nullptr);
  probabilities.emplace_back(explore);

  return {probabilities, regions};
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
AdvanceRegion(ExpansionRegion* const _r, const VertexSet& _newVIDs) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::AdvanceRegion");

  const LocalComponentDescriptor d = _r->GetLocalComponentDescriptor();

  if(this->m_debug)
    std::cout << "Advancing region " << _r << " for local component " << d << "."
              << std::endl;

  // Ensure this is an advancable region.
  if(_r == nullptr)
    throw RunTimeException(WHERE) << "Cannot advance whole environment.";

  // Check if the region has reached the end of its edge.
  if(!_r->LastPoint()) {
    // Advance the region to its next position.
    _r->Advance();
    if(this->m_debug)
      std::cout << "\tAdvanced to position " << _r->edgeIndex + 1 << "/"
                << (*_r->edgeIterator).property().size()
                << "."
                << std::endl;

    // Check for usable clearance.
    const double clearance = GetRegionRadius(_r->GetCenter());
    if(clearance >= 0)
      return true;

    if(this->m_debug)
      std::cout << "\tClearance " << clearance << " < 0 too low, deleting."
                << std::endl;

    // Remove region from the map. Keep the local component as we will still
    // want to connect it to others from the same skeleton vertex.
    m_regions[this->GetTask()->GetRobot()][d.edgeDescriptor].erase(d.representative);

    InitializeLowClearanceMap();
    m_lowClearanceMap[this->GetTask()->GetRobot()].insert(d.edgeDescriptor.id());

    return false;
  }

  if(this->m_debug)
    std::cout << "\tRegion reached the end of its edge."
              << "\n\tTrying to connect new VIDs to outbound local components."
              << "\n\t\tVIDs: " << _newVIDs
              << std::endl;

  // Define a function for trying to connect this region's local component to
  // those rooted at the edge target.
  VertexSet connectedVIDs;
  auto connector = [this, &_newVIDs, &connectedVIDs](
      const SkeletonEdgeDescriptor _ed, const std::map<VID, VertexSet>& _ccs,
      const bool _bridges) {
    if(this->m_debug)
      std::cout << "\tTrying " << _ccs.size()
                << (_bridges ? " bridges " : " components ")
                << "on edge (" << _ed.id() << "|" << _ed.source() << ","
                << _ed.target() << ")..."
                << std::endl;

    auto r = this->GetRoadmap();

    // Check each CC for connections.
    bool connected = false;
    for(const auto& cc : _ccs) {
      const VID representative = cc.first;
      const LocalComponentDescriptor d{_ed, representative, _bridges};
      const VertexSet& vids = cc.second;

      // If we've already connected to any vertex in this CC, we don't need to
      // make more.
      if(HaveCommonVertex(connectedVIDs, vids)) {
        if(this->m_debug)
          std::cout << "\t\tSkipping component " << d << ", already connected."
                    << std::endl;
        connected = true;
        continue;
      }

      // Check each of our new vids.
      for(const VID vid : _newVIDs) {
        // Make edges to the target component.
        const std::vector<EdgeOutput> edges = ConnectToComponent(
            r->GetVertex(vid), d);

        // If we made no edges, move on to the next VID.
        if(edges.empty())
          continue;

        // Add the connections to the roadmap and move on to the next CC.
        connected = true;
        for(const auto& edge : edges) {
          r->AddEdge(vid, edge.target, edge.weights);
          connectedVIDs.insert(edge.target);
        }

        if(this->m_debug)
          std::cout << "\t\t"
                    << (edges.size() ? "Formed connections on"
                                     : "Failed to connect to")
                    << " component " << d << "."
                    << std::endl;
        break;
      }
    }

    return connected;
  };

  // Try to connect to any local components rooted at this vertex.
  bool connected = false;
  const SkeletonVertexIterator vit = m_skeleton.find_vertex(
      (*_r->edgeIterator).target());
  for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
    const SkeletonEdgeDescriptor ed = eit->descriptor();
    const auto& components = m_localComponents[this->GetTask()->GetRobot()][ed];
    const auto& bridges = m_bridges[this->GetTask()->GetRobot()][ed];

    connected |= connector(ed, components, false);
    connected |= connector(ed, bridges, true);
  }

  // If we didn't connect, initialize new regions on the outbound edges of the
  // target.
  if(!connected) {
    if(this->m_debug)
      std::cout << "\tRegion did not connect to any components at the target."
                << "\tSparking new regions on the outbound edges."
                << std::endl;
    const VID newRepresentative = *_newVIDs.begin();
    for(auto eit = vit->begin(); eit != vit->end(); ++eit)
      MakeLocalComponent({eit->descriptor(), newRepresentative, false}, _newVIDs);
  }

  // Promote component to bridge (also removes the region).
  PromoteLocalComponent(d);
  UpdateEdgeConnectivity(d.edgeDescriptor);

  return false;
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
AreSamplesCovered(const ExpansionRegion* const _region,
    const VertexSet& _samples) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::AreSamplesCovered");

  const CSpaceBoundingSphere boundary = MakeBoundary(_region->GetCenter());
  auto r = this->GetRoadmap();

  // The samples are considered covered if any of them is inside the boundary.
  const bool covered = std::any_of(_samples.begin(), _samples.end(),
      [&boundary,r](const VID _vid) {
        return boundary.InBoundary(r->GetVertex(_vid));
      }
  );

  if(this->m_debug)
    std::cout << "\t" << (covered ? "At least one sample is" : "No samples are")
              << " covered at the region's new position."
              << std::endl;

  return covered;
}

/*----------------------------- General Planning -----------------------------*/

template <typename MPTraits>
std::vector<typename MPTraits::CfgType>
DynamicRegionsPRM<MPTraits>::
Sample(const Boundary* _b) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::Sample");

  // If we received a null boundary, use the full environment.
  if(!_b)
    _b = this->GetEnvironment()->GetBoundary();

  // Generate nodes with each sampler.
  std::vector<CfgType> samples;
  for(const auto& sampler : m_samplers) {
    auto s = this->GetSampler(sampler.label);
    s->Sample(sampler.count, sampler.attempts, _b, std::back_inserter(samples));

    if(this->m_debug)
      std::cout << "\tSampler '" << sampler.label << "' generated "
                << samples.size() << "/" << sampler.count << " configurations."
                << std::endl;
  }

  return samples;
}


template <typename MPTraits>
std::vector<Neighbor>
DynamicRegionsPRM<MPTraits>::
FindNearestNeighbors(const CfgType& _cfg, const VertexSet* const _candidates) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::FindNearestNeighbors");

  if(this->m_debug)
    std::cout << "\tSearching for nearest neighbors to cfg" //<< _cfg.PrettyPrint()
              << " with '" << m_nfLabel << "' from "
              << (_candidates
                  ? "a set of size " + std::to_string(_candidates->size())
                  : "the full roadmap")
              << "."
              << std::endl;

  std::vector<Neighbor> neighbors;
  auto r = this->GetRoadmap();
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  if(_candidates)
    nf->FindNeighbors(r, _cfg, *_candidates, std::back_inserter(neighbors));
  else
    nf->FindNeighbors(r, _cfg, std::back_inserter(neighbors));

  if(neighbors.empty()) {
    stats->IncStat(this->GetNameAndLabel() + "::FailedNF");
    if(this->m_debug)
      std::cout << "\t\tFailed to find a nearest neighbor." << std::endl;
  }
  else if(this->m_debug)
    std::cout << "\t\tFound " << neighbors.size() << " nearest neighbors."
              << std::endl;

  return neighbors;
}


template <typename MPTraits>
bool
DynamicRegionsPRM<MPTraits>::
AttemptConnection(const CfgType& _c1, const CfgType& _c2,
    LPOutput<MPTraits>& _lpOutput) {
  auto lp = this->GetLocalPlanner(m_lpLabel);
  Environment* const env = this->GetEnvironment();

  _lpOutput.Clear();
  return lp->IsConnected(_c1, _c2, &_lpOutput,
      env->GetPositionRes(), env->GetOrientationRes(), true, false);
}


template <typename MPTraits>
std::vector<typename DynamicRegionsPRM<MPTraits>::EdgeOutput>
DynamicRegionsPRM<MPTraits>::
ConnectToComponent(const CfgType& _cfg,
    const LocalComponentDescriptor& _d) {
  auto stats = this->GetStatClass();
  const std::string clock = this->GetNameAndLabel() + "::ConnectToComponent";
  MethodTimer mt(stats, clock);

  const VertexSet& cc = _d.bridge ? GetBridge(_d) : GetLocalComponent(_d);

  if(this->m_debug)
    std::cout << "\tTrying to connect to component " << _d << "."
              << "\n\t\tVIDs: " << cc
              << std::endl;

  // Find neighbors for _cfg in the component _d.
  const std::vector<Neighbor> neighbors = FindNearestNeighbors(_cfg, &cc);

  // Try to connect _cfg to each neighbor.
  auto r = this->GetRoadmap();
  LPOutput<MPTraits> lp;
  std::vector<EdgeOutput> output;
  for(const auto& neighbor : neighbors) {
    // Try the connection.
    const VID vid = neighbor.target;
    const bool success = AttemptConnection(_cfg, r->GetVertex(vid), lp);

    if(this->m_debug)
      std::cout << "\t\t" << (success ? "Connected" : "Failed to connect")
                << " through VID " << vid << "."
                << std::endl;

    // Skip failures.
    if(!success)
      continue;

    // Add the edge to the output.
    output.emplace_back(EdgeOutput{INVALID_VID, vid, lp.m_edge});
  }

  return output;
}


template <typename MPTraits>
typename DynamicRegionsPRM<MPTraits>::VID
DynamicRegionsPRM<MPTraits>::
Extend(const VID _nearVID, const CfgType& _target, LPOutput<MPTraits>& _lp) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Extend");

  auto r = this->GetRoadmap();
  auto e = this->GetExtender(m_exLabel);
  const CfgType& qNear = r->GetVertex(_nearVID);
  CfgType qNew(qNear.GetRobot());

  _lp.Clear();
  const bool success = e->Extend(qNear, _target, qNew, _lp);
  if(this->m_debug)
    std::cout << "\tExtending from VID " << _nearVID
              << "\n\t\tExtended "
              << std::setprecision(4) << _lp.m_edge.first.GetWeight()
              << " units."
              << std::endl;

  if(!success) {
    // The extension failed to exceed the minimum distance.
    if(this->m_debug)
      std::cout << "\tNode too close, not adding." << std::endl;
    return INVALID_VID;
  }

  // The extension succeeded. Add the node and edges.
  const auto newVID = r->AddVertex(qNew);
  r->AddEdge(_nearVID, newVID, _lp.m_edge);

  if(this->m_debug)
    std::cout << "\tGenerated new vertex " << newVID << " and edge "
              << "(" << _nearVID << "," << newVID << ")."
              << std::endl;

  return newVID;
}

/*-------------------------- Workspace and Clearance -------------------------*/

template<typename MPTraits>
CSpaceBoundingSphere
DynamicRegionsPRM<MPTraits>::
MakeBoundary(const Vector3d& _v) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::MakeBoundary");

  const std::vector<double> v{_v[0], _v[1], _v[2]};
  // I'm not sure what the boundary code might do with a negative radius. Bound
  // it below at zero just in case.
  const double radius = std::max(0., GetRegionRadius(_v));

  const bool threeD = this->GetTask()->GetRobot()->GetMultiBody()->GetBaseType()
                   == Body::Type::Volumetric;
  if(threeD)
    return CSpaceBoundingSphere(v, radius);
  else
    return CSpaceBoundingSphere({_v[0], _v[1]}, radius);
}


template<typename MPTraits>
double
DynamicRegionsPRM<MPTraits>::
GetRegionRadius(const Vector3d& _v) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::GetRegionRadius");

  const double clearance = GetClearance(_v),
               robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->GetBody(0)->
                             GetPolyhedron().GetMinRadius() / 2;
  return std::max(1.2 * (clearance - robotRadius), m_minRegionRadius);
}


template<typename MPTraits>
double
DynamicRegionsPRM<MPTraits>::
GetClearance(const Vector3d& _v) {
  static std::map<Vector3d, double> m_clearanceMap;

  // Check for cached clearance value.
  auto iter = m_clearanceMap.find(_v);
  const bool cached = iter != m_clearanceMap.end();
  if(cached)
    return iter->second;

  // The value isn't cached. Compute it now.
  auto pointRobot = this->GetMPProblem()->GetRobot("point");
  auto vc = this->GetValidityChecker("pqp_solid");

  CfgType cfg(_v, pointRobot);
  CDInfo cd(true);
  const std::string callee = this->GetNameAndLabel() + "::GetClearance";
  vc->IsValid(cfg, cd, callee);

  const double clearance = cd.m_minDist;

  m_clearanceMap.emplace(_v, clearance);
  return clearance;
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
BuildSkeleton() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::BuildSkeleton");

  if(m_skeletonIO == "read") {
    m_skeleton.Read(m_skeletonFilename);
    m_skeleton.DoubleEdges();
    return;
  }

  // Determine if we need a 2d or 3d skeleton.
  auto env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  const bool threeD = robot->GetMultiBody()->GetBaseType()
                   == Body::Type::Volumetric;

  if(threeD) {
    if(m_skeletonType == "mcs") {
      if(this->m_debug)
        std::cout << "Building a Mean Curvature skeleton." << std::endl;
      MeanCurvatureSkeleton3D mcs;
      mcs.SetEnvironment(this->GetEnvironment());
      mcs.BuildSkeleton();

      // Create the workspace skeleton.
      auto sk = mcs.GetSkeleton();
      m_skeleton = sk.first;
      m_skeleton.DoubleEdges();
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
      m_skeleton = reeb.GetSkeleton();
      m_skeleton.DoubleEdges();
    }
    else
      throw ParseException(WHERE) << "Unrecognized skeleton type '"
                                  << m_skeletonType << "', options for 3d "
                                  << "problems are {mcs, reeb}.";
  }
  else {
    if(m_skeletonType == "ma") {
      // Collect the obstacles we want to consider (all in this case).
      std::vector<GMSPolyhedron> polyhedra;
      for(size_t i = 0; i < env->NumObstacles(); ++i) {
        MultiBody* const obstacle = env->GetObstacle(i);
        for(size_t j = 0; j < obstacle->GetNumBodies(); ++j)
          polyhedra.emplace_back(obstacle->GetBody(j)->GetWorldPolyhedron());
      }

      // Build a skeleton from a 2D medial axis.
      MedialAxis2D ma(polyhedra, env->GetBoundary());
      ma.BuildMedialAxis();
      m_skeleton = get<0>(ma.GetSkeleton(1)); // 1 for free space.
      m_skeleton.DoubleEdges();
    }
    else
      throw ParseException(WHERE) << "Unrecognized skeleton type '"
                                  << m_skeletonType << "', options for 2d "
                                  << "problems are {ma}.";
  }


  if(this->m_debug) {
    auto& g = m_skeleton;
    std::cout << "The skeleton has " << g.get_num_vertices() << " vertices and "
              << g.get_num_edges() << " edges."
              << std::endl;

    // Print the whole thing for inspection.
    for(auto vit = g.begin(); vit != g.end(); ++vit) {
      std::cout << "Vertex " << vit->descriptor() << " has " << vit->size()
                << " edges.\n\t";
      for(auto eit = vit->begin(); eit != vit->end(); ++eit)
        std::cout << " (" << eit->descriptor().id()
                  << "|" << eit->descriptor().source()
                  << "," << eit->descriptor().target()
                  << ")";
      std::cout << std::endl;
    }
  }

  if(m_skeletonIO == "write")
    m_skeleton.Write(m_skeletonFilename);
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
InitializeLowClearanceMap() {
  if(!m_lowClearanceMap.count(this->GetTask()->GetRobot()))
    m_lowClearanceMap.emplace(std::make_pair(this->GetTask()->GetRobot(), std::set<size_t>()));
}


template <typename MPTraits>
void
DynamicRegionsPRM<MPTraits>::
InitializeLUnconnectedEdgeMap() {
  if(!m_unconnectedEdges.count(this->GetTask()->GetRobot()))
    m_unconnectedEdges.emplace(std::make_pair(this->GetTask()->GetRobot(), std::set<SkeletonEdgeDescriptor, EdgeCompare>()));
}

/*----------------------------------------------------------------------------*/

#endif
