#ifndef DYNAMIC_REGION_RRT_H_
#define DYNAMIC_REGION_RRT_H_

#include <queue>
#include <unordered_map>

#include "BasicRRTStrategy.h"

#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"
#include "MPLibrary/MPTools/ReebGraphConstruction.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"
#include "Workspace/WorkspaceSkeleton.h"

#ifdef VIZMO
#include "GUI/ModelSelectionWidget.h"
#include "Models/TempObjsModel.h"
#include "Models/ThreadSafeSphereModel.h"
#include "Models/Vizmo.h"
#endif

auto to_point = [] (const vector<double>& _c) -> Point3d {
  if(_c.size() < 3)
    return Point3d();
  return Point3d(_c[0], _c[1], _c[2]);
};


////////////////////////////////////////////////////////////////////////////////
/// DynamicRegionRRT uses an embedded Reeb graph to guide dynamic sampling
/// regions through the environment.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DynamicRegionRRT : public BasicRRTStrategy<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    ///@}
    ///@name External Types
    ///@{

    typedef typename BasicRRTStrategy<MPTraits>::TreeType TreeType;
    typedef WorkspaceSkeleton::adj_edge_iterator          SkeletonEdgeIter;

    ///@}
    ///@name Construction
    ///@{

    DynamicRegionRRT(string _dm = "euclidean", string _nf = "Nearest",
        string _vc = "rapid", string _nc = "kClosest", string _ex = "BERO",
        vector<string> _evaluators = vector<string>(),
        string _gt = "UNDIRECTED_TREE",  bool _growGoals = false,
        double _growthFocus = .05, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3);

    DynamicRegionRRT(XMLNode& _node);

    virtual ~DynamicRegionRRT() = default;

    ///@}
    ///@name MPBaseObject overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Run() override;
    virtual void Finalize() override;

    ///@}

  protected:

    ///@name RRT Overrides
    ///@{

    /// Computes the growth direction for the RRT, choosing between the entire
    /// environment and each attract region with uniform probability to generate
    /// q_rand.
    /// @return The resulting growth direction.
    virtual CfgType SelectDirection() override;

    /// Find the nearest configuration to the target configuration (_cfg) in the
    /// tree (_tree).
    /// Utilize bucketing scheme to limit nearest neighbor candidates.
    /// @return The VID of the nearest neighbor.
    virtual VID FindNearestNeighbor(const CfgType& _cfg, const TreeType& _tree)
        override;

    virtual VID Extend(const VID _nearVID, const CfgType& _qrand, const bool _lp)
        override;

    ///@}
    ///@name Helpers
    ///@{

    /// Collect candidate neighbors for a given configuration.
    /// @param _cfg The configuration of interest.
    std::vector<VID> CollectCandidates(const CfgType& _cfg);

    /// Back-track along a skeleton edge and collect candidate nearest neighbors
    /// from the bucket map. If we reach the source of the edge before going the
    /// maximum distance, continue back-tracking along each edge inbound on that
    /// source vertex.
    /// @param _candidates Storage for the candidate set.
    /// @param _edge The skeleton edge to back-trace.
    /// @param _edgeIndex The index to start the back-trace from.
    /// @param _distance The distance we have already back-traced.
    /// @param _maxDistance The maximum distance to back-track.
    void BackTrackSkeletonEdge(vector<VID>& _candidates, SkeletonEdgeIter _edge,
        size_t _edgeIndex, double _distance, const double _maxDistance);

    /// Get a biasing direction for the next sample's velocity based on where
    /// the current region is headed next.
    /// @return A unit vector in the direction the region is moving.
    const Vector3d GetVelocityBias();

    /// Add a new vertex to the nearest visible bucket.
    /// @param _vid The new vertex to bucket.
    void AddToNearestVisibleBucket(const VID _vid);

    void FixSkeletonClearance();

    ///@}

  private:

    ///@}
    ///@name Internal State
    ///@{

    bool m_prune{true};          ///< Prune the flow graph?
    bool m_bucketing{false};        ///< Use bucketing?
    bool m_biasing{false};          ///< Use velocity biasing?

    double m_regionFactor{2.5};  ///< The region radius is this * robot radius.
    double m_robotFactor{1.};
        ///< The robot is touching a sampling region if inside by this amount.
    double m_velocityAlignment{.1};  ///< Strength of velocity biasing.
    double m_backtrackFactor{.5}; ///< The backtrack distance fraction.

    Boundary* m_samplingRegion{nullptr};         ///< The current sampling region.

    ReebGraphConstruction* m_reebGraph{nullptr}; ///< Embedded reeb graph
    WorkspaceSkeleton m_skeleton;                ///< Workspace skeleton

    /// Association between a flow graph vertex and a set of VIDs that were
    /// sampled the region centered on that vertex.
    /// Provides a bucketing system for samples along the flow graph. When
    /// finding the nearest neighbor this map is used to limit the search so the
    /// entire tree is not searched.
    map<Point3d, vector<VID>> m_buckets;

    ///@}

};

/*------------------------------ Construction --------------------------------*/


template <typename MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(string _dm, string _nf, string _vc, string _nc, string _ex,
    vector<string> _evaluators, string _gt, bool _growGoals,
    double _growthFocus, size_t _numRoots, size_t _numDirections,
    size_t _maxTrial) :
    BasicRRTStrategy<MPTraits>(_dm, _nf, _vc, _nc, _ex, _evaluators, _gt,
        _growGoals, _growthFocus, _numRoots, _numDirections, _maxTrial) {
  this->SetName("DynamicRegionRRT");
}


template <typename MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(XMLNode& _node) : BasicRRTStrategy<MPTraits>(_node) {
  this->SetName("DynamicRegionRRT");

  // Parameters.
  m_regionFactor = _node.Read("regionFactor", false, 2.5, 1., 4., "The region "
      "radius is this * robot radius");
  m_robotFactor = _node.Read("robotFactor", false, 1., 0., 1., "The robot is "
      "touching a region if its bounding sphere is at least this fraction "
      "into the region");
  m_velocityAlignment = _node.Read("velocityAlignment", false, .1, -1., .99,
      "Minimum dot product for sampled velocity and biasing direction.");
  m_backtrackFactor = _node.Read("backtrackFactor", false, .5, 0., 1.,
      "Fraction of extender's maximum distance to backtrack along the skeleton "
      "when searching for candidates");

  // Options.
  m_prune = _node.Read("pruneFlowGraph", false, true, "Enable/disable skeleton "
      "graph pruning");
  m_bucketing = _node.Read("bucketing", false, false,
      "Use topological bucketing");
  m_biasing = _node.Read("biasing", false, false,
      "Use topological velocity biasing for nonholonomic robots");
}

/*------------------------- MPBaseObject Overriddes --------------------------*/

template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
Print(std::ostream& _os) const {
  BasicRRTStrategy<MPTraits>::Print(_os);
  _os << "  Dynamic Regions properties:"
      << "\n\tRegion factor: " << m_regionFactor
      << "\n\tRobot factor: " << m_robotFactor
      << "\n\tPruning: " << (m_prune ? "enabled" : "disabled")
      << "\n\tVelocity Alignment: " << m_velocityAlignment
      << std::endl;
}


/*----------------------- MPStrategyMethod Overriddes ------------------------*/

template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
Initialize() {
  if(this->m_debug)
    Print(cout);

  BasicRRTStrategy<MPTraits>::Initialize();

  auto env = this->GetEnvironment();

  StatClass* stats = this->GetStatClass();
  stats->StartClock("DynamicRegionRRT::SkeletonConstruction");

  //Embed ReebGraph
  delete m_reebGraph;
  m_reebGraph = new ReebGraphConstruction();
  m_reebGraph->Construct(env, this->GetBaseFilename());

  // Create the workspace skeleton.
  m_skeleton = m_reebGraph->GetSkeleton();

  // Direct the workspace skeleton outward from the starting point.
  const Point3d start = this->m_query->GetQuery()[0].GetPoint();
  m_skeleton = m_skeleton.Direct(start);

  // Prune the workspace skeleton relative to the goal.
  if(m_prune)
    m_skeleton.Prune(this->m_query->GetQuery()[1].GetPoint());

  stats->StopClock("DynamicRegionRRT::SkeletonConstruction");

  // Fix the skelton clearance.
  FixSkeletonClearance();

  // Spark a region for each outgoing edge of start
  m_samplingRegion = nullptr;
  const double robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->
      GetBoundingSphereRadius();
  const double regionRadius = m_regionFactor * robotRadius;
  m_skeleton.InitRegions(start, regionRadius);

  // Initialize bucket map.
  if(m_bucketing) {
    m_buckets.clear();
    for(auto eit = m_skeleton.GetGraph().edges_begin();
        eit != m_skeleton.GetGraph().edges_end(); eit++) {
      const auto& path = eit->property();
      for(const auto& point : path)
        m_buckets.emplace(point, vector<VID>());
    }
    m_buckets[to_point(m_skeleton.GetRegionMap().begin()->first->GetCenter())].
        push_back(0);
  }


#ifdef VIZMO
  GetVizmo().GetEnv()->AddWorkspaceDecompositionModel(env->GetDecomposition());
  GetVizmo().GetEnv()->AddReebGraphModel(m_reebGraph);
  GetMainWindow()->GetModelSelectionWidget()->CallResetLists();

  // Make map non-selectable during execution.
  GetVizmo().GetMap()->SetSelectable(false);
#endif
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
Run() {
  if(this->m_debug)
    cout << "\nBegin DynamicRegionRRT::Run" << endl;

  StatClass* stats = this->GetStatClass();
  stats->StartClock("DynamicRegionRRT::Run");

  const double robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->
    GetBoundingSphereRadius();
  const double regionRadius = m_regionFactor * robotRadius;

  CfgType target(this->GetTask()->GetRobot());
  while(!this->EvaluateMap()) {
    // Find growth direction: either sample or bias towards a goal.
    if(this->m_query && DRand() < this->m_growthFocus &&
        !this->m_query->GetGoals().empty())
      target = this->m_query->GetRandomGoal();
    else
      target = this->SelectDirection();

    // Randomize Current Tree
    this->m_currentTree = this->m_trees.begin() + LRand() % this->m_trees.size();

    // Try to extend the tree towards target.
    VID recent = this->ExpandTree(target);

    // If the attempt succeeds, update the regions.
    if(recent != INVALID_VID) {
      CfgType& newest = this->GetRoadmap()->GetGraph()->GetVertex(recent);

      // Mark a successful sample in this region.
      if(m_samplingRegion)
        m_skeleton.IncrementSuccess(m_samplingRegion);

      // Move existing regions along.
      stats->StartClock("DynamicRegionRRT::AdvanceRegions");
      m_skeleton.AdvanceRegions(newest);
      stats->StopClock("DynamicRegionRRT::AdvanceRegions");

      // Safety check
      m_samplingRegion = nullptr;

      // Create new regions near newest.
      stats->StartClock("DynamicRegionRRT::CreateRegions");
      m_skeleton.CreateRegions(newest.GetPoint(), regionRadius);
      stats->StopClock("DynamicRegionRRT::CreateRegions");

      // Connect trees if there are more than one.
      this->ConnectTrees(recent);
    }
#ifdef VIZMO
    GetVizmo().GetMap()->RefreshMap();
  }
  GetVizmo().GetMap()->SetSelectable(true);
#else
  }
#endif

  stats->StopClock("DynamicRegionRRT::Run");

  if(this->m_debug)
    std::cout << "\nEnd DynamicRegionRRT::Run" << std::endl;
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
Finalize() {
  auto stats = this->GetStatClass();

  stats->SetStat("CandidateFraction",
      stats->GetStat("CandidateFraction") / stats->GetStat("NF::BucketUsed"));
  stats->SetStat("SampleTarget::AvgTime",
      stats->GetSeconds("DynamicRegionRRT::SampleTarget") /
      stats->GetStat("SampleTarget::Num"));

  BasicRRTStrategy<MPTraits>::Finalize();
}


/*----------------------------- RRT Overrides --------------------------------*/

template <typename MPTraits>
typename DynamicRegionRRT<MPTraits>::CfgType
DynamicRegionRRT<MPTraits>::
SelectDirection() {
  auto stats = this->GetStatClass();

  const Boundary* samplingBoundary{nullptr};
  auto robot = this->GetTask()->GetRobot();

  // Randomly select a sampling region.
  stats->StartClock("DynamicRegionRRT::SelectRegion");
  m_samplingRegion = m_skeleton.SelectRegion();
  stats->StopClock("DynamicRegionRRT::SelectRegion");

  if(!m_samplingRegion)
    samplingBoundary = this->GetEnvironment()->GetBoundary();
  else
    samplingBoundary = m_samplingRegion;

  // Generate the target q_rand from within the selected region.
  stats->StartClock("DynamicRegionRRT::SampleTarget");
  stats->IncStat("SampleTarget::Num");
  CfgType mySample(robot);
  mySample.GetRandomCfg(samplingBoundary);
  stats->StopClock("DynamicRegionRRT::SampleTarget");

  // Bias sample velocity.
  if(m_biasing and robot->IsNonholonomic() and m_samplingRegion) {
    stats->StartClock("DynamicRegionRRT::VelocityBiasing");
    // Resample until the new Cfg aims relatively along the biasing direction.
    const Vector3d bias = GetVelocityBias();
    if(bias.norm() == 0)
      throw RunTimeException(WHERE, "Bias cannot be zero.");
    Vector3d velocity;
    do {
      mySample.GetRandomVelocity();
      velocity = mySample.GetLinearVelocity().normalize();
      if(this->m_debug)
        std::cout << "\tSampled velocity direction: " << velocity
                  << "\n\t\tDot product with bias: " << velocity * bias
                  << (velocity * bias < m_velocityAlignment ? " < " : " >= ")
                  << m_velocityAlignment
                  << std::endl;
    } while(velocity * bias < m_velocityAlignment);
    this->GetStatClass()->StopClock("DynamicRegionRRT::VelocityBiasing");
  }

  return mySample;
}


template<class MPTraits>
typename DynamicRegionRRT<MPTraits>::VID
DynamicRegionRRT<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const TreeType& _tree) {
  auto stats = this->GetStatClass();

  // Use regular neighborhood finding if there is no region (i.e., we are
  // sampling from the whole environment).
  if(!m_bucketing or !m_samplingRegion) {
    stats->IncStat("NF::NoBucket");
    return BasicRRTStrategy<MPTraits>::FindNearestNeighbor(_cfg, _tree);
  }

  // If we have a sampling region, collect candidates from the bucket map.
  vector<VID> candidates = CollectCandidates(_cfg);

  // If there are no candidates, then search the whole tree.
  if(candidates.empty()) {
    stats->IncStat("NF::NoCandidates");
    return BasicRRTStrategy<MPTraits>::FindNearestNeighbor(_cfg, _tree);
  }

  // Otherwise, track the percentage of the roadmap constituted by the candidate
  // set and search the set for a nearest neighbor.
  auto g = this->GetRoadmap()->GetGraph();

  stats->IncStat("CandidateFraction", static_cast<double>(candidates.size()) /
      g->get_num_vertices());
  stats->IncStat("NF::BucketUsed");
  stats->StartClock("DynamicRegionRRT::BucketNF");

  vector<pair<VID, double>> neighbors;
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  nf->FindNeighbors(this->GetRoadmap(),
      candidates.begin(), candidates.end(),
      candidates.size() == g->get_num_vertices(),
      _cfg, std::back_inserter(neighbors));

  stats->StopClock("DynamicRegionRRT::BucketNF");

#if 0
  // Validation test to check if we are finding the right nearest neighbor.
  // Make sure this is off when running time experiments.
  {
    stats->StartClock("DynamicRegionRRT::CheckNF");

    vector<pair<VID, double>> neighbors2;
    nf->FindNeighbors(this->GetRoadmap(),
        _tree.begin(), _tree.end(),
        _tree.size() == g->get_num_vertices(),
        _cfg, std::back_inserter(neighbors2));

    if(neighbors[0].first == neighbors2[0].first)
      stats->IncStat("CorrectNeighbor");
    else {
      stats->IncStat("IncorrectNeighbor");
      const double neighborDistance = this->GetDistanceMetric("euclidean")->
          Distance(g->GetVertex(neighbors[0].first),
                   g->GetVertex(neighbors2[0].first));
      stats->SetStat("NeighborDistance", neighborDistance);
    }

    stats->StopClock("DynamicRegionRRT::CheckNF");
  }
#endif

  return neighbors.empty() ? INVALID_VID : neighbors[0].first;
}


template<typename MPTraits>
typename MPTraits::RoadmapType::VID
DynamicRegionRRT<MPTraits>::
Extend(const VID _nearVID, const CfgType& _qrand, const bool _lp) {
  // First extend as usual.
  const VID newVID = BasicRRTStrategy<MPTraits>::Extend(_nearVID, _qrand, _lp);

  // If we produced a valid extension, add the new node to the bucket map.
  if(m_bucketing and newVID != INVALID_VID and
      newVID == this->GetRoadmap()->GetGraph()->get_num_vertices() - 1) {
    auto stats = this->GetStatClass();
    stats->StartClock("DynamicRegionRRT::BucketNewCfg");
    stats->IncStat("BucketNodes");

    // If this node was sampled from a dynamic region, add it to the bucket at
    // the region's center.
    if(m_samplingRegion)
      m_buckets[to_point(m_samplingRegion->GetCenter())].push_back(newVID);
    // Otherwise, use the nearest bucket.
    else
      AddToNearestVisibleBucket(newVID);

    stats->StopClock("DynamicRegionRRT::BucketNewCfg");
  }

  return newVID;
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
std::vector<typename MPTraits::RoadmapType::VID>
DynamicRegionRRT<MPTraits>::
CollectCandidates(const CfgType& _cfg) {
  auto stats = this->GetStatClass();
  stats->StartClock("DynamicRegionRRT::CollectCandidates");

  // Initialize the candidate set with the bucket at the region's center.
  const Point3d currentPoint = to_point(m_samplingRegion->GetCenter());
  vector<VID> candidates = m_buckets[currentPoint];

  // Find the edge that the region is traversing.
  const auto& regionData = m_skeleton.GetRegionData(m_samplingRegion);
  SkeletonEdgeIter edge;
  WorkspaceSkeleton::vertex_iterator vi;
  m_skeleton.GetGraph().find_edge(regionData.edgeDescriptor, vi, edge);

  // Follow the skeleton backwards until we have traveled the extender's maximum
  // distance.
  const size_t currentIndex = regionData.edgeIndex;
  const double maxDist = this->GetExtender(this->m_exLabel)->GetMaxDistance() *
      m_backtrackFactor;
  BackTrackSkeletonEdge(candidates, edge, currentIndex, 0, maxDist);

  stats->StopClock("DynamicRegionRRT::CollectCandidates");
  return candidates;
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
BackTrackSkeletonEdge(vector<VID>& _candidates, SkeletonEdgeIter _edge,
    size_t _edgeIndex, double _distance, const double _maxDistance) {
  const auto& path = _edge->property();

  // While we haven't reached our max distance or the front of this skeleton
  // edge, copy VID's from the previous edge point's bucket into our candidate
  // set.
  while(_edgeIndex > 0 and _distance < _maxDistance) {
    const auto& currentPoint = path[_edgeIndex];
    const auto& previousPoint = path[--_edgeIndex];

    _distance += (previousPoint - currentPoint).norm();

    // We copy the previous point instead of the current one to avoid duplicates
    // when the search splits across multiple edges.
    const auto& bucket = m_buckets[previousPoint];
    std::copy(bucket.begin(), bucket.end(), std::back_inserter(_candidates));
  }

  // If we hit the front of the edge but not our max distance, back track along
  // each edge inbound on the edge's source vertex.
  if(_edgeIndex == 0 and _distance < _maxDistance) {
    const auto inEdges = m_skeleton.FindInboundEdges(_edge->source());
    for(auto inEdge : inEdges) {
      // Start from the last index in the edge path.
      const size_t endIndex = inEdge->property().size() - 1;

      // Back-track along this edge.
      BackTrackSkeletonEdge(_candidates, inEdge, endIndex, _distance,
          _maxDistance);
    }
  }
}


template <typename MPTraits>
const Vector3d
DynamicRegionRRT<MPTraits>::
GetVelocityBias() {
  // Get the region data.
  const auto& regionData = m_skeleton.GetRegionData(m_samplingRegion);
  const size_t index = regionData.edgeIndex;

  // Find the skeleton edge path the region is traversing.
  auto edge = m_skeleton.FindEdge(regionData.edgeDescriptor);
  const auto& path = edge->property();

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
  auto vertex = m_skeleton.GetGraph().find_vertex(edge->target());

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
void
DynamicRegionRRT<MPTraits>::
AddToNearestVisibleBucket(const VID _vid) {
  this->GetStatClass()->StartClock("DynamicRegionRRT::FindNearestBucket");

  const CfgType& cfg = this->GetRoadmap()->GetGraph()->GetVertex(_vid);
  const Point3d p = cfg.GetPoint();

  auto bestBucket = m_buckets.end();
  double bestDist = numeric_limits<double>::infinity();

  using CDType = CollisionDetectionValidity<MPTraits>;
  auto vc = static_cast<CDType*>(this->GetValidityChecker("rapid").get());

  // Check each bucket to find the closest visible one.
  for(auto iter = m_buckets.begin(); iter != m_buckets.end(); ++iter) {
    // Check distance from configuration to bucket point.
    const double dist = (p - iter->first).norm();

    // If this bucket point is closer than the current best and also visible, it
    // is the new best.
    if(bestDist > dist and vc->WorkspaceVisibility(p, iter->first)) {
      bestDist = dist;
      bestBucket = iter;
    }
  }

  this->GetStatClass()->StopClock("DynamicRegionRRT::FindNearestBucket");

  if(bestBucket != m_buckets.end()) {
    bestBucket->second.push_back(_vid);
    if(this->m_debug)
      std::cout << "Found nearest bucket at " << bestDist << std::endl;
  }
  else if(this->m_debug)
    std::cout << "No visible bucket found!" << std::endl;
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
FixSkeletonClearance() {
  this->GetStatClass()->StartClock("DynamicRegionRRT::FixSkeletonClearance");

  auto g = m_skeleton.GetGraph();

  if(this->m_debug)
    cout << "Skeleton has " << g.get_num_vertices() << " vertices "
         << "and " << g.get_num_edges() << " edges."
         << "\n\tPushing nodes with low clearance away from nearest obstacles:";

  const double robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->
      GetBoundingSphereRadius();
  auto boundary = this->GetEnvironment()->GetBoundary();
  auto vc = this->GetValidityChecker("pqp_solid");

  auto pointRobot = this->GetMPProblem()->GetRobot("point");

  // This is a cheaper version of our clearance utility that is optimized for a
  // point and doesn't compute a witness. It finds the minimum clearance of the
  // input point _p.
  auto getClearanceInfo = [&](const Point3d& _p) -> pair<double, Point3d> {
    // Check against obstacles using a point robot.
    CfgType cfg(_p, pointRobot);
    CDInfo cdInfo(true);
    vc->IsValid(cfg, cdInfo, "Skeleton Push");

    // Check against boundary.
    const double boundaryClearance = boundary->GetClearance(_p);
    if(boundaryClearance < cdInfo.m_minDist) {
      cdInfo.m_objectPoint = boundary->GetClearancePoint(_p);
      cdInfo.m_minDist = boundaryClearance;
    }

    // Return the minimum clearance and nearest obstacle point.
    return make_pair(cdInfo.m_minDist, cdInfo.m_objectPoint);
  };

  auto push = [&](Point3d& _p) {
    if(this->m_debug)
      cout << "\n\t\tPushing from: " << setprecision(4) << _p
           << "\n\t\t          to: ";

    // Get clearance info for this point.
    auto initialClearance = getClearanceInfo(_p);
    const Point3d& objectPoint = initialClearance.second;

    // Check if we are at least one robot radius away from the nearest obstacle.
    const Vector3d w = _p - objectPoint;   // From obstacle to original point.
    const double wMag = w.norm();
    Vector3d t = w * (robotRadius / wMag); // As w, but one robot radius long.
    if(wMag >= t.norm()) {
      if(this->m_debug)
        cout << "(already clear)" << endl;
      return;
    }

    // Try to improve the clearance if we are too close.
    int tries = 3;
    while(tries-- && wMag < t.norm()) {
      // Set the new point as the nearest object point plus t.
      const Point3d newPoint = t + objectPoint;
      auto newClearance = getClearanceInfo(newPoint);

      // If t has better clearance than _p, push _p to t and quit.
      if(newClearance.first > initialClearance.first) {
        _p = t + objectPoint;
        if(this->m_debug)
          cout << _p << endl;
        return;
      }
      // Otherwise, cut the difference between t and w in half and retry.
      else
        t = (w + t) / 2.;
    }
    if(this->m_debug)
      cout << "(FAILED)" << endl;
  };

  // Push flowgraph vertices.
  for(auto vit = g.begin(); vit != g.end(); ++vit)
    push(vit->property());

  // Push flowgraph edges.
  for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)
    for(auto pit = eit->property().begin(); pit < eit->property().end(); ++pit)
      push(*pit);

  if(this->m_debug)
    cout << "\n\tSkeleton clearance adjustment complete." << endl;

  this->GetStatClass()->StopClock("DynamicRegionRRT::FixSkeletonClearance");
}

/*----------------------------------------------------------------------------*/

#endif
