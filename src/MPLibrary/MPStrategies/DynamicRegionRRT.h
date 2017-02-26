#ifndef DYNAMIC_REGION_RRT_H_
#define DYNAMIC_REGION_RRT_H_

#include <queue>
#include <unordered_map>

#include "BasicRRTStrategy.h"

#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"
#include "Utilities/ReebGraphConstruction.h"

#include "Utilities/MedialAxisUtilities.h"
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
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Run() override;

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

    /// Returns the direction of the next point in the edge path
    /// @param _region the reference region
    /// @return A unit vector if the direction the region is moving
    Vector3d RegionDirection(Boundary* _region);

    void FixSkeletonClearance();

    ///@}

  private:

    ///@}
    ///@name Internal State
    ///@{

    bool m_prune{true};          ///< Prune the flow graph?

    double m_regionFactor{2.5};  ///< The region radius is this * robot radius.
    double m_robotFactor{1.};
        ///< The robot is touching a sampling region if inside by this amount.

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
DynamicRegionRRT(XMLNode& _node) :
    BasicRRTStrategy<MPTraits>(_node) {
  this->SetName("DynamicRegionRRT");
  m_regionFactor = _node.Read("regionFactor", false, 2.5, 1., 4., "The region "
      "radius is this * robot radius");
  m_prune = _node.Read("pruneFlowGraph", false, true, "Enable/disable skeleton "
      "graph pruning");
  m_robotFactor = _node.Read("robotFactor", false, 1., 0., 1., "The robot is "
      "touch if inside by this amount");
}

/*----------------------- MPStrategyMethod Overriddes ------------------------*/

template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();

  auto env = this->GetEnvironment();

  StatClass* stats = this->GetStatClass();
  stats->StartClock("SkeletonConstruction");

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

  // Spark a region for each outgoing edge of start
  const double robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->
      GetBoundingSphereRadius();
  const double regionRadius = m_regionFactor * robotRadius;
  m_skeleton.MarkAllNodesUnvisited();
  m_skeleton.InitRegions(start, regionRadius);

  stats->StopClock("SkeletonConstruction");

  // Fix the skelton clearance.
  FixSkeletonClearance();

  // Initialize bucket map.
  for(auto eit = m_skeleton.GetGraph().edges_begin();
      eit != m_skeleton.GetGraph().edges_end(); eit++) {
    const auto& path = eit->property();
    for(auto& n : path)
      m_buckets.emplace(n, vector<VID>());
  }

  m_buckets[to_point(m_skeleton.GetRegionMap().begin()->first->GetCenter())].push_back(0);

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
      m_skeleton.AdvanceRegions(newest);

      // Safety check
      m_samplingRegion = nullptr;

      // Create new regions near newest.
      m_skeleton.CreateRegions(newest.GetPoint(), regionRadius);

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

/*----------------------------- RRT Overrides --------------------------------*/

template <typename MPTraits>
typename DynamicRegionRRT<MPTraits>::CfgType
DynamicRegionRRT<MPTraits>::
SelectDirection() {
  const Boundary* samplingBoundary{nullptr};
  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();

  // Randomly select a sampling region.
  m_samplingRegion = m_skeleton.SelectRegion();

  if(!m_samplingRegion)
    samplingBoundary = this->GetEnvironment()->GetBoundary();
  else
    samplingBoundary = m_samplingRegion;

  // Generate the target q_rand from within the selected region.
  try {
    CfgType mySample(robot);

// velocity correction
#if 0
    Vector3d dir = RegionDirection(samplingBoundary);
    double alpha = -1;
    while(alpha <= m_velLimit) {
      mySample.GetRandomCfg(env, samplingBoundary);
      const auto& vels = mySample.GetVelocity();
      Vector3d velDir{vels[0], vels[1], vels[2]};
      alpha = dir * velDir;
    }
#else
    mySample.GetRandomCfg(env, samplingBoundary);
#endif
    return mySample;
  }
  // Catch Boundary too small exception.
  catch(PMPLException _e) {
    CfgType mySample(robot);
    mySample.GetRandomCfg(env);
    return mySample;
  }
}

template<class MPTraits>
typename DynamicRegionRRT<MPTraits>::VID
DynamicRegionRRT<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const TreeType& _tree) {
  auto stats = this->GetStatClass();

  // Use regular neighborhood finding if there is no region (i.e., we are
  // sampling from the whole environment).
  if(!m_samplingRegion) {
    stats->IncStat("No Sampling Region");
    return BasicRRTStrategy<MPTraits>::FindNearestNeighbor(_cfg, _tree);
  }

  // If we have a sampling region, collect candidates from the bucket map.
  vector<VID> candidates = CollectCandidates(_cfg);

  // If there are no candidates, then search the whole tree.
  if(candidates.empty()) {
    stats->IncStat("No Candidates");
    return BasicRRTStrategy<MPTraits>::FindNearestNeighbor(_cfg, _tree);
  }

  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);

  stats->IncStat("BucketNF Call");
  stats->StartClock("BucketNF");
  vector<pair<VID, double>> neighbors;
  nf->FindNeighbors(this->GetRoadmap(),
      candidates.begin(), candidates.end(),
      candidates.size() == this->GetRoadmap()->GetGraph()->get_num_vertices(),
      _cfg, back_inserter(neighbors));
  stats->StopClock("BucketNF");

#if 0
  // Validation test to check if we are finding the right nearest neighbor.
  // Make sure this is off when running time experiments.
  {
    vector<pair<VID, double>> neighbors2;
    stats->StartClock("CheckNF");
    nf->FindNeighbors(this->GetRoadmap(),
        _tree.begin(), _tree.end(),
        _tree.size() == this->GetRoadmap()->GetGraph()->get_num_vertices(),
        _cfg, back_inserter(neighbors2));
    stats->StopClock("CheckNF");

    if(neighbors[0].first == neighbors2[0].first)
      stats->IncStat("CorrectNeighbor");
    else {
      stats->IncStat("IncorrectNeighbor");
      //stats->SetStat("NeighborDistance", neighbor
    }
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
  if(newVID != INVALID_VID) {
    auto stats = this->GetStatClass();
    stats->StartClock("BucketNewCfg");

    // If this node was sampled from a dynamic region, add it to the bucket at
    // the region's center.
    if(m_samplingRegion)
      m_buckets[to_point(m_samplingRegion->GetCenter())].push_back(newVID);
    // Otherwise, use the nearest bucket.
    else {
      const CfgType& cfg = this->GetRoadmap()->GetGraph()->GetVertex(newVID);
      const Point3d p = cfg.GetPoint();

      auto bestBucket = m_buckets.end();
      double bestDist = numeric_limits<double>::infinity();

      for(auto iter = m_buckets.begin(); iter != m_buckets.end(); ++iter) {
        const double dist = (p - iter->first).norm();
        if(bestDist > dist) {
          bestDist = dist;
          bestBucket = iter;
        }
      }
      bestBucket->second.push_back(newVID);
    }
    stats->StopClock("BucketNewCfg");
  }

  return newVID;
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
std::vector<typename MPTraits::RoadmapType::VID>
DynamicRegionRRT<MPTraits>::
CollectCandidates(const CfgType& _cfg) {
  auto stats = this->GetStatClass();
  stats->StartClock("CollectCandidates");

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
  const double maxDist = this->GetExtender(this->m_exLabel)->GetMaxDistance();
  BackTrackSkeletonEdge(candidates, edge, currentIndex, 0, maxDist);

  stats->StopClock("CollectCandidates");
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
Vector3d
DynamicRegionRRT<MPTraits>::
RegionDirection(Boundary* _region) {
  // if the region isn't in the list (i.e. the environment) then a normalized
  // random vector is returned
  if(!m_samplingRegion)
    return Vector3d{DRand()}.normalize();

  //this->GetStatClass()->StartClock("VelocityBiasing");
  //this->GetStatClass()->StopClock("VelocityBiasing");

  const auto& regionMap = m_skeleton.GetRegionMap();
  auto iter = regionMap.find(m_samplingRegion);

  const auto& regionData = iter->second;
  size_t pointIndex = regionData.edgeIndex;

  // find the edge
  decltype(m_skeleton.GetGraph().begin()) vi;
  decltype(vi->begin()) ei;
  m_skeleton.GetGraph().find_edge(regionData.edgeDescriptor, vi, ei);

  const auto& path = ei->property();

  // get the edge of the current region
  Point3d rCenter = path[pointIndex];

  // get the target of the current path
  Point3d target;
  if(pointIndex >= path.size() - 1) {
    // find the vertex
    auto vit = m_skeleton.GetGraph().find_vertex(ei->target());
    std::vector<decltype(vit->begin())> li;
    // if the vertex is found, then loop through all incident edges.
    // If the edge is an out edge then add to list otherwise ignore.
    if(vit != m_skeleton.GetGraph().end()) {
      for(auto iter = vit->begin(); iter != vit->end(); ++iter)
        if(iter->source() == vi->descriptor())
          li.push_back(iter);
      // if list not empty then select a random edge and bias in that direction.
      // other wise return random vector
      if(!li.empty()) {
        size_t index = rand() % li.size();
        target = li[index]->property()[2];
      }
      else
        return Vector3d{DRand()}.normalize();
    }
    else
      // other wise return random vector
      return Vector3d{DRand()}.normalize();
  }
  else
    target = path[pointIndex + 1];


  if(this->m_debug)
    std::cout << "Region Center:\t" << rCenter
              << "\nTarget Center:\t" << target
              << "\nVel Unit Vector:\t" << (target - rCenter).normalize()
              << std::endl;

  // returns a unit vector
  return (target - rCenter).normalize();
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
FixSkeletonClearance() {
  this->GetStatClass()->StartClock("FixSkeletonClearance");

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
    CDInfo cdInfo;
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

  this->GetStatClass()->StopClock("FixSkeletonClearance");
}

/*----------------------------------------------------------------------------*/

#endif
