#ifndef DYNAMIC_REGION_RRT_H_
#define DYNAMIC_REGION_RRT_H_

#include <queue>
#include <unordered_map>

#include "BasicRRTStrategy.h"

#include "Environment/Boundary.h"
#include "Environment/BoundingSphere.h"
#include "Utilities/ReebGraphConstruction.h"

#ifdef VIZMO
#include "GUI/ModelSelectionWidget.h"
#include "Models/TempObjsModel.h"
#include "Models/ThreadSafeSphereModel.h"
#include "Models/Vizmo.h"
#endif

////////////////////////////////////////////////////////////////////////////////
/// \brief  DynamicRegionRRT uses an embedded Reeb graph to guide dynamic
///         sampling regions through the environment.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class DynamicRegionRRT : public BasicRRTStrategy<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPProblemType  MPProblemType;
    typedef typename MPTraits::CfgType        CfgType;
    typedef typename MPTraits::CfgRef         CfgRef;
    typedef typename MPTraits::WeightType     WeightType;
    typedef typename MPProblemType::VID       VID;
    typedef typename MPProblemType::GraphType GraphType;

    ///@}
    ///\name Local Types
    ///@{

    typedef shared_ptr<Boundary>              RegionPtr;
    typedef ReebGraphConstruction::FlowGraph  FlowGraph;

    ///@}
    ///\name Construction
    ///@{

    DynamicRegionRRT(string _dm = "euclidean", string _nf = "Nearest",
        string _vc = "rapid", string _nc = "kClosest", string _ex = "BERO",
        vector<string> _evaluators = vector<string>(),
        string _gt = "UNDIRECTED_TREE",  bool _growGoals = false,
        double _growthFocus = .05, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3);

    DynamicRegionRRT(MPProblemType* _problem, XMLNode& _node);

    virtual ~DynamicRegionRRT() = default;

    ///@}
    ///\name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Run() override;

    ///@}

  protected:

    ///\name RRT Overrides
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief  Computes the growth direction for the RRT, choosing between the
    ///         entire environment and each attract region with uniform
    ///         probability to generate q_rand.
    /// \return The resulting growth direction.
    virtual CfgType SelectDirection() override;

    ///@}

  private:

    ///\name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// Initialize the flow graph.
    /// @return The flow-graph vertex descriptor of the starting node.
    size_t InitializeFlowGraph();

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Prune the flow graph by removing all vertices that have no path
    ///        to the goal.
    /// \param[in] _f The flow graph to prune.
    void PruneFlowGraph(FlowGraph* _f);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Push the nodes and edges of the flow graph to the medial axis.
    /// \param[in] _f The flow graph to push.
    void FixFlowgraphClearance(FlowGraph* _f) const;

    ////////////////////////////////////////////////////////////////////////////
    /// Initialize the tracking of current regions.
    /// @param _startID The vertex descriptor of the starting flow node.
    void InitializeCurrentRegions(size_t _startID);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Test whether the newest cfg is touching a region
    /// \param[in] _cfg The newest cfg.
    /// \param[in] _region The region begin tested.
    bool IsTouching(const CfgType& _cfg, RegionPtr _region);

    ///@}
    ///\name Internal State
    ///@{

    bool m_prune{true};       ///< Prune the flow graph?
    double m_regionFactor{2}; ///< The region radius is this * robot radius.
    double m_regionRadius;    ///< The region radius.
    double m_robotFactor{1.}; ///< The robot is touching if inside by this amount.

    RegionPtr m_samplingRegion;  ///< Points to the current sampling region.

    FlowGraph* m_flowGraph{nullptr}; ///< The flow graph.

    /// Which flow vertices have been visited?
    unordered_map<FlowGraph::vertex_descriptor, bool> m_visited;

    /// Current region tracking structure maps a region to a tuple of
    /// <flow edge descriptor, index along flow edge, num failed extentions>
    unordered_map<RegionPtr, tuple<FlowGraph::edge_descriptor, size_t, size_t>>
        m_currentRegions;

    // Extra models for vizmo land.
#ifdef VIZMO
    // Make temporary models for the regions.
    TempObjsModel* m_tom{nullptr};
    map<RegionPtr, Model*> m_models;
#endif

    ///@}
};

/*------------------------------ Construction --------------------------------*/


template<class MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(string _dm, string _nf, string _vc, string _nc, string _ex,
    vector<string> _evaluators, string _gt, bool _growGoals,
    double _growthFocus, size_t _numRoots, size_t _numDirections,
    size_t _maxTrial) :
    BasicRRTStrategy<MPTraits>(_dm, _nf, _vc, _nc, _ex, _evaluators, _gt,
        _growGoals, _growthFocus, _numRoots, _numDirections, _maxTrial) {
  this->SetName("DynamicRegionRRT");
}


template<class MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(MPProblemType* _problem, XMLNode& _node) :
    BasicRRTStrategy<MPTraits>(_problem, _node) {
  this->SetName("DynamicRegionRRT");

  m_regionFactor = _node.Read("regionFactor", false, 2.5, 1., 4., "The region "
      "radius is this * robot radius");
  m_prune = _node.Read("pruneFlowGraph", false, true, "Enable/disable flow "
      "graph pruning");
  m_robotFactor = _node.Read("robotFactor", false, 1., 0., 1., "The robot is "
      "touch if inside by this amount");
}

/*----------------------- MPStrategyMethod Overriddes ------------------------*/

template<class MPTraits>
void
DynamicRegionRRT<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();
#ifdef VIZMO
  m_tom = new TempObjsModel();
#endif

  auto env = this->GetEnvironment();
  m_samplingRegion.reset();

  // Compute region radius for our robot.
  m_regionRadius = m_regionFactor * env->GetRobot(0)->GetBoundingSphereRadius();

  size_t startFlowVID = InitializeFlowGraph();
  FixFlowgraphClearance(m_flowGraph);
  InitializeCurrentRegions(startFlowVID);

  // If we're in vizmo land, add the decomp/graph models to the scene.
#ifdef VIZMO
  GetVizmo().GetEnv()->AddWorkspaceDecompositionModel(env->GetDecomposition());
  GetVizmo().GetEnv()->AddGraphModel(*m_flowGraph);
  GetMainWindow()->GetModelSelectionWidget()->CallResetLists();

  // Make map non-selectable during execution.
  GetVizmo().GetMap()->SetSelectable(false);
#endif
}


template<class MPTraits>
void
DynamicRegionRRT<MPTraits>::
Run() {
  StatClass* stats = this->GetStatClass();
  stats->StartClock("DynamicRegionRRT::Run");

  if(this->m_debug)
    cout << "\nBegin DynamicRegionRRT::Run" << endl;

  while(!this->EvaluateMap()) {
    // Find growth direction.
    CfgType dir;
    if(this->m_query && DRand() < this->m_growthFocus &&
        !this->m_query->GetGoals().empty())
      dir = this->m_query->GetRandomGoal();
    else
      dir = this->SelectDirection();

    // Randomize Current Tree
    this->m_currentTree = this->m_trees.begin() + LRand() % this->m_trees.size();

    VID recent = this->ExpandTree(dir);
    if(recent != INVALID_VID) {

      CfgType& newest = this->GetRoadmap()->GetGraph()->GetVertex(recent);

      if(m_samplingRegion)
        get<2>(m_currentRegions[m_samplingRegion]) = 0;

      for(auto iter = m_currentRegions.begin(); iter != m_currentRegions.end();) {
        RegionPtr region = iter->first;
        bool increment = true;
        while(IsTouching(newest, region)) {
          Vector3d cur = region->GetCenter();

          FlowGraph::vertex_iterator vi;
          FlowGraph::adj_edge_iterator ei;
          m_flowGraph->find_edge(get<0>(iter->second), vi, ei);
          const vector<Vector3d>& path = ei->property();
          size_t i = get<1>(iter->second);
          size_t j = i + 1;
          if(j < path.size()) {
            Vector3d next = path[j];
            region->ApplyOffset(next - cur);
#ifdef VIZMO
            static_cast<ThreadSafeSphereModel*>(m_models[region])->MoveTo(next);
#endif
            get<1>(iter->second) = j;
          }
          //else need to delete region
          else {
#ifdef VIZMO
            m_tom->RemoveModel(m_models[region]);
            m_models.erase(region);
#endif
            iter = m_currentRegions.erase(iter);
            increment = false;
            break;
          }
        }
        if(increment) ++iter;
      }

      //Add new regions
      Vector3d p = newest.GetPoint();

      for(auto vit = m_flowGraph->begin(); vit != m_flowGraph->end(); ++vit) {
        double dist = (vit->property() - p).norm();
        if(dist < m_regionRadius && !m_visited[vit->descriptor()]) {
          for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
            RegionPtr r(new BoundingSphere(vit->property(), m_regionRadius));
            m_currentRegions.emplace(r, make_tuple(eit->descriptor(), 0, 0));
#ifdef VIZMO
            m_models[r] = new ThreadSafeSphereModel(vit->property(),
                m_regionRadius);
            m_tom->AddModel(m_models[r]);
#endif
          }
          m_visited[vit->descriptor()] = true;
        }
      }

      //connect various trees together
      this->ConnectTrees(recent);
    }
    else {
      if(m_samplingRegion) {
        ++get<2>(m_currentRegions[m_samplingRegion]);
        if(get<2>(m_currentRegions[m_samplingRegion]) > 1000)
          m_currentRegions.erase(m_samplingRegion);
      }
    }
#ifdef VIZMO
    GetVizmo().GetMap()->RefreshMap();
#endif
  }

  stats->StopClock("DynamicRegionRRT::Run");
#ifdef VIZMO
  m_models.clear();
  delete m_tom;
  m_tom = nullptr;
#endif

  m_currentRegions.clear();

  if(this->m_debug)
    cout<<"\nEnd DynamicRegionRRT::Run" << endl;
}

/*----------------------------- RRT Overrides --------------------------------*/

template<class MPTraits>
typename DynamicRegionRRT<MPTraits>::CfgType
DynamicRegionRRT<MPTraits>::
SelectDirection() {
  RegionPtr samplingBoundary;
  Environment* env = this->GetEnvironment();

  size_t _index = rand() % (m_currentRegions.size() + 1);

  if(_index == m_currentRegions.size()) {
    m_samplingRegion.reset();
    samplingBoundary = this->GetEnvironment()->GetBoundary();
  }
  else {
    auto iter = m_currentRegions.begin();
    advance(iter, _index);
    m_samplingRegion = iter->first;
    samplingBoundary = m_samplingRegion;
  }

  try {
    CfgType mySample;
    mySample.GetRandomCfg(env, samplingBoundary);
    return mySample;
  }
  //catch Boundary too small exception
  catch(PMPLException _e) {
    CfgType mySample;
    mySample.GetRandomCfg(env);
    return mySample;
  }
  //catch all others and exit
  catch(exception _e) {
    cerr << _e.what() << endl;
    exit(1);
  }
}

/*-------------------------------- Helpers -----------------------------------*/

template <typename MPTraits>
size_t
DynamicRegionRRT<MPTraits>::
InitializeFlowGraph() {
  auto stats = this->GetStatClass();
  auto env = this->GetEnvironment();

  // Build Reeb graph.
  stats->StartClock("ReebGraphConstruction");
  auto reebGraph = new ReebGraphConstruction();
  reebGraph->Construct(env, this->GetBaseFilename());
  stats->StopClock("ReebGraphConstruction");

  // Build flow graph.
  stats->StartClock("FlowGraphConstruction");
  Vector3d start = this->m_query->GetQuery()[0].GetPoint();

  // Get flow from reeb graph.
  pair<FlowGraph*, FlowGraph::vertex_descriptor> flow = reebGraph->
      GetFlowGraph(start, env->GetPositionRes());
  delete m_flowGraph;
  m_flowGraph = flow.first;

  // Prune flow graph of non-relevant paths.
  if(m_prune)
    PruneFlowGraph(m_flowGraph);

  stats->StopClock("FlowGraphConstruction");
  return flow.second;
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
PruneFlowGraph(FlowGraph* _f) {
  using VD = FlowGraph::vertex_descriptor;

  // Find the flow-graph node nearest to the goal.
  const CfgType& goalCfg = this->m_query->GetQuery()[1];
  Vector3d goalPoint(goalCfg[0], goalCfg[1], goalCfg[2]);
  double closestDistance = std::numeric_limits<double>::max();
  VD goal;
  for(auto vit = _f->begin(); vit != _f->end(); ++vit) {
    const auto& thisPoint = vit->property();
    double distance = (thisPoint - goalPoint).norm();
    if(distance < closestDistance) {
      closestDistance = distance;
      goal = vit->descriptor();
    }
  }

  // Initialize a list of vertices to prune with every vertex in the graph.
  vector<VD> toPrune;
  toPrune.reserve(_f->get_num_vertices());
  for(const auto& v : *_f)
    toPrune.push_back(v.descriptor());

  // Remove vertices from the prune list by starting from the goal and working
  // backwards up the incoming edges. Don't prune any vertex that is an ancestor
  // of the goal.
  queue<VD> q;
  q.push(goal);
  do {
    VD current = q.front();
    q.pop();

    auto iter = find(toPrune.begin(), toPrune.end(), current);
    if(iter != toPrune.end())
      toPrune.erase(iter);

    for(auto ancestor : _f->find_vertex(current)->predecessors())
      q.push(ancestor);
  } while(!q.empty());

  // Remove the vertices we aren't keeping.
  for(auto vd : toPrune)
    if(_f->find_vertex(vd) != _f->end())
      _f->delete_vertex(vd);
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
FixFlowgraphClearance(FlowGraph* _f) const {
  this->GetStatClass()->StartClock("FlowgraphClearance");

  if(this->m_debug)
    cout << "Flow graph has " << _f->get_num_vertices() << " vertices "
         << " and " << _f->get_num_edges() << " edges."
         << "\n\tPushing nodes with low clearance away from nearest obstacles:";

  const double robotRadius = this->GetEnvironment()->GetRobot(0)->
      GetBoundingSphereRadius();
  auto boundary = this->GetEnvironment()->GetBoundary();
  auto vc = this->GetValidityChecker("pqp_solid");

  // This is a cheaper version of our clearance utility that is optimized for a
  // point and doesn't compute a witness. It finds the minimum clearance of the
  // input point _p.
  auto getClearanceInfo = [&](const Point3d& _p) -> pair<double, Point3d> {
    // Check against obstacles using a point robot.
    CfgType cfg(_p, size_t(-1));
    CDInfo cdInfo(true);
    vc->IsValid(cfg, cdInfo, "Flowgraph Push");

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
  for(auto vit = _f->begin(); vit != _f->end(); ++vit)
    push(vit->property());

  // Push flowgraph edges.
  for(auto eit = _f->edges_begin(); eit != _f->edges_end(); ++eit)
    for(auto pit = eit->property().begin(); pit < eit->property().end(); ++pit)
      push(*pit);

  if(this->m_debug)
    cout << "\n\tFlow-graph clearance adjustment complete." << endl;

  this->GetStatClass()->StopClock("FlowgraphClearance");
}


template <typename MPTraits>
void
DynamicRegionRRT<MPTraits>::
InitializeCurrentRegions(size_t _startID) {
  auto startFlowVertex = m_flowGraph->find_vertex(_startID);

  // Track which flow-graph vertices have been visited.
  m_visited.clear();
  for(auto vit = m_flowGraph->begin(); vit != m_flowGraph->end(); ++vit)
    m_visited[vit->descriptor()] = false;
  m_visited[startFlowVertex->descriptor()] = true;

  // Intialize current regions.
  m_currentRegions.clear();
  for(auto eit = startFlowVertex->begin(); eit != startFlowVertex->end(); ++eit) {
    RegionPtr r(new BoundingSphere(startFlowVertex->property(), m_regionRadius));
    m_currentRegions.emplace(r, make_tuple(eit->descriptor(), 0, 0));
#ifdef VIZMO
    m_models[r] = new ThreadSafeSphereModel(r->GetCenter(), m_regionRadius);
    m_tom->AddModel(m_models[r]);
#endif
  }
}


template <typename MPTraits>
bool
DynamicRegionRRT<MPTraits>::
IsTouching(const CfgType& _cfg, RegionPtr _region) {
  auto region = static_pointer_cast<BoundingSphere>(_region);

  const Point3d& robotCenter = _cfg.GetPoint();
  const Point3d& regionCenter = region->GetCenter();

  double robotRadius = this->GetEnvironment()->GetRobot(0)->
      GetBoundingSphereRadius();
  double regionRadius = region->GetRadius();

  // The robot is touching if at least one robot factor of it's bounding sphere
  // penetrates into the region.
  double dist = (regionCenter - robotCenter).norm();
  double maxPenetration = robotRadius + regionRadius - dist;
  return maxPenetration > 0 && maxPenetration >= 2 * robotRadius * m_robotFactor;
}

/*----------------------------------------------------------------------------*/

#endif
