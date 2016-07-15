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

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef shared_ptr<Boundary> RegionPtr;
    typedef ReebGraphConstruction::FlowGraph FlowGraph;

    ///@}
    ///\name Construction
    ///@{

    DynamicRegionRRT(const CfgType& _start = CfgType(),
        const CfgType& _goal = CfgType(),
        string _dm = "euclidean", string _nf = "BFNF",
        string _extenderLabel = "BERO",
        string _vc = "PQP_SOLID",
        string _nc = "kClosest", string _gt = "UNDIRECTED_TREE",
        vector<string> _evaluators = vector<string>(),
        double _minDist = 0.001, double _growthFocus = 0.05,
        bool _evaluateGoal = true, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3,
        bool _growGoals = false);
    DynamicRegionRRT(MPProblemType* _problem, XMLNode& _node);

    ///@}
    ///\name MPBaseObject Overrides
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
    CfgType SelectDirection();

    ///@}

  private:

    ///\name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Prune the flow graph by removing all vertices that have no path
    ///        to the goal.
    /// \param[in] _f The flow graph to prune.
    void PruneFlowGraph(FlowGraph& _f) const;

    ///@}
    ///\name Internal State
    ///@{

    vector<RegionPtr> m_regions; ///< All Regions
    RegionPtr m_samplingRegion;  ///< Points to the current sampling region.

    ReebGraphConstruction* m_reebGraphConstruction; ///< Embedded reeb graph

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template<class MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(const CfgType& _start, const CfgType& _goal, string _dm,
    string _nf, string _extenderLabel, string _vc, string _nc, string _gt,
    vector<string> _evaluators, double _minDist,
    double _growthFocus, bool _evaluateGoal, size_t _numRoots,
    size_t _numDirections, size_t _maxTrial, bool _growGoals) :
    BasicRRTStrategy<MPTraits>(_dm, _nf, _vc, _nc, _gt, _extenderLabel,
    _evaluators, _minDist, _growthFocus, _evaluateGoal,
    _start, _goal, _numRoots, _numDirections, _maxTrial, _growGoals) {
  this->SetName("DynamicRegionRRT");
}

template<class MPTraits>
DynamicRegionRRT<MPTraits>::
DynamicRegionRRT(MPProblemType* _problem, XMLNode& _node) :
    BasicRRTStrategy<MPTraits>(_problem, _node) {
  this->SetName("DynamicRegionRRT");
}

/*-------------------------- MPBaseObject Overriddes -------------------------*/

template<class MPTraits>
void
DynamicRegionRRT<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();

  StatClass* stats = this->GetStatClass();

  //Embed ReebGraph
  stats->StartClock("ReebGraphConstruction");
  delete m_reebGraphConstruction;
  m_reebGraphConstruction = new ReebGraphConstruction();
  m_reebGraphConstruction->Construct(this->GetEnvironment(),
      this->GetBaseFilename());
#ifdef VIZMO
  GetVizmo().GetEnv()->AddTetGenDecompositionModel(m_reebGraphConstruction->
      GetTetrahedralization());
  GetVizmo().GetEnv()->AddReebGraphModel(m_reebGraphConstruction);
  GetMainWindow()->GetModelSelectionWidget()->CallResetLists();

  // Make map non-selectable during execution.
  GetVizmo().GetMap()->SetSelectable(false);
#endif

  stats->StopClock("ReebGraphConstruction");
}


template<class MPTraits>
void
DynamicRegionRRT<MPTraits>::
Run() {
  if(this->m_debug)
    cout << "\nBegin DynamicRegionRRT::Run" << endl;

  // Setup MP Variables
  StatClass* stats = this->GetStatClass();
  Environment* env = this->GetEnvironment();

  stats->StartClock("DynamicRegionRRT");

  const CfgType& s = this->m_query->GetQuery()[0];
  Vector3d start(s[0], s[1], s[2]);

  //Get directed flow network
  typedef FlowGraph::vertex_descriptor FVD;
  typedef FlowGraph::edge_descriptor FED;
  pair<FlowGraph, FVD> flow = m_reebGraphConstruction->
    GetFlowGraph(start, env->GetPositionRes());

  // Prune flow-graph of non-relevant paths.
  PruneFlowGraph(flow.first);

  unordered_map<FVD, bool> visited;
  for(auto vit = flow.first.begin(); vit != flow.first.end(); ++vit)
    visited[vit->descriptor()] = false;

#ifdef VIZMO
  // Make temporary models for the regions.
  map<RegionPtr, Model*> models;
  TempObjsModel tom;
#endif

  //Spark a region for each outgoing edge of start

  //Region structure stores tuple of flow edge descriptor,
  //index along flow edge, number of failed extentions
  unordered_map<RegionPtr, tuple<FED, size_t, size_t>> regions;
  //unordered_map<RegionPtr, pair<FED, size_t>> regions;

  const double regionRadius = 2 * env->GetRobot(0)->GetBoundingSphereRadius();
  auto sit = flow.first.find_vertex(flow.second);
  for(auto eit = sit->begin(); eit != sit->end(); ++eit) {
    auto i = regions.emplace(
        RegionPtr(new BoundingSphere(sit->property(), regionRadius)),
        make_tuple(eit->descriptor(), 0, 0));
        //make_pair(eit->descriptor(), 0));
    m_regions.push_back(i.first->first);
#ifdef VIZMO
    models[m_regions.back()] = new ThreadSafeSphereModel(
        m_regions.back()->GetCenter(), regionRadius);
    tom.AddOther(models[m_regions.back()]);
#endif
  }
  visited[sit->descriptor()] = true;

  CfgType dir;
  while(!this->EvaluateMap()) {
    //find my growth direction. Default is to randomly select node or bias
    //towards a goal
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

      if(m_samplingRegion) {
        get<2>(regions[m_samplingRegion]) = 0;

        while(env->InBounds(newest, m_samplingRegion)) {
          Vector3d cur = m_samplingRegion->GetCenter();

          auto& pr = regions[m_samplingRegion];
          FlowGraph::vertex_iterator vi;
          FlowGraph::adj_edge_iterator ei;
          flow.first.find_edge(get<0>(pr), vi, ei);
          //flow.first.find_edge(pr.first, vi, ei);
          vector<Vector3d>& path = ei->property();
          size_t& i = get<1>(pr);
          //size_t& i = pr.second;
          size_t j = i+1;
          if(j < path.size()) {
            Vector3d& next = path[j];
            m_samplingRegion->ApplyOffset(next-cur);
#ifdef VIZMO
            static_cast<ThreadSafeSphereModel*>(models[m_samplingRegion])->
                MoveTo(next);
#endif
            i = j;
          }
          //else need to delete region
          else {
#ifdef VIZMO
            tom.RemoveOther(models[m_samplingRegion]);
            models.erase(m_samplingRegion);
#endif
            auto rit = find(m_regions.begin(), m_regions.end(), m_samplingRegion);
            m_regions.erase(rit);
            regions.erase(m_samplingRegion);
            break;
          }
        }
      }

      //Add new regions
      Vector3d p(newest[0], newest[1], newest[2]);

      for(auto vit = flow.first.begin(); vit != flow.first.end(); ++vit) {
        double dist = (vit->property() - p).norm();
        if(dist < regionRadius && !visited[vit->descriptor()]) {
          for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
            auto i = regions.emplace(
                RegionPtr(new BoundingSphere(vit->property(), regionRadius)),
                make_tuple(eit->descriptor(), 0, 0));
            m_regions.push_back(i.first->first);
#ifdef VIZMO
            models[m_regions.back()] = new ThreadSafeSphereModel(
                vit->property(), regionRadius);
            tom.AddOther(models[m_regions.back()]);
#endif
          }
          visited[vit->descriptor()] = true;
        }
      }

      //connect various trees together
      this->ConnectTrees(recent);
    }
    else {
      if(m_samplingRegion) {
        ++get<2>(regions[m_samplingRegion]);
        if(get<2>(regions[m_samplingRegion]) > 1000) {
          auto rit = find(m_regions.begin(), m_regions.end(), m_samplingRegion);
          m_regions.erase(rit);
          regions.erase(m_samplingRegion);
        }
      }
    }
#ifdef VIZMO
    GetVizmo().GetMap()->RefreshMap();
#endif
  }

  stats->StopClock("DynamicRegionRRT");

  m_regions.clear();
  regions.clear();

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

  size_t _index = rand() % (m_regions.size() + 1);

  if(_index == m_regions.size()) {
    m_samplingRegion.reset();
    samplingBoundary = this->GetEnvironment()->GetBoundary();
  }
  else {
    m_samplingRegion = m_regions[_index];
    samplingBoundary = m_samplingRegion;
  }

  try {
    CfgType mySample;
    mySample.GetRandomCfg(env,samplingBoundary);
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
void
DynamicRegionRRT<MPTraits>::
PruneFlowGraph(FlowGraph& _f) const {
  using VD = FlowGraph::vertex_descriptor;

  // Find the flow-graph node nearest to the goal.
  const CfgType& goalCfg = this->m_query->GetQuery()[1];
  Vector3d goalPoint(goalCfg[0], goalCfg[1], goalCfg[2]);
  double closestDistance = std::numeric_limits<double>::max();
  VD goal;
  for(auto vit = _f.begin(); vit != _f.end(); ++vit) {
    const auto& thisPoint = vit->property();
    double distance = (thisPoint - goalPoint).norm();
    if(distance < closestDistance) {
      closestDistance = distance;
      goal = vit->descriptor();
    }
  }

  // Initialize a list of vertices to prune with every vertex in the graph.
  vector<VD> toPrune;
  toPrune.reserve(_f.get_num_vertices());
  for(const auto& v : _f)
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

    for(auto ancestor : _f.find_vertex(current)->predecessors())
      q.push(ancestor);
  } while(!q.empty());

  // Remove the vertices we aren't keeping.
  for(auto vd : toPrune)
    if(_f.find_vertex(vd) != _f.end())
      _f.delete_vertex(vd);
}

/*----------------------------------------------------------------------------*/

#endif
