#ifndef KINODYNAMIC_REGION_RRT_H_
#define KINODYNAMIC_REGION_RRT_H_

#include <unordered_map>

#include "KinodynamicRRTStrategy.h"

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
/// \brief  KinoDynamicRegionRRT
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class KinoDynamicRegionRRT : public KinodynamicRRTStrategy<MPTraits> {

  public:

    // Local Types
    typedef typename MPTraits::CfgType StateType;
    typedef typename MPTraits::CfgRef StateRef;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer
        NeighborhoodFinderPointer;
    typedef vector<VID> TreeType;
    typedef typename MPProblemType::ExtenderPointer ExtenderPointer;
    typedef shared_ptr<Boundary> RegionPtr;
    typedef ReebGraphConstruction::FlowGraph FlowGraph;

    // Construction
    KinoDynamicRegionRRT(const StateType& _start = StateType(),
        const StateType& _goal = StateType(),
        string _dm = "", string _nf = "", string _vc = "",
        string _nc = "", string _gt = "",
        string _extenderLabel = "",
        vector<string> _evaluators = vector<string>(),
        double _minDist = 0.001, double _growthFocus = 0.05,
        bool _evaluateGoal = true, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3,
        double _goalDist = 10.0);
    KinoDynamicRegionRRT(MPProblemType* _problem, XMLNode& _node);

    void ParseXML(XMLNode& _node);
    // Inherited functions
    void Initialize();
    void Run();
    void Finalize();

  private:

    ////////////////////////////////////////////////////////////////////////////
    /// \brief  Computes the growth direction for the RRT, choosing between the
    ///         entire environment and each attract region with uniform
    ///         probability to generate q_rand.
    /// \return The resulting growth direction.
    StateType SelectDirection();

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Prune the flow graph by removing all vertices that have no path
    ///        to the goal.
    /// \param[in] _f The flow graph to prune.
    void PruneFlowGraph(FlowGraph& _f) const;

    vector<RegionPtr> m_regions; ///< All Regions
    RegionPtr m_samplingRegion;  ///< Points to the current sampling region.
    double m_regionRadius{2.5};
    ReebGraphConstruction* m_reebGraphConstruction; ///< Embedded reeb graph
};


template<class MPTraits>
KinoDynamicRegionRRT<MPTraits>::
KinoDynamicRegionRRT(const StateType& _start, const StateType& _goal, string _dm,
    string _nf, string _vc, string _nc, string _gt, string _extenderLabel,
    vector<string> _evaluators, double _minDist,
    double _growthFocus, bool _evaluateGoal, size_t _numRoots,
    size_t _numDirections, size_t _maxTrial, double _goalDist) :
    KinodynamicRRTStrategy<MPTraits>(_start, _goal, _dm, _nf, _vc,
    _extenderLabel, _evaluators, _goalDist, _minDist, _growthFocus,
    _evaluateGoal) {

    this->SetName("KinoDynamicRegionRRT");
    m_reebGraphConstruction = new ReebGraphConstruction();
  }

template<class MPTraits>
KinoDynamicRegionRRT<MPTraits>::
KinoDynamicRegionRRT(MPProblemType* _problem, XMLNode& _node) :
  KinodynamicRRTStrategy<MPTraits>(_problem, _node),
  m_reebGraphConstruction(new ReebGraphConstruction(_node)) {
    this->SetName("KinoDynamicRegionRRT");
    ParseXML(_node);
  }

template<class MPTraits>
void
KinoDynamicRegionRRT<MPTraits>::
ParseXML(XMLNode& _node) {
  m_regionRadius = _node.Read("regionRadius", true, 3., 1., 4.,
      "Region radius multiplier");
}

template<class MPTraits>
void
KinoDynamicRegionRRT<MPTraits>::
Initialize() {
  KinodynamicRRTStrategy<MPTraits>::Initialize();

  StatClass* stats = this->GetStatClass();

  //Embed ReebGraph
  stats->StartClock("ReebGraphConstruction");
  m_reebGraphConstruction->Construct(this->GetEnvironment(),
      this->GetBaseFilename());
  stats->StopClock("ReebGraphConstruction");
}


template<class MPTraits>
void
KinoDynamicRegionRRT<MPTraits>::
Run() {
  if(this->m_debug)
    cout << "\nBegin KinoDynamicRegionRRT::Run" << endl;

  // Setup MP Variables
  StatClass* stats = this->GetStatClass();
  Environment* env = this->GetEnvironment();

  stats->StartClock("KinoDynamicRegionRRT");

  StateType s = this->m_query->GetQuery()[0];
  Vector3d start(s[0], s[1], s[2]);

  //Get directed flow network
  typedef ReebGraphConstruction::FlowGraph FlowGraph;
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
  // Make Temporary models for the regions
  map<RegionPtr, Model*> models;
  TempObjsModel tom;
#endif


  //Spark a region for each outgoing edge of start

  //Region structure stores tuple of flow edge descriptor,
  //index along flow edge, number of failed extentions
  unordered_map<RegionPtr, tuple<FED, size_t, size_t>> regions;

  double regionRadius = m_regionRadius *
      env->GetRobot(0)->GetBoundingSphereRadius();
  auto sit = flow.first.find_vertex(flow.second);
  for(auto eit = sit->begin(); eit != sit->end(); ++eit) {
    auto i = regions.emplace(
        RegionPtr(new BoundingSphere(start, regionRadius)),
        make_tuple(eit->descriptor(), 0, 0));
    m_regions.push_back(i.first->first);
  }

#ifdef VIZMO
  models[m_regions.back()] = new ThreadSafeSphereModel(
    m_regions.back()->GetCenter(), regionRadius);
  tom.AddOther(models[m_regions.back()]);
#endif

  visited[sit->descriptor()] = true;

  StateType dir;
  while(!this->EvaluateMap()) {
    //find my growth direction. Default is to randomly select node or bias
    //towards a goal
    if(this->m_query && DRand() < this->m_growthFocus &&
        !this->m_query->GetGoals().empty()) {
      dir = this->m_query->GetRandomGoal();
      if(this->m_debug)
        cout << "Goal Biased direction selected: " << dir << endl;
    }
    else {
      dir = SelectDirection();
      if(this->m_debug)
        cout << "Random Direction selected: " << dir << endl;
    }
    // Randomize Current Tree
    VID recent = this->ExpandTree(dir);
    if(recent != INVALID_VID) {

      StateRef newest = this->GetRoadmap()->GetGraph()->GetVertex(recent);

      if(m_samplingRegion) {
        get<2>(regions[m_samplingRegion]) = 0;

        while(env->InBounds(newest, m_samplingRegion)) {
          Vector3d cur = m_samplingRegion->GetCenter();

          auto& pr = regions[m_samplingRegion];
          FlowGraph::vertex_iterator vi;
          FlowGraph::adj_edge_iterator ei;
          flow.first.find_edge(get<0>(pr), vi, ei);
          vector<Vector3d>& path = ei->property();
          size_t& i = get<1>(pr);
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

    }
    else if(m_samplingRegion) {
      ++get<2>(regions[m_samplingRegion]);
      if(get<2>(regions[m_samplingRegion]) > 100) {
        auto rit = find(m_regions.begin(), m_regions.end(), m_samplingRegion);
        m_regions.erase(rit);
        regions.erase(m_samplingRegion);
      }
    }
#ifdef VIZMO
    GetVizmo().GetMap()->RefreshMap();
#endif
  }

  stats->StopClock("KinoDynamicRegionRRT");

  m_regions.clear();
  regions.clear();

  if(this->m_debug)
    cout<<"\nEnd KinoDynamicRegionRRT::Run" << endl;
}


template<class MPTraits>
void
KinoDynamicRegionRRT<MPTraits>::
Finalize() {
  KinodynamicRRTStrategy<MPTraits>::Finalize();
}

template<class MPTraits>
typename KinoDynamicRegionRRT<MPTraits>::StateType
KinoDynamicRegionRRT<MPTraits>::
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
    StateType mySample;
    mySample.GetRandomCfg(env,samplingBoundary);
    return mySample;
  }
  //catch Boundary too small exception
  catch(PMPLException _e) {
    StateType mySample;
    mySample.GetRandomCfg(env);
    return mySample;
  }
  //catch all others and exit
  catch(exception _e) {
    cerr << _e.what() << endl;
    exit(1);
  }
}

template <typename MPTraits>
void
KinoDynamicRegionRRT<MPTraits>::
PruneFlowGraph(FlowGraph& _f) const {
  using VD = FlowGraph::vertex_descriptor;

  // Find the flow-graph node nearest to the goal.
  const StateType goalCfg = this->m_query->GetQuery()[1];
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

#endif
