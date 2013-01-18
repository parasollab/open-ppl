#ifndef OPTIMALREWIRE_H_
#define OPTIMALREWIRE_H_

#include "OptimalConnection.h"

template <typename MPTraits>	
class OptimalRewire : public OptimalConnection<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    OptimalRewire(MPProblemType* _problem=NULL, string _lp = "", string _nf = "", bool _radius = false, string _dm = ""); 
    OptimalRewire(MPProblemType* _problem, XMLNodeReader& _node); 
    ~OptimalRewire() {}

    virtual void PrintOptions(ostream& _os); 

    template <typename OutputIterator>
      void ConnectNeighbors (RoadmapType* _rm, StatClass& _stats,
          VID _vid, vector<VID>& _closest, OutputIterator _collision);

    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect( RoadmapType* _rm, StatClass& _stats, ColorMap& cmap,
          InputIterator _iter1First, InputIterator _iter1Last,
          InputIterator _iter2First, InputIterator _iter2Last, OutputIterator _collision);

    double GetShortestPath(VID _root, VID _vid, RoadmapType* _rm);
    double GetDistance(VID _vid1, VID _vid2, RoadmapType* _rm);
};

template <class MPTraits>
OptimalRewire<MPTraits>::OptimalRewire(MPProblemType* _problem, string _lp, string _nf, bool _radius, string _dm)
  : OptimalConnection<MPTraits>(_problem, _lp,_nf,_radius) {
    this->SetName("OptimalRewire");
    this->m_lpMethod = _lp;
    this->m_nfMethod = _nf;
    this->SetMPProblem(_problem);
    if (this->m_radius) {
      cout << "Error, radius-based feature not available, defaulting to k-based"<<endl;
      this->m_radius = false; 
    }
  }

  template <class MPTraits>
OptimalRewire<MPTraits>::OptimalRewire(MPProblemType* _problem, XMLNodeReader& _node) 
  : OptimalConnection<MPTraits>(_problem, _node) {
    this->SetName("OptimalRewire");
  }

template <class MPTraits>
void 
OptimalRewire<MPTraits>::PrintOptions (ostream& _os) {
  OptimalConnection<MPTraits>::PrintOptions(_os);
  _os << "OptimalRewire::PrintOptions" << endl;
}

template <class MPTraits>
double
OptimalRewire<MPTraits>::GetShortestPath(VID _root, VID _vid, RoadmapType* _rm) {
  vector<VID> shortest;
  stapl::sequential::find_path_dijkstra(*(_rm->GetGraph()), _root, _vid, shortest, GraphType::edge_property::MaxWeight()); 
  double totalWeight = 0;

  if (shortest.size() > 0) {
    for (size_t i = 0; i < shortest.size() - 1; i++) { 
      totalWeight += GetDistance(shortest[i], shortest[i+1], _rm);
    }
  }
  return totalWeight; 
}

template <class MPTraits>
double
OptimalRewire<MPTraits>::GetDistance(VID _vid1, VID _vid2, RoadmapType* _rm) {

  
  CfgType cfg1 = _rm->GetGraph()->GetCfg(_vid1);
  CfgType cfg2 = _rm->GetGraph()->GetCfg(_vid2);
  double distance = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod()->
    Distance(this->GetMPProblem()->GetEnvironment(), cfg1, cfg2);
  return distance;
}

template <class MPTraits>
template<typename ColorMap, typename InputIterator, typename OutputIterator>
void 
OptimalRewire<MPTraits>::Connect( RoadmapType* _rm, StatClass& _stats,
    ColorMap& cmap,
    InputIterator _iter1First, InputIterator _iter1Last,
    InputIterator _iter2First, InputIterator _iter2Last, 
    OutputIterator _collision) {

  if (this->m_debug) { cout << endl; this->PrintOptions (cout); }
  ///To do - uncomment after const vertex iter problem  in STAPL pGraph is fixed
#ifndef _PARALLEL
  for (InputIterator iter1 = _iter1First; iter1 != _iter1Last; ++iter1) {
    CfgType cfg = _rm->GetGraph()->GetCfg(*iter1);
    if (this->m_debug) {
      cout << "Attempting connection from " << *iter1 << "--> " << cfg << endl;
    }
    vector<VID> closest;
    back_insert_iterator< vector<VID> > iterBegin(closest);
    this->FindNeighbors(_rm, cfg, _iter2First, _iter2Last, iterBegin); 
    this->ConnectNeighbors(_rm, _stats, *iter1, closest, _collision);
  }
#else 
  stapl_assert(false,"Optimal Rewire using const VIT");
#endif
}

template <class MPTraits>
template<typename OutputIterator>
void 
OptimalRewire<MPTraits>::ConnectNeighbors (RoadmapType* _rm, StatClass& _stats,
    VID _vid, vector<VID>& _closest, OutputIterator _collision) {
 
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();
  LPOutput<MPTraits> lpOutput, minlpOutput;
  typename GraphType::vertex_iterator vi = _rm->GetGraph()->find_vertex(_vid);
  typename GraphType::adj_edge_iterator ei = (*vi).begin();
  VID root = 0;
  VID parent = vi->property().GetStat("Parent");
  VID vmin = parent;    // initialize min to current vid

  double currentMin = GetShortestPath(root, _vid, _rm);
  for (size_t i = 0; i < _closest.size(); i++) {
    VID neighbor = _closest[i];
    CfgType col;
    double neighborCost = GetShortestPath(root, neighbor, _rm);
    double neighborDistance = GetDistance(neighbor, _vid, _rm); 
    if( ( neighborCost + neighborDistance) < currentMin ) {
      if(this->GetMPProblem()->GetLocalPlanner(this->m_lpMethod)->
          IsConnected(this->GetMPProblem()->GetEnvironment(), _stats, dm,
            vi->property(),
            _rm->GetGraph()->GetCfg(neighbor),
            col, &lpOutput, this->m_connectionPosRes, this->m_connectionOriRes, true )) {
        vmin = neighbor;
        currentMin = neighborCost + GetDistance(_vid, neighbor, _rm);
        minlpOutput = lpOutput;
      }
    }
  }
  // Found optimal path from neighbors
  if (vmin != parent) {
    _rm->GetGraph()->AddEdge(_vid, vmin, minlpOutput.edge);
    vi->property().SetStat("Parent", vmin);
    CfgType cfg1 = _rm->GetGraph()->GetCfg(parent);
    CfgType cfg2 = _rm->GetGraph()->GetCfg(_vid);
    _rm->GetGraph()->delete_edge(parent, _vid);
    _rm->GetGraph()->delete_edge(_vid, parent);
    VDRemoveEdge(cfg1, cfg2);     // for vizmo
    VDRemoveEdge(cfg2, cfg1);
  }
  _rm->SetCache(_vid, vmin, true);
  this->m_connectionAttempts.push_back(make_pair(make_pair(_vid, vmin), true));
  if (this->m_debug) cout << "Connected to Optimal Neighbor" << endl;

  double vidCost = GetShortestPath(root, _vid, _rm);
  for (size_t i = 0; i < _closest.size(); i++) {
    VID neighbor = _closest[i];
    CfgType col;
    double neighborCost = GetShortestPath(root, neighbor, _rm);
    if( ( vidCost + GetDistance(neighbor, _vid, _rm)) < neighborCost ) { 
      if(this->GetMPProblem()->GetLocalPlanner(this->m_lpMethod)->
          IsConnected(this->GetMPProblem()->GetEnvironment(), _stats, dm,
            _rm->GetGraph()->GetCfg(_vid),
            _rm->GetGraph()->GetCfg(neighbor),
            col, &lpOutput, this->m_connectionPosRes, this->m_connectionOriRes, true )) {
        // Getting the parent
        vi = _rm->GetGraph()->find_vertex(neighbor);
        ei = (*vi).begin();
        parent = ((*ei).target());
        if(vi->property().GetStat("Parent")!=parent) {
          parent = vi->property().GetStat("Parent");
        }
        // Removing the parent-child edge
        CfgType cfg1 = _rm->GetGraph()->GetCfg(parent);
        CfgType cfg2 = _rm->GetGraph()->GetCfg(neighbor);
        _rm->GetGraph()->delete_edge(parent, neighbor);
        VDRemoveEdge(cfg1, cfg2);
        _rm->GetGraph()->delete_edge(neighbor, parent);
        VDRemoveEdge(cfg2, cfg1);
        // Add edge to optimal path to neighbor
        _rm->GetGraph()->AddEdge(_vid, neighbor, lpOutput.edge);
        vi->property().SetStat("Parent", _vid);
        _rm->SetCache(_vid, neighbor, true);
        this->m_connectionAttempts.push_back(make_pair(make_pair(_vid, neighbor), true));
        if (this->m_debug) cout << "Added Neighbor Edge" << endl;
      }
    }
  }
}

#endif
