#ifndef OPTIMALREWIRE_H_
#define OPTIMALREWIRE_H_

#include "OptimalConnection.h"

template <typename CFG, typename WEIGHT>	
class OptimalRewire : public OptimalConnection<CFG, WEIGHT> {
  public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    OptimalRewire(string _nf = "", bool _radius = false, string _dm = ""); 
    OptimalRewire(XMLNodeReader& _node, MPProblem* _problem); 
    ~OptimalRewire() {}

    virtual void PrintOptions(ostream& _os); 

    template <typename OutputIterator>
      void ConnectNeighbors (Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
          VID _vid, vector<VID>& _closest, OutputIterator _collision);

    template<typename InputIterator, typename OutputIterator, typename ColorMap>
      void Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, ColorMap& cmap,
          InputIterator _iter1First, InputIterator _iter1Last,
          InputIterator _iter2First, InputIterator _iter2Last, OutputIterator _collision);

    double GetShortestPath(VID _root, VID _vid, Roadmap<CFG, WEIGHT>* _rm);
    double GetDistance(VID _vid1, VID _vid2, Roadmap<CFG, WEIGHT>* _rm);
  private:
    string m_dm;
};

template <typename CFG, typename WEIGHT>
OptimalRewire<CFG,WEIGHT>::OptimalRewire(string _nf, bool _radius, string _dm) : 
  OptimalConnection<CFG,WEIGHT>(_nf, _radius) {
    this->SetName("OptimalRewire");
    m_dm = _dm;
    if (this->m_radius) {
      cout << "Error, radius-based feature not available, defaulting to k-based"<<endl;
      this->m_radius = false; 
    }
  }

  template <typename CFG, typename WEIGHT>
OptimalRewire<CFG,WEIGHT>::OptimalRewire(XMLNodeReader& _node, MPProblem* _problem) 
  : OptimalConnection<CFG,WEIGHT>(_node, _problem) {
    this->SetName("OptimalRewire");
    m_dm = _node.stringXMLParameter("dm", true, "", "Distance Metric Label"); 
  }

template <typename CFG, typename WEIGHT>
void 
OptimalRewire<CFG, WEIGHT>::PrintOptions (ostream& _os) {
  OptimalConnection<CFG,WEIGHT>::PrintOptions(_os);
  _os << "OptimalRewire::PrintOptions" << endl;
  _os << "DistanceMetric::" << m_dm << endl << endl;
}

template <typename CFG, typename WEIGHT>
double
OptimalRewire<CFG, WEIGHT>::GetShortestPath(VID _root, VID _vid, Roadmap<CFG, WEIGHT>* _rm) {
  vector<VID> shortest;
  stapl::sequential::find_path_dijkstra(*(_rm->m_pRoadmap), _root, _vid, shortest, WEIGHT::MaxWeight()); 
  double totalWeight = 0;

  if (shortest.size() > 0) {
    for (size_t i = 0; i < shortest.size() - 1; i++) { 
      totalWeight += GetDistance(shortest[i], shortest[i+1], _rm);
    }
  }
  return totalWeight; 
}

template <typename CFG, typename WEIGHT>
double
OptimalRewire<CFG, WEIGHT>::GetDistance(VID _vid1, VID _vid2, Roadmap<CFG, WEIGHT>* _rm) {

  CFG cfg1 = pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(_vid1);
  CFG cfg2 = pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(_vid2);

  double distance = this->GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm)->
    Distance(this->GetMPProblem()->GetEnvironment(), cfg1, cfg2);
  return distance;
}

template <typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator, typename ColorMap>
void 
OptimalRewire<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    ColorMap& cmap,
    InputIterator _iter1First, InputIterator _iter1Last,
    InputIterator _iter2First, InputIterator _iter2Last, 
    OutputIterator _collision) {

  if (this->m_debug) { cout << endl; this->PrintOptions (cout); }
  ///To do - uncomment after const vertex iter problem  in STAPL pGraph is fixed
#ifndef _PARALLEL
  for (InputIterator iter1 = _iter1First; iter1 != _iter1Last; ++iter1) {
    CFG cfg = pmpl_detail::GetCfg<InputIterator>(_rm->m_pRoadmap)(iter1);
    if (this->m_debug) {
      cout << "Attempting connection from " << *iter1 << "--> " << cfg << endl;
    }
    vector<VID> closest;
    back_insert_iterator< vector<VID> > iterBegin(closest);
    FindNeighbors(_rm, cfg, _iter2First, _iter2Last, iterBegin); 
    this->ConnectNeighbors(_rm, _stats, *iter1, closest, _collision);
  }
#else 
  stapl_assert(false,"Optimal Rewire using const VIT");
#endif
}

template <typename CFG, typename WEIGHT>
template<typename OutputIterator>
void 
OptimalRewire<CFG,WEIGHT>::ConnectNeighbors (Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    VID _vid, vector<VID>& _closest, OutputIterator _collision) {
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  LPOutput <CFG, WEIGHT> lpOutput, minlpOutput;
  typename RoadmapGraph<CFG, WEIGHT>::vertex_iterator vi = _rm->m_pRoadmap->find_vertex(_vid);
  typename RoadmapGraph<CFG, WEIGHT>::adj_edge_iterator ei = (*vi).begin();
  VID root = 0;
  VID parent = vi->property().GetStat("Parent");
  VID vmin = parent;    // initialize min to current vid

  double currentMin = GetShortestPath(root, _vid, _rm);
  for (size_t i = 0; i < _closest.size(); i++) {
    VID neighbor = _closest[i];
    CFG col;
    double neighborCost = GetShortestPath(root, neighbor, _rm);
    double neighborDistance = GetDistance(neighbor, _vid, _rm); 
    if( ( neighborCost + neighborDistance) < currentMin ) {
      if(this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpMethod)->
          IsConnected(_rm->GetEnvironment(), _stats, dm,
            vi->property(),
            pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(neighbor),
            col, &lpOutput, this->m_connectionPosRes, this->m_connectionOriRes, true )) {
        vmin = neighbor;
        currentMin = neighborCost + GetDistance(_vid, neighbor, _rm);
        minlpOutput = lpOutput;
      }
    }
  }
  // Found optimal path from neighbors
  if (vmin != parent) {
    _rm->m_pRoadmap->AddEdge(_vid, vmin, minlpOutput.edge);
    vi->property().SetStat("Parent", vmin);
    CFG cfg1 = pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(parent);
    CFG cfg2 = pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(_vid);
    _rm->m_pRoadmap->delete_edge(parent, _vid);
    _rm->m_pRoadmap->delete_edge(_vid, parent);
    VDRemoveEdge(cfg1, cfg2);     // for vizmo
    VDRemoveEdge(cfg2, cfg1);
  }
  _rm->SetCache(_vid, vmin, true);
  this->m_connectionAttempts.push_back(make_pair(make_pair(_vid, vmin), true));
  if (this->m_debug) cout << "Connected to Optimal Neighbor" << endl;

  double vidCost = GetShortestPath(root, _vid, _rm);
  for (size_t i = 0; i < _closest.size(); i++) {
    VID neighbor = _closest[i];
    CFG col;
    double neighborCost = GetShortestPath(root, neighbor, _rm);
    if( ( vidCost + GetDistance(neighbor, _vid, _rm)) < neighborCost ) {
      if(this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpMethod)->
          IsConnected(_rm->GetEnvironment(), _stats, dm,
            pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(_vid),
            pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(neighbor),
            col, &lpOutput, this->m_connectionPosRes, this->m_connectionOriRes, true )) {
        // Getting the parent
        vi = _rm->m_pRoadmap->find_vertex(neighbor);
        ei = (*vi).begin();
        parent = ((*ei).target());
        if(vi->property().GetStat("Parent")!=parent) {
          parent = vi->property().GetStat("Parent");
        }
        // Removing the parent-child edge
        CFG cfg1 = pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(parent);
        CFG cfg2 = pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(neighbor);
        _rm->m_pRoadmap->delete_edge(parent, neighbor);
        VDRemoveEdge(cfg1, cfg2);
        _rm->m_pRoadmap->delete_edge(neighbor, parent);
        VDRemoveEdge(cfg2, cfg1);
        // Add edge to optimal path to neighbor
        _rm->m_pRoadmap->AddEdge(_vid, neighbor, lpOutput.edge);
        vi->property().SetStat("Parent", _vid);
        _rm->SetCache(_vid, neighbor, true);
        this->m_connectionAttempts.push_back(make_pair(make_pair(_vid, neighbor), true));
        if (this->m_debug) cout << "Added Neighbor Edge" << endl;
      }
    }
  }
}

#endif
