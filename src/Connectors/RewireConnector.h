#ifndef REWIRECONNECTOR_H_
#define REWIRECONNECTOR_H_

#include "ConnectorMethod.h"

template<typename MPTraits>
class RewireConnector : public ConnectorMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;

    RewireConnector(string _nfLabel = "", string _lpLabel = "");
    RewireConnector(MPProblemType* _problem, XMLNodeReader& _node);

    template<typename ColorMap, typename InputIterator1, typename InputIterator2, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& cmap,
          InputIterator1 _itr1First, InputIterator1 _itr1Last,
          InputIterator2 _itr2First, InputIterator2 _itr2Last, OutputIterator _collision);

  protected:
    template<typename OutputIterator>
      void ConnectNeighbors (RoadmapType* _rm, StatClass& _stats,
          VID _vid, vector<pair<VID, double> >& _closest, OutputIterator _collision);

    double GetDistance(VID _vid1, VID _vid2, RoadmapType* _rm);
    double GetShortestPath(VID _root, VID _vid, RoadmapType* _rm);
};

template<class MPTraits>
RewireConnector<MPTraits>::RewireConnector(string _nfLabel, string _lpLabel)
  : ConnectorMethod<MPTraits>(_nfLabel, _lpLabel) {
    this->SetName("RewireConnector");
  }

template<class MPTraits>
RewireConnector<MPTraits>::RewireConnector(MPProblemType* _problem, XMLNodeReader& _node)
  : ConnectorMethod<MPTraits>(_problem, _node) {
    this->SetName("RewireConnector");
  }

template<class MPTraits>
template<typename ColorMap, typename InputIterator1, typename InputIterator2, typename OutputIterator>
void
RewireConnector<MPTraits>::Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    OutputIterator _collision) {


  if(this->m_debug){
    cout << endl;
    this->PrintOptions(cout);
  }

  NeighborhoodFinderPointer nfptr = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel);

  // the vertices in this iteration are the source for the connection operation
  for(InputIterator1 itr1 = _itr1First; itr1 != _itr1Last; ++itr1){

    // find cfg pointed to by itr1
    VID vid = _rm->GetGraph()->GetVID(itr1);
    CfgRef vCfg = _rm->GetGraph()->GetVertex(itr1);

    if(this->m_debug)
      cout << (itr1 - _itr1First)
        << "\tAttempting connections: VID = "
        << vid << "  --> Cfg = " << vCfg << endl;

    //determine nearest neighbors
    vector<pair<VID, double> > closest;
    nfptr->FindNeighbors(_rm, _itr2First, _itr2Last, vCfg, back_inserter(closest));

    if(this->m_debug){
      cout << "Neighbors | ";
      for(typename vector<pair<VID, double> >::iterator nit = closest.begin(); nit!=closest.end(); ++nit)
        cout << nit->first << " ";
    }

    //test connections through LP
    ConnectNeighbors(_rm, _stats, vid, closest, _collision);
  }
}

template<class MPTraits>
template<typename OutputIterator>
void
RewireConnector<MPTraits>::ConnectNeighbors(RoadmapType* _rm, StatClass& _stats,
    VID _vid, vector<pair<VID, double> >& _closest, OutputIterator _collision) {

  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel)->GetDMMethod();
  LPOutput<MPTraits> lpOutput, minlpOutput;

  typename GraphType::vertex_iterator vi = _rm->GetGraph()->find_vertex(_vid);
  typename GraphType::adj_edge_iterator ei = (*vi).begin();

  VID root = 0;
  VID parent = vi->property().GetStat("Parent");
  VID vmin = parent;    // initialize min to current vid

  double currentMin = GetShortestPath(root, _vid, _rm);
  for(size_t i = 0; i < _closest.size(); i++) {
    VID neighbor = _closest[i].first;
    CfgType col;
    double neighborCost = GetShortestPath(root, neighbor, _rm);
    double neighborDistance = _closest[i].second;
    if(neighborCost + neighborDistance < currentMin) {
      bool connectable = this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel)->
        IsConnected(env, _stats, dm,
            vi->property(),
            _rm->GetGraph()->GetVertex(neighbor),
            col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes(), true);
      this->AddConnectionAttempt(vi->descriptor(), neighbor, connectable);
      if(connectable) {
        vmin = neighbor;
        currentMin = neighborCost + neighborDistance;
        minlpOutput = lpOutput;
      }
    }
  }

  // Found optimal path from neighbors
  if(vmin != parent) {
    _rm->GetGraph()->AddEdge(_vid, vmin, minlpOutput.m_edge);
    vi->property().SetStat("Parent", vmin);
    CfgRef cfg1 = _rm->GetGraph()->GetVertex(parent);
    CfgRef cfg2 = _rm->GetGraph()->GetVertex(_vid);
    _rm->GetGraph()->delete_edge(parent, _vid);
    _rm->GetGraph()->delete_edge(_vid, parent);
    VDRemoveEdge(cfg1, cfg2);     // for vizmo
    VDRemoveEdge(cfg2, cfg1);
  }
  if(this->m_debug) cout << "Connected to Optimal Neighbor" << endl;

  double vidCost = GetShortestPath(root, _vid, _rm);
  for(size_t i = 0; i < _closest.size(); i++) {
    VID neighbor = _closest[i].first;
    CfgType col;
    double neighborCost = GetShortestPath(root, neighbor, _rm);
    if(vidCost + _closest[i].second < neighborCost) {
      bool connectable = this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel)->
        IsConnected(env, _stats, dm,
            _rm->GetGraph()->GetVertex(_vid),
            _rm->GetGraph()->GetVertex(neighbor),
            col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes(), true);
      this->AddConnectionAttempt(_vid, neighbor, connectable);
      if(connectable) {
        // Getting the parent
        vi = _rm->GetGraph()->find_vertex(neighbor);
        ei = (*vi).begin();
        parent = ((*ei).target());
        if(vi->property().GetStat("Parent")!=parent) {
          parent = vi->property().GetStat("Parent");
        }
        // Removing the parent-child edge
        CfgRef cfg1 = _rm->GetGraph()->GetVertex(parent);
        CfgRef cfg2 = _rm->GetGraph()->GetVertex(neighbor);
        _rm->GetGraph()->delete_edge(parent, neighbor);
        VDRemoveEdge(cfg1, cfg2);
        _rm->GetGraph()->delete_edge(neighbor, parent);
        VDRemoveEdge(cfg2, cfg1);
        // Add edge to optimal path to neighbor
        _rm->GetGraph()->AddEdge(_vid, neighbor, lpOutput.m_edge);
        vi->property().SetStat("Parent", _vid);
        if(this->m_debug) cout << "Added Neighbor Edge" << endl;
      }
    }
  }
}

template<class MPTraits>
double
RewireConnector<MPTraits>::GetShortestPath(VID _root, VID _vid, RoadmapType* _rm) {
  vector<VID> shortest;
  stapl::sequential::find_path_dijkstra(*(_rm->GetGraph()), _root, _vid, shortest, GraphType::edge_property::MaxWeight());
  double totalWeight = 0;
  if(shortest.size() > 0) {
    for(size_t i = 0; i < shortest.size() - 1; i++) {
      totalWeight += GetDistance(shortest[i], shortest[i+1], _rm);
    }
  }
  return totalWeight;
}

template<class MPTraits>
double
RewireConnector<MPTraits>::GetDistance(VID _vid1, VID _vid2, RoadmapType* _rm) {
  CfgRef cfg1 = _rm->GetGraph()->GetVertex(_vid1);
  CfgRef cfg2 = _rm->GetGraph()->GetVertex(_vid2);
  double distance = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel)->GetDMMethod()->
    Distance(cfg1, cfg2);
  return distance;
}

#endif
