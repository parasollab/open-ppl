#ifndef REWIRE_CONNECTOR_H_
#define REWIRE_CONNECTOR_H_

#include "ConnectorMethod.h"
#include "Utilities/MedialAxisUtilities.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits>
class RewireConnector : public ConnectorMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;

    RewireConnector(string _nfLabel = "", string _lpLabel = "");
    RewireConnector(MPProblemType* _problem, XMLNode& _node);

    template<typename InputIterator1, typename InputIterator2,
      typename OutputIterator>
        void Connect(RoadmapType* _rm,
            InputIterator1 _itr1First, InputIterator1 _itr1Last,
            InputIterator2 _itr2First, InputIterator2 _itr2Last,
            bool _fromFullRoadmap,
            OutputIterator _collision);

  private:
    template<typename OutputIterator>
      void ConnectNeighbors(RoadmapType* _rm, VID _vid,
          vector<pair<VID, double> >& _closest, OutputIterator _collision);

    double GetDistance(VID _vid1, VID _vid2, RoadmapType* _rm);
    double GetShortestPath(VID _root, VID _vid, RoadmapType* _rm);

    bool m_distanceBased;
    ClearanceUtility<MPTraits> m_clearanceUtility;
};

template<class MPTraits>
RewireConnector<MPTraits>::
RewireConnector(string _nfLabel, string _lpLabel) :
  ConnectorMethod<MPTraits>(_nfLabel, _lpLabel) {
    this->SetName("RewireConnector");
  }

template<class MPTraits>
RewireConnector<MPTraits>::
RewireConnector(MPProblemType* _problem, XMLNode& _node) :
  ConnectorMethod<MPTraits>(_problem, _node) {
    this->SetName("RewireConnector");
    m_distanceBased = _node.Read("distanceBased", false, true,
        "Optimization criteria: True - path length; False - path clearance.");
    if(!m_distanceBased)
      m_clearanceUtility = ClearanceUtility<MPTraits>(_problem, _node);
  }

template<class MPTraits>
template<typename InputIterator1, typename InputIterator2, typename OutputIterator>
void
RewireConnector<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {


  NeighborhoodFinderPointer nfptr = this->GetNeighborhoodFinder(this->m_nfLabel);

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
    nfptr->FindNeighbors(_rm, _itr2First, _itr2Last, _fromFullRoadmap, vCfg,
        back_inserter(closest));

    if(this->m_debug){
      cout << "Neighbors | ";
      for(auto&  neighbor : closest)
        cout << neighbor.first << " ";
    }

    //test connections through LP
    ConnectNeighbors(_rm, vid, closest, _collision);
  }
}

template<class MPTraits>
template<typename OutputIterator>
void
RewireConnector<MPTraits>::
ConnectNeighbors(RoadmapType* _rm, VID _vid,
    vector<pair<VID, double> >& _closest, OutputIterator _collision) {
 #ifndef _PARALLEL // proper fix will be to call parallel dijkstra if there is one
  Environment* env = this->GetEnvironment();
  LPOutput<MPTraits> lpOutput, minlpOutput;

  typename GraphType::vertex_iterator vi = _rm->GetGraph()->find_vertex(_vid);
  typename GraphType::adj_edge_iterator ei = (*vi).begin();

  VID root = 0;
  VID parent = vi->property().GetStat("Parent");
  VID vmin = parent;    // initialize min to current vid

  double currentMin = GetShortestPath(root, _vid, _rm);
  double minWeight = 0;

  typedef typename vector<pair<VID, double> >::reverse_iterator RVIT;
  for(RVIT rvit = _closest.rbegin(); rvit!=_closest.rend(); rvit++) {
    VID neighbor = rvit->first;
    CfgType col;
    double neighborCost = GetShortestPath(root, neighbor, _rm);
    double neighborDistance = GetDistance(neighbor, _vid, _rm);
    if((m_distanceBased && neighborCost + neighborDistance < currentMin) ||
        (!m_distanceBased && min(neighborCost, neighborDistance) > currentMin)) {
      if(this->GetLocalPlanner(this->m_lpLabel)->
          IsConnected(vi->property(), _rm->GetGraph()->GetVertex(neighbor),
            col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes())) {
        vmin = neighbor;
        if(m_distanceBased)
          currentMin = neighborCost + neighborDistance;
        else
          currentMin = min(neighborCost, neighborDistance);
        minWeight = neighborDistance;
        minlpOutput = lpOutput;
      }
    }
  }

  // Found optimal path from neighbors
  if (vmin != parent) {
    if(!m_distanceBased){
      minlpOutput.m_edge.first.SetClearance(minWeight);
      minlpOutput.m_edge.second.SetClearance(minWeight);
    }
    _rm->GetGraph()->AddEdge(_vid, vmin, minlpOutput.m_edge);
    vi->property().SetStat("Parent", vmin);
    CfgRef cfg1 = _rm->GetGraph()->GetVertex(parent);
    CfgRef cfg2 = _rm->GetGraph()->GetVertex(_vid);
    VDRemoveEdge(cfg1, cfg2);     // for vizmo
    VDRemoveEdge(cfg2, cfg1);
    _rm->GetGraph()->delete_edge(parent, _vid);
    _rm->GetGraph()->delete_edge(_vid, parent);
  }
  if(this->m_debug)
    cout << "Connected to Optimal Neighbor" << endl;

  double vidCost = GetShortestPath(root, _vid, _rm);
  for(RVIT rvit = _closest.rbegin(); rvit!=_closest.rend(); rvit++) {

    //make sure not to not reroute to cause cycles
    VID v = _vid;
    bool cont = false;
    while(_rm->GetGraph()->GetVertex(v).IsStat("Parent")){
      v = _rm->GetGraph()->GetVertex(v).GetStat("Parent");
      if(rvit->first == v) {
        cont = true;
        break;
      }
    }
    //if(rvit->first == root)
    if(cont)
      continue;

    VID neighbor = rvit->first;
    CfgType col;
    double neighborCost = GetShortestPath(root, neighbor, _rm);
    double neighborDist = GetDistance(neighbor, _vid, _rm);
    if((m_distanceBased && vidCost + neighborDist < neighborCost) ||
        (!m_distanceBased && min(vidCost, neighborDist) > neighborCost)){
      bool connectable = this->GetLocalPlanner(this->m_lpLabel)->
          IsConnected(_rm->GetGraph()->GetVertex(_vid),
            _rm->GetGraph()->GetVertex(neighbor),
            col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes());
      this->AddConnectionAttempt(_vid, neighbor, connectable);
      if(connectable) {
        // Getting the parent
        vi = _rm->GetGraph()->find_vertex(neighbor);
        ei = (*vi).begin();
        parent = vi->property().GetStat("Parent");
        // Removing the parent-child edge
        CfgRef cfg1 = _rm->GetGraph()->GetVertex(parent);
        CfgRef cfg2 = _rm->GetGraph()->GetVertex(neighbor);
        _rm->GetGraph()->delete_edge(parent, neighbor);
        VDRemoveEdge(cfg1, cfg2);
        _rm->GetGraph()->delete_edge(neighbor, parent);
        VDRemoveEdge(cfg2, cfg1);
        // Add edge to optimal path to neighbor
        if(!m_distanceBased){
          lpOutput.m_edge.first.SetClearance(neighborDist);
          lpOutput.m_edge.second.SetClearance(neighborDist);
        }
        _rm->GetGraph()->AddEdge(_vid, neighbor, lpOutput.m_edge);
        vi->property().SetStat("Parent", _vid);
        if(this->m_debug)
          cout << "Added Neighbor Edge" << endl;
      }
    }
  }
  #endif
}

template<class MPTraits>
double
RewireConnector<MPTraits>::
GetShortestPath(VID _root, VID _vid, RoadmapType* _rm) {
  vector<VID> shortest;
  VID v = _vid;
  shortest.push_back(v);
  while(_rm->GetGraph()->GetVertex(v).IsStat("Parent")){
    v = _rm->GetGraph()->GetVertex(v).GetStat("Parent");
    shortest.push_back(v);
  }

  double totalWeight = m_distanceBased ? 0 : 1e6;

  if (shortest.size() > 0) {
    for (size_t i = 0; i < shortest.size() - 1; i++) {
      if(m_distanceBased)
        totalWeight += GetDistance(shortest[i], shortest[i+1], _rm);
      else
        totalWeight = min(totalWeight,
            GetDistance(shortest[i], shortest[i+1], _rm));
    }
  }
  return totalWeight;
}

template<class MPTraits>
double
RewireConnector<MPTraits>::
GetDistance(VID _vid1, VID _vid2, RoadmapType* _rm) {
  typedef typename RoadmapType::GraphType::EI EI;
  EI ei;
  if(!m_distanceBased && _rm->GetGraph()->IsEdge(_vid1, _vid2, ei) &&
      (*ei).property().HasClearance()) {
    //return weight for this edge based on clearance
    return (*ei).property().GetClearance();
  }
  else if(m_distanceBased) {
    //grab the individual Cfgs and calculate distance
    CfgRef cfg1 = _rm->GetGraph()->GetVertex(_vid1);
    CfgRef cfg2 = _rm->GetGraph()->GetVertex(_vid2);
    double distance = this->GetNeighborhoodFinder(this->m_nfLabel)->
      GetDMMethod()->Distance(cfg1, cfg2);
    return distance;
  }
  else {
    //calculate the clearance between them.
    CfgRef cfg1 = _rm->GetGraph()->GetVertex(_vid1);
    CfgRef cfg2 = _rm->GetGraph()->GetVertex(_vid2);
    WeightType w(this->m_lpLabel);
    return m_clearanceUtility.MinEdgeClearance(cfg1, cfg2, w);
  }
}

#endif
