#ifndef PMPL_REWIRE_CONNECTOR_H_
#define PMPL_REWIRE_CONNECTOR_H_

#include "ConnectorMethod.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"

////////////////////////////////////////////////////////////////////////////////
/// Generates 're-wiring' connections for optimal planners like RRT*.
///
/// @todo Add paper reference.
///
/// @todo Clean up and validate. This has not been exercised in a long time.
///       Replace clearance utility and 'distance-based' option with a single
///       distance metric label (which may evaluate anything).
///
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits>
class RewireConnector : public ConnectorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::WeightType        WeightType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename RoadmapType::VID            VID;

    ///@}
    ///@name Construction
    ///@{

    RewireConnector();

    RewireConnector(XMLNode& _node);

    virtual ~RewireConnector() = default;

    ///@}
    ///@name Connector Interface
    ///@{

    template <typename InputIterator1, typename InputIterator2,
              typename OutputIterator>
    void Connect(RoadmapType* _rm,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision);


    template <typename InputIterator1, typename InputIterator2,
              typename OutputIterator>
    void Connect(GroupRoadmapType* _rm,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision);

    ///@}

  private:

    ///@name Helpers
    ///@{

    template<typename OutputIterator>
    void ConnectNeighbors(RoadmapType* _rm, VID _vid,
        vector<Neighbor>& _closest, OutputIterator _collision);

    double GetDistance(VID _vid1, VID _vid2, RoadmapType* _rm);
    double GetShortestPath(VID _root, VID _vid, RoadmapType* _rm);

    ///@}
    ///@name Internal State
    ///@{

    bool m_distanceBased;
    ClearanceUtility<MPTraits> m_clearanceUtility;

    ///@}

};

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
RewireConnector<MPTraits>::
RewireConnector() {
  this->SetName("RewireConnector");
}

template <typename MPTraits>
RewireConnector<MPTraits>::
RewireConnector(XMLNode& _node) : ConnectorMethod<MPTraits>(_node) {
  this->SetName("RewireConnector");
  m_distanceBased = _node.Read("distanceBased", false, true,
      "Optimization criteria: True - path length; False - path clearance.");
  if(!m_distanceBased)
    m_clearanceUtility = ClearanceUtility<MPTraits>(_node);
}

/*--------------------------- Connector Interface ----------------------------*/

template <typename MPTraits>
template<typename InputIterator1, typename InputIterator2, typename OutputIterator>
void
RewireConnector<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  std::vector<Neighbor> closest;

  // Try to generate rewire connections eminating from the first range.
  for(auto iter = _itr1First; iter != _itr1Last; ++iter) {
    const VID source = _rm->GetVID(iter);
    const CfgType& cfg = _rm->GetVertex(iter);

    if(this->m_debug)
      std::cout << std::distance(_itr1First, iter)
                << "\tAttempting connections: VID = "
                << source << "  --> Cfg = " << cfg.PrettyPrint()
                << std::endl;

    // Find nearest neighbors in the second range.
    closest.clear();
    nf->FindNeighbors(_rm, _itr2First, _itr2Last, _fromFullRoadmap, cfg,
        std::back_inserter(closest));

    // Attempt to connect source to the discovered neighbors.
    ConnectNeighbors(_rm, source, closest, _collision);
  }
}


template <typename MPTraits>
template <typename InputIterator1, typename InputIterator2,
          typename OutputIterator>
void
RewireConnector<MPTraits>::
Connect(GroupRoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {
  throw NotImplementedException(WHERE);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
template<typename OutputIterator>
void
RewireConnector<MPTraits>::
ConnectNeighbors(RoadmapType* _rm, VID _vid,
    vector<Neighbor>& _closest, OutputIterator _collision) {
  Environment* env = this->GetEnvironment();
  LPOutput<MPTraits> lpOutput, minlpOutput;

  typename RoadmapType::vertex_iterator vi = _rm->find_vertex(_vid);
  typename RoadmapType::adj_edge_iterator ei = (*vi).begin();

  VID root = 0;
  VID parent = vi->property().GetStat("Parent");
  VID vmin = parent;    // initialize min to current vid

  double currentMin = GetShortestPath(root, _vid, _rm);
  double minWeight = 0;

  for(auto rvit = _closest.rbegin(); rvit!=_closest.rend(); rvit++) {
    VID neighbor = rvit->target;
    CfgType col(this->GetTask()->GetRobot());
    double neighborCost = GetShortestPath(root, neighbor, _rm);
    double neighborDistance = GetDistance(neighbor, _vid, _rm);
    if((m_distanceBased && neighborCost + neighborDistance < currentMin) ||
        (!m_distanceBased && min(neighborCost, neighborDistance) > currentMin)) {
      if(this->GetLocalPlanner(this->m_lpLabel)->
          IsConnected(vi->property(), _rm->GetVertex(neighbor),
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
    _rm->AddEdge(_vid, vmin, minlpOutput.m_edge);
    vi->property().SetStat("Parent", vmin);
    CfgType& cfg1 = _rm->GetVertex(parent);
    CfgType& cfg2 = _rm->GetVertex(_vid);
    VDRemoveEdge(cfg1, cfg2);     // for vizmo
    VDRemoveEdge(cfg2, cfg1);
    _rm->DeleteEdge(parent, _vid);
    _rm->DeleteEdge(_vid, parent);
  }
  if(this->m_debug)
    cout << "Connected to Optimal Neighbor" << endl;

  double vidCost = GetShortestPath(root, _vid, _rm);
  for(auto rvit = _closest.rbegin(); rvit!=_closest.rend(); rvit++) {

    //make sure not to not reroute to cause cycles
    VID v = _vid;
    bool cont = false;
    while(_rm->GetVertex(v).IsStat("Parent")){
      v = _rm->GetVertex(v).GetStat("Parent");
      if(rvit->target == v) {
        cont = true;
        break;
      }
    }
    //if(rvit->target == root)
    if(cont)
      continue;

    VID neighbor = rvit->target;
    CfgType col(this->GetTask()->GetRobot());
    double neighborCost = GetShortestPath(root, neighbor, _rm);
    double neighborDist = GetDistance(neighbor, _vid, _rm);
    if((m_distanceBased && vidCost + neighborDist < neighborCost) ||
        (!m_distanceBased && min(vidCost, neighborDist) > neighborCost)){
      bool connectable = this->GetLocalPlanner(this->m_lpLabel)->
          IsConnected(_rm->GetVertex(_vid),
            _rm->GetVertex(neighbor),
            col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes());
      if(connectable) {
        // Getting the parent
        vi = _rm->find_vertex(neighbor);
        ei = (*vi).begin();
        parent = vi->property().GetStat("Parent");
        // Removing the parent-child edge
        CfgType& cfg1 = _rm->GetVertex(parent);
        CfgType& cfg2 = _rm->GetVertex(neighbor);
        _rm->DeleteEdge(parent, neighbor);
        VDRemoveEdge(cfg1, cfg2);
        _rm->DeleteEdge(neighbor, parent);
        VDRemoveEdge(cfg2, cfg1);
        // Add edge to optimal path to neighbor
        if(!m_distanceBased){
          lpOutput.m_edge.first.SetClearance(neighborDist);
          lpOutput.m_edge.second.SetClearance(neighborDist);
        }
        _rm->AddEdge(_vid, neighbor, lpOutput.m_edge);
        vi->property().SetStat("Parent", _vid);
        if(this->m_debug)
          cout << "Added Neighbor Edge" << endl;
      }
      else
        this->CacheFailedConnection(_vid, neighbor);
    }
  }
}


template <typename MPTraits>
double
RewireConnector<MPTraits>::
GetShortestPath(VID _root, VID _vid, RoadmapType* _rm) {
  vector<VID> shortest;
  VID v = _vid;
  shortest.push_back(v);
  while(_rm->GetVertex(v).IsStat("Parent")){
    v = _rm->GetVertex(v).GetStat("Parent");
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


template <typename MPTraits>
double
RewireConnector<MPTraits>::
GetDistance(VID _vid1, VID _vid2, RoadmapType* _rm) {
  typedef typename RoadmapType::EI EI;
  EI ei;
  if(!m_distanceBased && _rm->GetEdge(_vid1, _vid2, ei) &&
      (*ei).property().HasClearance()) {
    //return weight for this edge based on clearance
    return (*ei).property().GetClearance();
  }
  else if(m_distanceBased) {
    //grab the individual Cfgs and calculate distance
    CfgType& cfg1 = _rm->GetVertex(_vid1);
    CfgType& cfg2 = _rm->GetVertex(_vid2);
    auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
    auto dm = this->GetDistanceMetric(nf->GetDMLabel());
    double distance = dm->Distance(cfg1, cfg2);
    return distance;
  }
  else {
    //calculate the clearance between them.
    CfgType& cfg1 = _rm->GetVertex(_vid1);
    CfgType& cfg2 = _rm->GetVertex(_vid2);
    WeightType w(this->m_lpLabel);
    vector<double> clearanceVec = m_clearanceUtility.EdgeClearance(cfg1, cfg2, w);
    return *min_element(clearanceVec.begin(), clearanceVec.end());
  }
}

/*----------------------------------------------------------------------------*/

#endif
