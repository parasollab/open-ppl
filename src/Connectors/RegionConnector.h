#ifndef REGION_CONNECT_H_
#define REGION_CONNECT_H_

#include "ConnectorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RegionConnector : public ConnectorMethod<MPTraits> {
  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;

    // Constructors
    RegionConnector(string _nfLabel = "", string _lpLabel = "", size_t _iters = 10);
    RegionConnector(MPProblemType* _problem, XMLNode& _node);

    // Utility Methods and Typedefs
    virtual void Print(ostream& _os) const;

    bool CheckEdge(VID _vid1, VID _vid2, RoadmapType* _rm);

    // Connection Methods
    template<typename InputIterator1, typename InputIterator2,
      typename OutputIterator>
        void Connect(RoadmapType* _rm,
            InputIterator1 _itr1First, InputIterator1 _itr1Last,
            InputIterator2 _itr2First, InputIterator2 _itr2Last,
            OutputIterator _collision);

  protected:
    template<typename OutputIterator>
      void ConnectNeighbors(RoadmapType* _rm, VID _vid,
          vector<pair<VID, double>>& _closest, OutputIterator _collision);

  private:
    size_t m_iters;
};

template<class MPTraits>
RegionConnector<MPTraits>::
RegionConnector(string _nfLabel, string _lpLabel, size_t _iters) :
  ConnectorMethod<MPTraits>(_nfLabel, _lpLabel), m_iters(_iters) {
    this->SetName("RegionConnector");
  }

template<class MPTraits>
RegionConnector<MPTraits>::
RegionConnector(MPProblemType* _problem, XMLNode& _node) :
  ConnectorMethod<MPTraits>(_problem, _node)  {
    this->SetName("RegionConnector");
    m_iters = _node.Read("numIters", true, 1, 0, MAX_INT,
        "number of times to execute the region connection");
  }

template<class MPTraits>
void
RegionConnector<MPTraits>::
Print(ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
}

// 1. A random node from region 1 is chosen
// 2. the K-closest nodes from region 2 are obtained
// 3. the K-closest nodes of nodes found in step 2 from region 1 are obtained
// 4. Connections are attempted from those two groups
template<class MPTraits>
template<typename InputIterator1, typename InputIterator2, typename OutputIterator>
void
RegionConnector<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    OutputIterator _collision) {

  for(size_t iter = 0; iter < m_iters; iter++) {
    // 1. A random node from region 1 is chosen
    InputIterator1 randNode =  _itr1First;

    long rand = LRand() % abs(_itr1Last - _itr1First);

    std::advance(randNode, rand);

    // 2. the K-closest nodes from region 2 are obtained
    vector<pair<VID, double>> closestRegion2;
    CfgType cfg = _rm->GetGraph()->GetVertex(randNode);

    NeighborhoodFinderPointer nfptr = this->GetNeighborhoodFinder(this->m_nfLabel);
    nfptr->FindNeighbors(_rm, _itr2First, _itr2Last, cfg, back_inserter(closestRegion2));

    // 3. the closest node from region 1 of nodes found in step 2 are obtained
    typedef typename vector<pair<VID, double>>::iterator VIT;
    for(VIT vit = closestRegion2.begin(); vit != closestRegion2.end(); ++vit) {
      CfgType cfg = _rm->GetGraph()->GetVertex(vit->first);
      vector<pair<VID, double>> closest;

      nfptr->FindNeighbors(_rm, _itr1First, _itr1Last, cfg, back_inserter(closest));

      // 4. Connections are attempted from those two groups
      this->ConnectNeighbors(_rm, vit->first, closest, _collision);
    }
  }
}

// Will connect the neighbors contained in the vector with the current node
template<class MPTraits>
template<typename OutputIterator>
void
RegionConnector<MPTraits>::
ConnectNeighbors(RoadmapType* _rm, VID _vid,
    vector<pair<VID, double>>& _closest, OutputIterator _collision) {

  Environment* env = this->GetEnvironment();
  LocalPlannerPointer lp = this->GetLocalPlanner(this->m_lpLabel);
  LPOutput <MPTraits> lpOutput;

  typedef typename vector<pair<VID, double>>::iterator VIT;
  for(VIT vit = _closest.begin(); vit != _closest.end(); ++vit) {
    // Stopping Conditions
    if(!CheckEdge(_vid, vit->first, _rm))
      continue;

    CfgType col;
    if(lp->IsConnected(_rm->GetGraph()->GetVertex(_vid),
          _rm->GetGraph()->GetVertex(vit->first),
          col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes())) {

      _rm->GetGraph()->AddEdge(_vid, vit->first, lpOutput.m_edge);

      this->AddConnectionAttempt(_vid, vit->first, true);

      if (this->m_debug)
        cout << "| Connection was successful" << endl;
    }
    else {
      this->AddConnectionAttempt(_vid, vit->first, false);
      if (this->m_debug)
        cout << "| Connection failed" << endl;
    }

    if(col != CfgType())
      *_collision++ = col;
  }
}

template<class MPTraits>
bool
RegionConnector<MPTraits>::CheckEdge(VID _vid1, VID _vid2, RoadmapType* _rm) {

  if (_vid2 == INVALID_VID) {
    if (this->m_debug) cout << "Skipping... Invalid node" << endl;
    return false;
  }
  if (_vid1 == _vid2) {
    if (this->m_debug) cout << "Skipping... Same nodes" << endl;
    return false;  // don't attempt between the same node
  }
  if(this->IsCached(_vid1, _vid2)) {
    if (this->m_debug) cout << "Skipping... Already attempted connection" << endl;
    return false;  // don't attempt if already exists
  }
  /*
     if ( _rm->GetGraph()->IsEdge(_vid1, _vid2) ) {
     if (this->m_debug) cout << "Skipping... Edge already exists" << endl;
     return false;
     }
     */
  return true;
}
#endif
