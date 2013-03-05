#ifndef REGIONCONNECT_H_
#define REGIONCONNECT_H_

#include "ConnectorMethod.h"

#define K_CLOSEST 5

template<class MPTraits>
class RegionConnector : public ConnectorMethod<MPTraits> {
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID; 
  public:

    // Constructors
    RegionConnector(string _nf = "", int _k = K_CLOSEST); 
    RegionConnector(MPProblemType* _problem, XMLNodeReader& _node); 
    ~RegionConnector();

    // Utility Methods and Typedefs
    typedef typename vector<VID>::iterator VIDIT;
    virtual void PrintOptions(ostream& _os); 
    virtual void ParseXML(XMLNodeReader& _node);
    bool CheckEdge(VID _vid1, VID _vid2, RoadmapType* _rm);

    // Connection Methods
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect( RoadmapType* _rm, StatClass& _stats, 
          ColorMap& _cmap, InputIterator _region1First, InputIterator _region1Last,
          InputIterator _region2First, InputIterator _region2Last, OutputIterator _collision);

  protected:
    template <typename OutputIterator>
      void ConnectNeighbors(RoadmapType* _rm, StatClass& _stats,
          VID _vid, vector<VID>& _closest, OutputIterator _collision);

    template<typename InputIterator, typename OutputIterator>
      void FindNeighbors( RoadmapType* _rm, CfgType _cfg, 
          InputIterator _region2First, InputIterator _region2Last, 
          OutputIterator _closestIterator);

  private:
  int m_k;
  int m_iters;
  /* Cannot get shared pointer of DM from NF getting it separately */
  string m_dmLabel;

};

template<class MPTraits>
RegionConnector<MPTraits>::RegionConnector(string _nf, int _k)
  : ConnectorMethod<MPTraits>() {
    this->m_nfMethod = _nf;
    this->m_k = _k;
    this->SetName("RegionConnector");
  }

template<class MPTraits>
RegionConnector<MPTraits>::RegionConnector(MPProblemType* _problem, XMLNodeReader& _node)
  : ConnectorMethod<MPTraits>(_problem, _node)  {
    this->SetName("RegionConnector");
    ParseXML(_node);
  }

template<class MPTraits>
RegionConnector<MPTraits>::~RegionConnector() { 
}

template<class MPTraits>
void 
RegionConnector<MPTraits>::PrintOptions (ostream& _os) {
  ConnectorMethod<MPTraits>::PrintOptions(_os);
  _os << "RegionConnector::PrintOptions" << endl;
  _os << "K:  " << this->m_k << endl;
  _os << "Neighborhood Finder: " << this->m_nfMethod << endl;
  //nOriRes = _problem->GetEnvironment()->GetOrientationRes();
  _os << "Local Planner: " << this->m_lpMethod << endl;
}

template<class MPTraits>
void 
RegionConnector<MPTraits>::ParseXML (XMLNodeReader& _node) {  
  m_k = _node.numberXMLParameter("k", true, 0, 0, 10000, "k-value (max neighbors to find). k = 0 --> all-pairs");
  this->m_dmLabel = _node.stringXMLParameter("dmLabel", true, "", "Distance Metric Label");
  m_iters = _node.numberXMLParameter("numIters", true, 1, 0, 10000, "number of times to execute the region connection");
}

// 1. A random node from region 1 is chosen
// 2. the K-closest nodes from region 2 are obtained
// 3. the K-closest nodes of nodes found in step 2 from region 1 are obtained
// 4. Connections are attempted from those two groups
template<class MPTraits>
template<typename ColorMap, typename InputIterator, typename OutputIterator>
void 
RegionConnector<MPTraits>::Connect( RoadmapType* _rm, StatClass& _stats,
    ColorMap& _cmap, InputIterator _region1First, InputIterator _region1Last,
    InputIterator _region2First, InputIterator _region2Last, OutputIterator _collision) {

  if (this->m_debug) { cout << endl; PrintOptions (cout); }
  ///To do - uncomment after const vertex iter problem  in STAPL pGraph is fixed
  //#ifndef _PARALLEL
  for (size_t iter = 0; iter<m_iters; iter++) { 
    // 1. A random node from region 1 is chosen
    InputIterator randNode =  _region1First;
    InputIterator last1 = _region1Last;
    
    long rand = LRand() % abs(_region1Last - _region1First);
     
    std::advance(randNode, rand);

    // 2. the K-closest nodes from region 2 are obtained
    vector<VID> closestRegion2;
    back_insert_iterator< vector<VID> > iterRegion2(closestRegion2);
    CfgType cfg = _rm->GetGraph()->GetCfg(randNode);
    FindNeighbors(_rm, cfg, _region2First, _region2Last, iterRegion2);
     // 3. the closest node from region 1 of nodes found in step 2 are obtained
    for (InputIterator iter2 = closestRegion2.begin(); iter2 != closestRegion2.end(); iter2++) {
      CfgType cfg = _rm->GetGraph()->GetCfg(iter2);
      vector<VID> closest;
      back_insert_iterator< vector<VID> > iterBegin(closest);
      FindNeighbors(_rm, cfg, _region1First, _region1Last, iterBegin); 

      // 4. Connections are attempted from those two groups
      this->ConnectNeighbors(_rm, _stats, *iter2, closest, _collision);
      
    }
  }
  //#else 
  //stapl_assert(false,"Optimal Connection using const VIT");
  //#endif
}

// Will connect the neighbors contained in the vector with the current node 
template<class MPTraits>
template<typename OutputIterator>
void 
RegionConnector<MPTraits>::ConnectNeighbors(RoadmapType* _rm, StatClass& _stats,
    VID _vid, vector<VID>& _closest, OutputIterator _collision) {

  Environment* env = this->GetMPProblem()->GetEnvironment();
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(this->m_lpMethod);
  
  LPOutput <MPTraits> lpOutput;
  // shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod)->GetDMMethod();
  MPProblemType* problem = this->GetMPProblem(); 
  DistanceMetricPointer dm = problem->GetDistanceMetric(m_dmLabel);

  for (VIDIT iter2 = _closest.begin(); iter2 != _closest.end(); ++iter2) {
    // Stopping Conditions
    if ( !CheckEdge(_vid, *iter2, _rm) )
      continue;
    CfgType col;

    if (lp->IsConnected(env, _stats, dm,
          _rm->GetGraph()->GetCfg(_vid),
          _rm->GetGraph()->GetCfg(iter2),
          col, &lpOutput, this->m_connectionPosRes, this->m_connectionOriRes, (!this->m_addAllEdges) )) {  

      _rm->GetGraph()->AddEdge(_vid, *iter2, lpOutput.edge);

      _rm->SetCache(_vid, *iter2, true);

      //this->connection_attempts.push_back(make_pair(make_pair(_vid, *iter2), true));
      if (this->m_debug) {
        cout << "| Connection was successful" << endl;
      }
    }
    else {
      _rm->SetCache(_vid, *iter2, false);
      //this->connection_attempts.push_back(make_pair(make_pair(_vid, *iter2), false));
      if (this->m_debug) {
        cout << "| Connection failed" << endl;
      }
    }

    if(col != CfgType())
      *_collision++ = col;	
  }

}


template<class MPTraits>
template <typename InputIterator, typename OutputIterator>
void 
RegionConnector<MPTraits>::FindNeighbors(RoadmapType* _rm, CfgType _cfg, 
    InputIterator _region2First, InputIterator _region2Last, OutputIterator _closestIter) {

    NeighborhoodFinderPointer nfptr = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod);
    if (this->m_debug) 
      cout << "Finding closest neighbors with k = " << m_k << endl; 
    nfptr->KClosest(_rm, _region2First, _region2Last, _cfg, m_k, _closestIter);
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
    if (_rm->IsCached(_vid1, _vid2)) {
      //TODO Cesar verify if this is safe if ( !_rm->GetCache(_vid1, _vid2) )
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
