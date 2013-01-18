#ifndef OPTIMALCONNECTOR_H_
#define OPTIMALCONNECTOR_H_

#include "ConnectorMethod.h"

template <class MPTraits>	
class OptimalConnection : public ConnectorMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename vector<VID>::iterator VIDIT;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;

    // Constructors
    OptimalConnection(MPProblemType* _problem = NULL, string _lp = "", string _nf = "", bool _radius = true);
    OptimalConnection(MPProblemType* _problem, XMLNodeReader& _node); 
    ~OptimalConnection();

    // Utility Methods 
    virtual void PrintOptions(ostream& _os); 
    virtual void ParseXML(XMLNodeReader& _node);
    bool CheckEdge(VID _vid1, VID _vid2, RoadmapType* _rm);

    // Connection Methods
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect( RoadmapType* _rm, StatClass& _stats, 
          ColorMap& _cmap, InputIterator _iter1First, InputIterator _iter1Last,
          InputIterator _iter2First, InputIterator _iter2Last, OutputIterator _collision);

  protected:
    template <typename OutputIterator>
      void ConnectNeighbors(RoadmapType* _rm, StatClass& _stats,
          VID _vid, vector<VID>& _closest, OutputIterator _collision);

    template<typename InputIterator, typename OutputIterator>
      void FindNeighbors( RoadmapType* _rm, CfgType _cfg, 
          InputIterator _iter2First, InputIterator _iter2Last, 
          OutputIterator _closestIterator);
  protected:
    bool m_radius; /* will determine which type of NF to use. In case is true, 
                      a radius based NF will be used to find the neighbors */
};

  template <class MPTraits>
  OptimalConnection<MPTraits>::OptimalConnection( MPProblemType* _problem, string _lp, string _nf, bool _radius)
  : ConnectorMethod<MPTraits>(), m_radius(_radius) {
    this->SetName("OptimalConnection");
    this->m_lpMethod = _lp;
    this->m_nfMethod = _nf;
    this->SetMPProblem(_problem);
    if (m_radius) {
      if(this->m_debug) cout << "Error, radius-based feature not available, defaulting to k-based"<<endl;
      m_radius = false; 
    }
  }

  template <class MPTraits>
  OptimalConnection<MPTraits>::OptimalConnection(MPProblemType* _problem, XMLNodeReader& _node)
  : ConnectorMethod<MPTraits>(_problem, _node) {
    this->SetName("OptimalConnection");
    ParseXML(_node);
  }

template <class MPTraits>
OptimalConnection<MPTraits>::~OptimalConnection() { 
}

template <class MPTraits>
void 
OptimalConnection<MPTraits>::PrintOptions (ostream& _os) {
  ConnectorMethod<MPTraits>::PrintOptions(_os);
  _os << "OptimalConnection::PrintOptions" << endl;
  _os << "Neighborhood Finder: " << this->m_nfMethod << endl;
  _os << "Local Planner: " << this->m_lpMethod << endl;
  _os << "Radius-based or K-based: ";
  if (m_radius) 
    _os << "Radius" << endl << endl;
  else
    _os << "K-based" << endl << endl;
}

template <class MPTraits>
void 
OptimalConnection<MPTraits>::ParseXML (XMLNodeReader& _node) {  
  m_radius = _node.boolXMLParameter("radius", true, false, "If true, use radius-based NF, otherwise use k-based NF"); 
  if (m_radius) {
    if(this->m_debug) cout << "Error, radius-based feature not available, defaulting to k-based"<<endl;
    m_radius = false; 
  }
}

// Will iterate through the map and find each nodes closest neighbors 
// and call ConnectNeighbors to connect them
template <class MPTraits>
template<typename ColorMap, typename InputIterator, typename OutputIterator>
void 
OptimalConnection<MPTraits>::Connect( RoadmapType* _rm, StatClass& _stats,
    ColorMap& _cmap, InputIterator _iter1First, InputIterator _iter1Last,
    InputIterator _iter2First, InputIterator _iter2Last, OutputIterator _collision) {

  if (this->m_debug) { cout << endl; PrintOptions (cout); }
  ///To do - uncomment after const vertex iter problem  in STAPL pGraph is fixed
  #ifndef _PARALLEL
  for (InputIterator iter1 = _iter1First; iter1 != _iter1Last; ++iter1) {
    CfgType cfg = _rm->GetGraph()->GetCfg(*iter1);
    if (this->m_debug) {
      cout << "Attempting connection from " << *iter1 << "--> " << cfg << endl;
    }
    vector<VID> closest;
    back_insert_iterator< vector<VID> > iterBegin(closest);
    FindNeighbors(_rm, cfg, _iter2First, _iter2Last, iterBegin); 
    this->ConnectNeighbors(_rm, _stats, *iter1, closest, _collision);
  }
  #else 
  stapl_assert(false,"Optimal Connection using const VIT");
  #endif
}

// Will connect the neighbors contained in the vector with the current node 
template <class MPTraits>
template<typename OutputIterator>
void 
OptimalConnection<MPTraits>::ConnectNeighbors(RoadmapType* _rm, StatClass& _stats,
    VID _vid, vector<VID>& _closest, OutputIterator _collision) {

  
  LPOutput<MPTraits> lpOutput;
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();
  for (VIDIT iter2 = _closest.begin(); iter2 != _closest.end(); ++iter2) {
    // Stopping Conditions
    if ( !CheckEdge(_vid, *iter2, _rm) )
      continue;
    CfgType col;
    if(this->GetMPProblem()->GetLocalPlanner(this->m_lpMethod)->
        IsConnected(this->GetMPProblem()->GetEnvironment(), _stats, dm,
          _rm->GetGraph()->GetCfg(_vid),
          _rm->GetGraph()->GetCfg(*iter2),
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


// Find the neighbors of current node. There are two cases:
// If it is radius based, find neighbors within that area
// If it is k based, find the k-closest neighbor
template <class MPTraits>
template <typename InputIterator, typename OutputIterator>
void 
OptimalConnection<MPTraits>::FindNeighbors(RoadmapType* _rm, CfgType _cfg, 
    InputIterator _iter2First, InputIterator _iter2Last, OutputIterator _closestIter) {
  // Waiting for radius based NF to complete function
  if (m_radius) {
    if (this->m_debug) {
      cout << "Finding closest neighbors within radius = " << endl; 
    }
    // Calculate radius
    // Call NF with radius obtained
    // return neighbors
  }
  else {

    int k = (int)ceil( 2*2.71828 * log ( _rm->GetGraph()->get_num_vertices() ) ) ;  // Rounding up
    NeighborhoodFinderPointer nfptr = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod);
    if (this->m_debug) {
      cout << "Finding closest neighbors with k = " << k << endl; 
    }
    nfptr->KClosest(_rm, _iter2First, _iter2Last, _cfg, k, _closestIter);
  }
}


template <class MPTraits>
bool
OptimalConnection<MPTraits>::CheckEdge(VID _vid1, VID _vid2, RoadmapType* _rm) {
    
    if (_vid2 == INVALID_VID) { 
      if (this->m_debug) cout << "Skipping... Invalid node" << endl;
      return false;
    }
    if (_vid1 == _vid2) {
      if (this->m_debug) cout << "Skipping... Same nodes" << endl;
      return false;  // don't attempt between the same node
    }
    if (_rm->IsCached(_vid1, _vid2)) {
      return false;  // don't attempt if already exists
    }
    if ( _rm->GetGraph()->IsEdge(_vid1, _vid2) ) {
      if (this->m_debug) cout << "Skipping... Edge already exists" << endl;
      return false;
    }
    return true;
}
#endif
