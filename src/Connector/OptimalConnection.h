#ifndef OPTIMALCONNECTOR_H_
#define OPTIMALCONNECTOR_H_

#include "ConnectionMethod.h"

template <class CFG, class WEIGHT>	
class OptimalConnection : public ConnectionMethod<CFG, WEIGHT> {
  public:

    // Constructors
    OptimalConnection(string _nf = "", bool _radius = true); 
    OptimalConnection(XMLNodeReader& _node, MPProblem* _problem); 
    ~OptimalConnection();

    // Utility Methods and Typedefs
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    typedef typename vector<typename RoadmapGraph<CFG,WEIGHT>::VID>::iterator VIDIT;
    virtual void PrintOptions(ostream& _os); 
    virtual void ParseXML(XMLNodeReader& _node);

    // Connection Methods
    void Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats);

    template <typename OutputIterator>
      void Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
          OutputIterator _collision) ;

    template<typename InputIterator, typename OutputIterator>
      void Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
          InputIterator _iterFirst, InputIterator _iterLast, OutputIterator _collision) ;

    template<typename InputIterator, typename OutputIterator>
      void Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, 
          InputIterator _iter1First, InputIterator _iter1Last,
          InputIterator _iter2First, InputIterator _iter2Last, OutputIterator _collision);

  protected:
    template <typename OutputIterator>
      void ConnectNeighbors(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
          VID _vid, vector<VID>& _closest, OutputIterator _collision);

    template<typename InputIterator, typename OutputIterator>
      void FindNeighbors( Roadmap<CFG, WEIGHT>* _rm, CFG _cfg, 
          InputIterator _iter2First, InputIterator _iter2Last, 
          OutputIterator _closestIterator);
  private:
    bool m_radius; /* will determine which type of NF to use. In case is true, 
                      a radius based NF will be used to find the neighbors */
};

  template <class CFG, class WEIGHT>
OptimalConnection<CFG,WEIGHT>::OptimalConnection(string _nf, bool _radius)
  : ConnectionMethod<CFG,WEIGHT>(), m_radius(_radius) {
    this->m_nfMethod = _nf;
    this->SetName("OptimalConnection");
    if (m_radius) {
      if(this->m_debug) cout << "Error, radius-based feature not available, defaulting to k-based"<<endl;
      m_radius = false; 
    }
  }

  template <class CFG, class WEIGHT>
OptimalConnection<CFG,WEIGHT>::OptimalConnection(XMLNodeReader& _node, MPProblem* _problem)
  : ConnectionMethod<CFG,WEIGHT>(_node, _problem) {
    this->SetName("OptimalConnection");
    ParseXML(_node);
  }

template <class CFG, class WEIGHT>
OptimalConnection<CFG,WEIGHT>::~OptimalConnection() { 
}

template <class CFG, class WEIGHT>
void 
OptimalConnection<CFG, WEIGHT>::PrintOptions (ostream& _os) {
  ConnectionMethod<CFG,WEIGHT>::PrintOptions(_os);
  _os << "OptimalConnection::PrintOptions" << endl;
  _os << "Neighborhood Finder: " << this->m_nfMethod << endl;
  _os << "Local Planner: " << this->m_lpMethod << endl;
  _os << "Radius-based or K-based: ";
  if (m_radius) 
    _os << "Radius" << endl << endl;
  else
    _os << "K-based" << endl << endl;
}

template <class CFG, class WEIGHT>
void 
OptimalConnection<CFG, WEIGHT>::ParseXML (XMLNodeReader& _node) {  
  m_radius = _node.boolXMLParameter("radius", true, false, "If true, use radius-based NF, otherwise use k-based NF"); 
  if (m_radius) {
    if(this->m_debug) cout << "Error, radius-based feature not available, defaulting to k-based"<<endl;
    m_radius = false; 
  }
}

template <class CFG, class WEIGHT>
void OptimalConnection<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats){
  vector<CFG> collision;
  Connect(_rm, _stats, back_inserter(collision));
}

template <class CFG, class WEIGHT>
template<typename OutputIterator>
void 
OptimalConnection<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, 
    OutputIterator _collision) {
  vector<VID> vertices;
  _rm->m_pRoadmap->GetVerticesVID(vertices);
  Connect(_rm, _stats, vertices.begin(), vertices.end(), vertices.begin(), vertices.end(), _collision);
}

template <class CFG, class WEIGHT>
template<typename InputIterator, typename OutputIterator>
void 
OptimalConnection<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    InputIterator _iter1First, InputIterator _iter1Last, 
    OutputIterator _collision) {
  vector<VID> vertices;
  _rm->m_pRoadmap->GetVerticesVID(vertices);
  Connect(_rm, _stats, _iter1First, _iter1Last, vertices.begin(), vertices.end(), _collision);
}


// Will iterate through the map and find each nodes closest neighbors 
// and call ConnectNeighbors to connect them
template <class CFG, class WEIGHT>
template<typename InputIterator, typename OutputIterator>
void 
OptimalConnection<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    InputIterator _iter1First, InputIterator _iter1Last,
    InputIterator _iter2First, InputIterator _iter2Last, 
    OutputIterator _collision) {

  if (this->m_debug) { cout << endl; PrintOptions (cout); }

  for (InputIterator iter1 = _iter1First; iter1 != _iter1Last; ++iter1) {
    CFG cfg = pmpl_detail::GetCfg<InputIterator>(_rm->m_pRoadmap)(iter1);
    if (this->m_debug) {
      cout << "Attempting connection from " << *iter1 << "--> " << cfg << endl;
    }
    vector<VID> closest;
    back_insert_iterator< vector<VID> > iterBegin(closest);
    FindNeighbors(_rm, cfg, _iter2First, _iter2Last, iterBegin);
    ConnectNeighbors(_rm, _stats, *iter1, closest, _collision);
  }
}

// Will connect the neighbors contained in the vector with the current node 
template <class CFG, class WEIGHT>
template<typename OutputIterator>
void 
OptimalConnection<CFG,WEIGHT>::ConnectNeighbors ( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    VID _vid, 
    vector<VID>& _closest, 
    OutputIterator _collision) {

  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod)->GetDMMethod();
  LPOutput <CFG, WEIGHT> lpOutput;

  for (VIDIT iter2 = _closest.begin(); iter2 != _closest.end(); ++iter2) {
    // Stopping Conditions
    if (*iter2 == INVALID_VID) { 
      if (this->m_debug) {
        cout << "Skipping... Invalid node" << endl;
      }
      continue;
    }
    if (_vid == *iter2) {
      if (this->m_debug) {
        cout << "Skipping... Same nodes" << endl;
      }
      continue;  // don't attempt between the same node
    }
    if (_rm->IsCached(_vid, *iter2)) {
      if ( !_rm->GetCache(_vid, *iter2) )
        if (this->m_debug) {
          cout << "Skipping... Already attempted connection" << endl;
        }
      continue;  // don't attempt if already exists
    }
    if ( _rm->m_pRoadmap->IsEdge(_vid, *iter2) ) {
      if (this->m_debug) {
        cout << "Skipping... Edge already exists" << endl;
      }
      continue;
    }
    CfgType col;

    if(this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpMethod)->
        IsConnected(_rm->GetEnvironment(), _stats, dm,
          pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(_vid),
          pmpl_detail::GetCfg<VIDIT>(_rm->m_pRoadmap)(iter2),
          col, &lpOutput, this->m_connectionPosRes, this->m_connectionOriRes, (!this->m_addAllEdges) )) {  

      _rm->m_pRoadmap->AddEdge(_vid, *iter2, lpOutput.edge);

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
template <class CFG, class WEIGHT>
template <typename InputIterator, typename OutputIterator>
void 
OptimalConnection<CFG, WEIGHT>::FindNeighbors(Roadmap<CFG, WEIGHT>* _rm, CFG _cfg, 
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

    int k = (int)ceil( 2*2.71828 * log ( _rm->m_pRoadmap->get_num_vertices() ) ) ;  // Rounding up
    NeighborhoodFinder::NeighborhoodFinderPointer nfptr = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod);
    if (this->m_debug) {
      cout << "Finding closest neighbors with k = " << k << endl; 
    }
    this->GetMPProblem()->GetNeighborhoodFinder()->KClosest(nfptr, _rm, _iter2First, _iter2Last, _cfg, k, _closestIter);
  }
}

#endif
