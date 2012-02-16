#ifndef OPTIMALCONNECTOR_H_
#define OPTIMALCONNECTOR_H_

#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "NeighborhoodFinder.h"
#include "MetricUtils.h"
#include "GraphAlgo.h"
#include "MPStrategy.h"

template <class CFG, class WEIGHT>	
class OptimalConnection : public NodeConnectionMethod<CFG, WEIGHT> {
public:

	OptimalConnection(string _nf, string _lp, bool _radius); 
	OptimalConnection(XMLNodeReader& _node, MPProblem* _problem); 
	~OptimalConnection();
	
	typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  void SetDefault();
	virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);  
	void PrintOptions(ostream& _os); 
	virtual void ParseXML(XMLNodeReader& _node);
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();

  template <typename OutputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
        bool _addPartialEdge, bool _addAllEdges, OutputIterator _collision) ;

  template<typename InputIterator, typename OutputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
        bool _addPartialEdge, bool _addAllEdges,
        InputIterator _iterFirst, InputIterator _iterLast, OutputIterator _collision) ;
  
	template<typename InputIterator, typename OutputIterator>
	void ConnectNodes (Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, 
				bool _addPartialEdge, bool _addAllEdges,
        InputIterator _iter1First, InputIterator _iter1Last,
        InputIterator _iter2First, InputIterator _iter2Last, OutputIterator _collision);

  template <typename OutputIterator>
	void ConnectNeighbors (Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
				VID _vid, bool _addAllEdges, vector<VID>& _closest, OutputIterator _collision);

  template<typename InputIterator, typename OutputIterator>
	OutputIterator FindNeighbors(Roadmap<CFG, WEIGHT>* _rm, CFG _cfg, 
              InputIterator _iter2First, InputIterator _iter2Last, 
               OutputIterator _closestIterator);
private:
	string m_nf;
	string m_lp;
	bool m_radius;

};

template <class CFG, class WEIGHT>
OptimalConnection<CFG,WEIGHT>::OptimalConnection(string _nf, string _lp, bool _radius) : 
    NodeConnectionMethod<CFG,WEIGHT>(), 
    m_lp(_lp), m_radius(_radius), m_nf(_nf) {
	this->SetName("OptimalConnection");
}

template <class CFG, class WEIGHT>
OptimalConnection<CFG,WEIGHT>::OptimalConnection(XMLNodeReader& _node, MPProblem* _problem) : 
    NodeConnectionMethod<CFG,WEIGHT>(_node, _problem) {
	ParseXML(_node);
}

template <class CFG, class WEIGHT>
OptimalConnection<CFG,WEIGHT>::~OptimalConnection() { 
}


template <class CFG, class WEIGHT>
void OptimalConnection<CFG,WEIGHT>::SetDefault() {
	m_radius = false;
}

template <class CFG, class WEIGHT>
void OptimalConnection<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  PrintOptions(_os);
}

template <class CFG, class WEIGHT>
void OptimalConnection<CFG, WEIGHT>::
PrintValues(ostream& _os){
  PrintOptions(_os);
}


template <class CFG, class WEIGHT>
void OptimalConnection<CFG, WEIGHT>::
PrintOptions (ostream& _os) {
	_os << "OptimalConnection::PrintOptions" << endl;
	_os << "Neighborhood Finder: " << m_nf << endl;
	_os << "Local Planner: " << m_lp << endl;
	_os << "Radius-based or K-based: ";
	if (m_radius) 
		_os << "Radius" << endl << endl;
	else
		_os << "K-based" << endl << endl;
}

template <class CFG, class WEIGHT>
void OptimalConnection<CFG, WEIGHT>::
ParseXML (XMLNodeReader& _node) {  
	this->SetName("OptimalConnection");
	m_nf = _node.stringXMLParameter("nf", true, "", "NeighborhoodFinder");
  m_lp = _node.stringXMLParameter("lp_method", true, "", "Local Planner");
  m_radius = _node.boolXMLParameter("radius", true, false, "If true, use radius-based NF, otherwise use k-based NF"); 
}

template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
OptimalConnection<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = new OptimalConnection<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
template<typename OutputIterator>
void OptimalConnection<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, 
            bool _addPartialEdge, bool _addAllEdges, OutputIterator _collision) {

   vector<VID> vertices;
  _rm->m_pRoadmap->GetVerticesVID(vertices);
  
  ConnectNodes(_rm, _stats,
               _addPartialEdge, _addAllEdges, 
        vertices.begin(), vertices.end(),
        vertices.begin(), vertices.end(), _collision);
}


template <class CFG, class WEIGHT>
template<typename InputIterator, typename OutputIterator>
void OptimalConnection<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
            bool _addPartialEdge, bool _addAllEdges,
            InputIterator _iter1First, InputIterator _iter1Last, OutputIterator _collision) {
  
	vector<VID> vertices;
  _rm->m_pRoadmap->GetVerticesVID(vertices);
        
  ConnectNodes(_rm, _stats, _addPartialEdge, _addAllEdges, 
               _iter1First, _iter1Last,
               vertices.begin(), vertices.end(), _collision);
}

template <class CFG, class WEIGHT>
template<typename InputIterator, typename OutputIterator>
void OptimalConnection<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
						bool _addPartialEdge, bool _addAllEdges,
            InputIterator _iter1First, InputIterator _iter1Last,
            InputIterator _iter2First, InputIterator _iter2Last, OutputIterator _collision) {

	if (this->m_debug) { cout << endl; PrintOptions (cout); }


	// Step 1: Find neighbors
  // Step 2: Connect neighbors
	for (InputIterator iter1 = _iter1First; iter1 != _iter1Last; ++iter1) {
    CFG cfg = (*(_rm->m_pRoadmap->find_vertex(*iter1))).property();
		if (this->m_debug) {
			cout << "Attempting connection from " << *iter1 << "--> " << cfg << endl;
		}
		vector<VID>closest;
		back_insert_iterator< vector<VID> > iterBegin(closest);
		back_insert_iterator< vector<VID> > iterEnd = FindNeighbors(_rm, cfg, _iter2First, _iter2Last, iterBegin);
		ConnectNeighbors(_rm, _stats, *iter1, _addAllEdges, closest, _collision);
  }
}

template <class CFG, class WEIGHT>
template<typename OutputIterator>
void OptimalConnection<CFG,WEIGHT>::
ConnectNeighbors (Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
				VID _vid, bool _addAllEdges, vector<VID>& _closest, OutputIterator _collision) {

  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(m_nf)->GetDMMethod();
  LPOutput <CFG, WEIGHT> lpOutput;
  
  for (typename vector<VID>::iterator iter2 = _closest.begin(); iter2 != _closest.end(); ++iter2) {
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
    
    if(this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp)->
         IsConnected(_rm->GetEnvironment(), _stats, dm,
                     (*(_rm->m_pRoadmap->find_vertex(_vid))).property(),
                     (*(_rm->m_pRoadmap->find_vertex(*iter2))).property(),
                     col, &lpOutput, this->connectionPosRes, this->connectionOriRes,
                     (!_addAllEdges) ))			//#########################################
    {  
      _rm->m_pRoadmap->AddEdge(_vid, *iter2, lpOutput.edge);
      _rm->SetCache(_vid, *iter2, true);
      this->connection_attempts.push_back(make_pair(make_pair(_vid, *iter2), true));
			if (this->m_debug) {
				cout << "| Connection was successful" << endl;
			}
    }
    else {
      _rm->SetCache(_vid, *iter2, false);
      this->connection_attempts.push_back(make_pair(make_pair(_vid, *iter2), false));
			if (this->m_debug) {
				cout << "| Connection failed" << endl;
			}
    }

    if(col != CfgType())
      *_collision++ = col;	
	}

}

template <class CFG, class WEIGHT>
template <typename InputIterator, typename OutputIterator>
OutputIterator OptimalConnection<CFG, WEIGHT>::
FindNeighbors(Roadmap<CFG, WEIGHT>* _rm, CFG _cfg, 
              InputIterator _iter2First, InputIterator _iter2Last, 
               OutputIterator _closestIter) {
  // Step 1: Calculate 'k' or 'r' accordingly
  // Step 2: Call the NF and find neighbors  
  
  if (m_radius) {
		if (this->m_debug) {
			cout << "Finding closest neighbors within radius = " << endl; 
		}
    // Calculate radius
    // Call NF with radius obtained
    // return neighbors
  }
  else {
	
    int k = ( 2*2.71828 * log ( _rm->m_pRoadmap->get_num_vertices() ) ) + 1;  // Rounding up
    NeighborhoodFinder::NeighborhoodFinderPointer nfptr = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(m_nf);
		if (this->m_debug) {
			cout << "Finding closest neighbors with k = " << k << endl; 
		}
    return this->GetMPProblem()->GetNeighborhoodFinder()->KClosest(nfptr, _rm, _iter2First, _iter2Last, _cfg, k, _closestIter);
  }
}

#endif
