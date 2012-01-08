#ifndef Disconnect_h
#define Disconnect_h
#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "RoadmapGraph.h"


template <class CFG, class WEIGHT>
class Disconnect: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  //////////////////////
  // Constructors and Destructor
  Disconnect();
  Disconnect(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~Disconnect();
 
  //////////////////////
  // Access
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);  
  ///Used in new MPProblem framework.
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();

  //////////////////////
  // Core: Connection method
  void Connect();

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
             DistanceMetric *,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges);  

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
             DistanceMetric *,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<VID>& v1, vector<VID>& v2);

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
             DistanceMetric*,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<vector<VID> >& verticesList);


 private:
  //////////////////////
  // Data

};


template <class CFG, class WEIGHT>
Disconnect<CFG,WEIGHT>::Disconnect():NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "disconnect"; 
  SetDefault();
}

template <class CFG, class WEIGHT>
Disconnect<CFG,WEIGHT>::Disconnect(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem) { 
  LOG_DEBUG_MSG("Disconnect::Disconnect()"); 
  this->element_name = "disconnect"; 
  SetDefault();
  this->ParseXML(in_Node);
  
  
  LOG_DEBUG_MSG("~Disconnect::Disconnect()"); 
}





template <class CFG, class WEIGHT>
Disconnect<CFG,WEIGHT>::~Disconnect() { 
}


template <class CFG, class WEIGHT>
void Disconnect<CFG,WEIGHT>::SetDefault() {
}


template <class CFG, class WEIGHT>
void
Disconnect<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << this->GetName() << " ";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
Disconnect<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << this->GetName();
  _os << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
Disconnect<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new Disconnect<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
void Disconnect<CFG,WEIGHT>::
Connect() {
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl ;
  //cout << "DOING NOTHING" << endl;
}



template <class CFG, class WEIGHT>
void Disconnect<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;
  cout << "disconnect" << flush;
  
    vector<VID> vertices;
    RoadmapGraph< CFG, WEIGHT >* rGraph = _rm->m_pRoadmap;
    RoadmapGraph< CFG, WEIGHT > g = *(_rm->m_pRoadmap);
    cout<< "\nTo delete edges " <<endl;
    rGraph->GetVerticesVID(vertices);
    cout << "from " << vertices.size() << "vertices" <<endl;

/*
    for(int i=0; i<vertices.size(); ++i) {
      //rGraph->DeleteAllEdges(vertices[i]);
      cout<< "** DELETING EDGE from vertex "<< i << endl;
    }
*/
//to delete all edges, replace the DeleteAllEdges with either of following ways. fix_lantao
/*
    vector<typename Roadmap<CFG, WEIGHT>::edge_descriptor> eds;
    for(typename Roadmap<CFG, WEIGHT>::vertex_iterator vi = rGraph.begin(); vi!=rGraph.end(); vi++)
	for(typename Roadmap<CFG, WEIGHT>::adj_edge_iterator ei = vi.begin(); ei != vi.end(); ei++)
		eds.push_back(ei.descriptor());
    for(typename Roadmap<CFG, WEIGHT>::edge_iterator eitr= eds.begin(); eitr != eds.end(); eitr++) 	
 	rGraph->delete_edge(eitr.descriptor());
*/
    for(typename RoadmapGraph<CFG, WEIGHT>::edge_iterator ei = g.edges_begin(); ei != g.edges_end(); ++ei)
        rGraph->delete_edge(*ei.descriptor());

}


template <class CFG, class WEIGHT>
void Disconnect<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<vector<VID> >& verticesList)
{
  Connect(_rm, Stats, dm, lp, addPartialEdge, addAllEdges);
}


template <class CFG, class WEIGHT>
void Disconnect<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<VID>& v1, vector<VID>& v2) 
{
  Connect(_rm, Stats, dm, lp, addPartialEdge, addAllEdges);
}

#endif
