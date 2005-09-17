#ifndef Disconnect_h
#define Disconnect_h
#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "GraphAlgo.h"


template <class CFG, class WEIGHT>
class Disconnect: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  Disconnect();
  Disconnect(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  ~Disconnect();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods
  void ParseCommandLine(std::istringstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);  
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os);  
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();
  virtual void ParseXML(TiXmlNode* in_pNode);

  //////////////////////
  // Core: Connection method
  void Connect();

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
             CollisionDetection*, DistanceMetric *,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges);  

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
             CollisionDetection*, DistanceMetric *,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<VID>& v1, vector<VID>& v2);

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
             CollisionDetection*, DistanceMetric*,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<vector<VID> >& verticesList);


 private:
  //////////////////////
  // Data

};


template <class CFG, class WEIGHT>
Disconnect<CFG,WEIGHT>::Disconnect():NodeConnectionMethod<CFG,WEIGHT>() { 
  element_name = "disconnect"; 
  SetDefault();
}

template <class CFG, class WEIGHT>
Disconnect<CFG,WEIGHT>::Disconnect(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_pNode, in_pProblem) { 
  LOG_DEBUG_MSG("Disconnect::Disconnect()"); 
  element_name = "disconnect"; 
  SetDefault();
  ParseXML(in_pNode);
  
  
  LOG_DEBUG_MSG("~Disconnect::Disconnect()"); 
}





template <class CFG, class WEIGHT>
Disconnect<CFG,WEIGHT>::~Disconnect() { 
}


template <class CFG, class WEIGHT>
void Disconnect<CFG,WEIGHT>::ParseXML(TiXmlNode* in_pNode) { 
  
 
}


template <class CFG, class WEIGHT>
void Disconnect<CFG,WEIGHT>::
ParseCommandLine(std::istringstream& is) {
}


template <class CFG, class WEIGHT>
void Disconnect<CFG,WEIGHT>::SetDefault() {
}


template <class CFG, class WEIGHT>
void
Disconnect<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
Disconnect<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName();
  _os << endl;
}

template <class CFG, class WEIGHT>
void
Disconnect<CFG, WEIGHT>::
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() ;
  out_os << endl;
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
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;
#ifndef QUIET
  cout << "disconnect" << flush;
#endif
  
    vector<VID> vertices;
    RoadmapGraph< CFG, WEIGHT >* rGraph = _rm->m_pRoadmap;
    cout<< "\nTo delete edges " <<endl;
    rGraph->GetVerticesVID(vertices);
    cout << "from " << vertices.size() << "vertices" <<endl;
    for(int i=0; i<vertices.size(); ++i) {
      rGraph->DeleteAllEdges(vertices[i]);
      cout<< "** DELETING EDGE from vertex "<< i << endl;
    }
}


template <class CFG, class WEIGHT>
void Disconnect<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<vector<VID> >& verticesList)
{
  Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges);
}


template <class CFG, class WEIGHT>
void Disconnect<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<VID>& v1, vector<VID>& v2) 
{
  Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges);
}

#endif
