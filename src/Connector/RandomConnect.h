#ifndef RandomConnect_h
#define RandomConnect_h
#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "GraphAlgo.h"


// ------------------------------------------------------------------
// Connect nodes in a random way.
// ------------------------------------------------------------------
   /**Connect nodes in map Totally Randomly.
    *Following Algorithm is used:
    *   -# loop following CN::GetNumEdges times
    *       -# node1 = random node in map, _rm
    *       -# node2 = random node in map, _rm
    *       -# lp_set is a random local planner set
    *       -# using local planning functions in lp_set
    *          to connect node1 and node2
    *       -# if connected, add this edge to map, _rm.
    *
    *@param info provides inforamtion other than connection, like
    *collision dection, local planner, and distance metrics.
    *@param _cn provides information for specific node connection 
    *paramters.
    *@param lp Local planner for connecting given 2 Cfgs.
    *@see RoadmapGraph::AddEdge and LocalPlanners::IsConnected
    */


# define DEFAULT_numEdges 5 


template <class CFG, class WEIGHT>
class RandomConnect: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  RandomConnect();
  ~RandomConnect();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);  
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();
  //////////////////////
  // Core: Connection method

  void Connect();
  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
			 CollisionDetection*, 
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge,
			 bool addAllEdges);

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
             CollisionDetection*, DistanceMetric *,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<CFG>& v1, vector<CFG>& v2);

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
             CollisionDetection*, DistanceMetric*,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<vector<CFG> >& verticesList);


 private:
  //////////////////////
  // Data

  int m_numEdges;
};


///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
RandomConnect<CFG,WEIGHT>::RandomConnect():NodeConnectionMethod<CFG,WEIGHT>() { 
  element_name = "random"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
RandomConnect<CFG,WEIGHT>::~RandomConnect() { 
}

template <class CFG, class WEIGHT>
void RandomConnect<CFG,WEIGHT>::SetDefault() {
  m_numEdges = DEFAULT_numEdges;
}


template <class CFG, class WEIGHT>
void
RandomConnect<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\tINTEGER (default m_numEdges = 5)";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
RandomConnect<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " m_numEdges = ";
  _os << m_numEdges;
  _os << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
RandomConnect<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new RandomConnect<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void RandomConnect<CFG,WEIGHT>::
Connect() {
  cout << "Connecting CCs with method: random m_numEdges:"<< m_numEdges << endl ;
  cout << "DOING NOTHING" << endl;
}
 

template <class CFG, class WEIGHT>
void RandomConnect<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		  CollisionDetection* cd , 
		  DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge, 
		  bool addAllEdges) {
#ifndef QUIET
  cout << GetName() << "(m_numEdges="<< m_numEdges <<"): "<<flush;
#endif
  vector<CFG> vertices;
  _rm->m_pRoadmap->GetVerticesData(vertices);
  
  LPOutput<CFG,WEIGHT> lpOutput;
  for (int i=0; i < m_numEdges; i++) {
    int c1id = (int)(OBPRM_lrand()%_rm->m_pRoadmap->GetVertexCount());
    int c2id = (int)(OBPRM_lrand()%_rm->m_pRoadmap->GetVertexCount());
    CFG c1 = vertices[c1id];
    CFG c2 = vertices[c2id];
    
  
    if (!_rm->m_pRoadmap->IsEdge(c1,c2) && 
	lp->IsConnected(_rm->GetEnvironment(), Stats,
			cd,dm,c1,c2,&lpOutput, connectionPosRes, 
			connectionOriRes, (!addAllEdges)) ) {
      _rm->m_pRoadmap->AddEdge(c1id, c2id, lpOutput.edge);
    }
  }  
}

template <class CFG, class WEIGHT>
void RandomConnect<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
             CollisionDetection* cd, DistanceMetric * dm,
             LocalPlanners<CFG,WEIGHT>* lp,
             bool addPartialEdge, bool addAllEdges,
             vector<CFG>& v1, vector<CFG>& v2){

#ifndef QUIET
  cout << GetName() << "(m_numEdges="<< m_numEdges <<"): "<<flush;
#endif

  LPOutput<CFG,WEIGHT> lpOutput;
  for (int i=0; i < m_numEdges; i++) {
    int c1id = (int)(OBPRM_lrand()%v1.size());
    int c2id = (int)(OBPRM_lrand()%v2.size());
    CFG c1 = v1[c1id];
    CFG c2 = v2[c2id];


    VID rmc1, rmc2;
    rmc1=_rm->m_pRoadmap->GetVID(c1);
    rmc2=_rm->m_pRoadmap->GetVID(c2);

    if (!_rm->m_pRoadmap->IsEdge(c1,c2) &&
        lp->IsConnected(_rm->GetEnvironment(), Stats,
                        cd,dm,c1,c2,&lpOutput, connectionPosRes,
                        connectionOriRes, (!addAllEdges)) ) {
      _rm->m_pRoadmap->AddEdge(rmc1, rmc2, lpOutput.edge);
    }
  }

}

template <class CFG, class WEIGHT>
void RandomConnect<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
             CollisionDetection* cd, DistanceMetric* dm,
             LocalPlanners<CFG,WEIGHT>* lp,
             bool addPartialEdge, bool addAllEdges,
             vector<vector<CFG> >& verticesList){
  for(int i=0; i<verticesList.size()-1; ++i)
    for(int j=i+1; j<verticesList.size(); ++j)
      Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges,
            verticesList[i], verticesList[j]);
}
#endif
