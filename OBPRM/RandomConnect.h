#ifndef RandomConnect_h
#define RandomConnect_h
#include "ConnectionMethod.h"


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
class RandomConnect: public ConnectionMethod<CFG,WEIGHT> {
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
  void ParseCommandLine(istrstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);  
  virtual ConnectionMethod<CFG, WEIGHT>* CreateCopy();
  //////////////////////
  // Core: Connection method

  void ConnectComponents();
  void ConnectComponents(Roadmap<CFG, WEIGHT>*, 
			 CollisionDetection*, 
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge,
			 bool addAllEdges);

 private:
  //////////////////////
  // Data

  int numEdges;
};


///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
RandomConnect<CFG,WEIGHT>::RandomConnect():ConnectionMethod<CFG,WEIGHT>() { 
  element_name = "random"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
RandomConnect<CFG,WEIGHT>::~RandomConnect() { 
}


template <class CFG, class WEIGHT>
void RandomConnect<CFG,WEIGHT>::
ParseCommandLine(istrstream& is) {
  char c;
  SetDefault();
  try {
    c = is.peek();
    while(c == ' ' || c == '\n') {
      is.get();
      c = is.peek();
    }    
    if (c >= '0' && c <= '9') {
      if (is >> numEdges) {
        if (numEdges < 0)
	  throw BadUsage();
      } else
        throw BadUsage();
    }

  } catch (BadUsage) {
    cerr << "Error in \'random\' parameters" << endl;
    PrintUsage(cerr);
    exit(-1);
  }
}


template <class CFG, class WEIGHT>
void RandomConnect<CFG,WEIGHT>::SetDefault() {
  numEdges = DEFAULT_numEdges;
}


template <class CFG, class WEIGHT>
void
RandomConnect<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\tINTEGER (default numEdges = 5)";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
RandomConnect<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " numEdges = ";
  _os << numEdges;
  _os << endl;
}


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>* 
RandomConnect<CFG,WEIGHT>::
CreateCopy() {
  ConnectionMethod<CFG,WEIGHT>* _copy = 
           new RandomConnect<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void RandomConnect<CFG,WEIGHT>::
ConnectComponents() {
  cout << "Connecting CCs with method: random numEdges:"<< numEdges << endl ;
  cout << "DOING NOTHING" << endl;
}
 

template <class CFG, class WEIGHT>
void RandomConnect<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, 
		  CollisionDetection* cd , 
		  DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge, 
		  bool addAllEdges) {
  //cout << "Connecting CCs with method: numEdges="<< numEdges << endl;
#ifndef QUIET
  cout << GetName() << "(numEdges="<< numEdges <<"): "<<flush;
#endif
  vector<CFG> vertices;
  _rm->m_pRoadmap->GetVerticesData(vertices);
  //vector<pair<SID,vector<LP> > > sets = lp->planners.GetLPSets();
  
  LPOutput<CFG,WEIGHT> lpOutput;
  for (int i=0; i < numEdges; i++) {
    int c1id = (int)(lrand48()%_rm->m_pRoadmap->GetVertexCount());
    int c2id = (int)(lrand48()%_rm->m_pRoadmap->GetVertexCount());
    CFG c1 = vertices[c1id];
    CFG c2 = vertices[c2id];
    
    //int random_lp = (int)(lrand48()%(lp->GetCounter())) + 1;
  
    if (!_rm->m_pRoadmap->IsEdge(c1,c2) && 
	lp->IsConnected(_rm->GetEnvironment(),
			cd,dm,c1,c2,&lpOutput, connectionPosRes, 
			connectionOriRes, (!addAllEdges)) ) {
      _rm->m_pRoadmap->AddEdge(c1id, c2id, lpOutput.edge);
    }
  }  
 
}

#endif
