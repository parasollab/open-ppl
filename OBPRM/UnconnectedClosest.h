#ifndef UnconnectedClosest_h
#define UnconnectedClosest_h
#include "ConnectionMethod.h"


//Connect K Unconnected Closest
   /**Connect nodes in map to their k closest neighbors not in same cc.
    *
    *@param info provides information other than connection, like
    *collision dection, local planner, and distance metrics.
    *@param _cn provides information for specific node connection 
    *paramters.
    *@param lp Local planner for connecting given 2 Cfgs.
    *
    *@see RoadmapGraph::AddEdge and LocalPlanners::IsConnected
    */


#define KUNCONNECTEDCLOSEST 5 


template <class CFG, class WEIGHT>
class UnconnectedClosest: public ConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  UnconnectedClosest();
  ~UnconnectedClosest();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods

  void ParseCommandLine(std::istringstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);  
  virtual ConnectionMethod<CFG, WEIGHT>* CreateCopy();
  //////////////////////
  // Core: Connection method
  void ConnectComponents();
  void ConnectComponents(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
			 CollisionDetection*, 
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge,
			 bool addAllEdges);

 private:
  //////////////////////
  // Data

  int kclosest;
};


template <class CFG, class WEIGHT>
UnconnectedClosest<CFG,WEIGHT>::UnconnectedClosest():ConnectionMethod<CFG,WEIGHT>() { 
  element_name = "unconnectedClosest"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
UnconnectedClosest<CFG,WEIGHT>::~UnconnectedClosest() { 
}


template <class CFG, class WEIGHT>
void UnconnectedClosest<CFG,WEIGHT>::
ParseCommandLine(std::istringstream& is) {
  char c;
  SetDefault();
  try {
    c = is.peek();
    while(c == ' ' || c == '\n') {
      is.get();
      c = is.peek();
    }    
    if (c >= '0' && c <= '9') {
      if (is >> kclosest) {
        if (kclosest < 0)
          throw BadUsage();
      } else
        throw BadUsage();
    }

  } catch (BadUsage) {
    cerr << "ERROR in \'unconnectedClosest\' parameters\n";
    PrintUsage(cerr);
    exit(-1);
  }
}


template <class CFG, class WEIGHT>
void UnconnectedClosest<CFG,WEIGHT>::SetDefault() {
  kclosest = KUNCONNECTEDCLOSEST;
}


template <class CFG, class WEIGHT>
void
UnconnectedClosest<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\tINTEGER (default " << KUNCONNECTEDCLOSEST << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
UnconnectedClosest<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " kclosest = ";
  _os << kclosest;
  _os << endl;
}


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>* 
UnconnectedClosest<CFG,WEIGHT>::
CreateCopy() {
  ConnectionMethod<CFG,WEIGHT>* _copy = 
           new UnconnectedClosest<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
void UnconnectedClosest<CFG,WEIGHT>::
ConnectComponents() {
  //cout << "Connecting CCs with method: unconnectedClosest k="<< kclosest << endl ;
  //cout << "DOING NOTHING" << endl;
}


template <class CFG, class WEIGHT>
void UnconnectedClosest<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
                  CollisionDetection* cd , 
                  DistanceMetric * dm,
                  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges) {
  //cout << "Connecting CCs with method: unconnectedClosest k="<< kclosest << endl;
#ifndef QUIET
  cout << "unconnectedClosest(k="<< kclosest <<"): "<<flush;
#endif

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<CFG> vertices;
  pMap->GetVerticesData(vertices);
  const int k = min(kclosest,vertices.size());

  bool done = false;
  for(int z=0; (z<vertices.size())&&!done;++z) {  
    vector< pair<VID,VID> > kp;
    // Find k closest cfgs to each cfg in the roadmap
    if(k < vertices.size()) {
      kp = dm->KUnconnectedClosest(_rm, vertices[z], vertices, k); 
    } 
    else { // all the pairs
      for(int i=0; i<vertices.size(); ++i)
	for(int j=0; j<vertices.size(); ++j){
	  if( vertices[i]==vertices[j] ) continue;
	  kp.push_back(pair<VID,VID>(pMap->GetVID(vertices[i]), pMap->GetVID(vertices[j])));
	  done=true;
	}
    }
    
    //	 for each pair identified
    LPOutput<CFG,WEIGHT> lpOutput;
    for (int j=0; j < kp.size(); j++) {
      if( _rm->m_pRoadmap->IsEdge(kp[j].first, kp[j].second)) continue;
#if CHECKIFSAMECC
      if(IsSameCC(*(_rm->m_pRoadmap), kp[j].first,kp[j].second)) continue;
#endif	
      Stats.IncConnections_Attempted();
      if (lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm,
			  _rm->m_pRoadmap->GetData(kp[j].first),
			  _rm->m_pRoadmap->GetData(kp[j].second),
			  &lpOutput,
			  connectionPosRes, connectionOriRes, 
			  (!addAllEdges) )) {
	_rm->m_pRoadmap->AddEdge(kp[j].first, kp[j].second, lpOutput.edge);
	Stats.IncConnections_Made();
      }
    } //endfor j
  }//endfor z
}

#endif
