#ifndef Closest_h
#define Closest_h
#include "ConnectionMethod.h"


//Connect K Closest
   /**Connect nodes in map to their k closest neighbors.
    *Following Algorithm is used:
    *   -# for evry node, cfg1, in roadmap
    *       -# find k closet neighbors for cfg1
    *       -# lp_set is a local planner set defined in info.lpsetid
    *       -# for every node, cfg2, in k-closest neighbor list for cfg1
    *           -# using local planning functions in lp_set
    *              to connect cfg1 and cfg2
    *           -# if connected, add this edge to map, _rm.
    *       -#end for
    *   -# end for
    *
    *@param info provides inforamtion other than connection, like
    *collision dection, local planner, and distance metrics.
    *@param _cn provides information for specific node connection 
    *paramters.
    *@param lp Local planner for connecting given 2 Cfgs.
    *
    *@see RoadmapGraph::AddEdge and LocalPlanners::IsConnected
    */


#define KCLOSEST 5 


template <class CFG, class WEIGHT>
class Closest: public ConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  Closest();
  ~Closest();
 
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
Closest<CFG,WEIGHT>::Closest():ConnectionMethod<CFG,WEIGHT>() { 
  element_name = "closest"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
Closest<CFG,WEIGHT>::~Closest() { 
}


template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
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
    cerr << "ERROR in \'closest\' parameters\n";
    PrintUsage(cerr);
    exit(-1);
  }
}


template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::SetDefault() {
  kclosest = KCLOSEST;
}


template <class CFG, class WEIGHT>
void
Closest<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\tINTEGER (default " << KCLOSEST << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
Closest<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " kclosest = ";
  _os << kclosest;
  _os << endl;
}


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>* 
Closest<CFG,WEIGHT>::
CreateCopy() {
  ConnectionMethod<CFG,WEIGHT>* _copy = 
           new Closest<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
ConnectComponents() {
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl ;
  //cout << "DOING NOTHING" << endl;
}


template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
                  CollisionDetection* cd , 
                  DistanceMetric * dm,
                  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges) {
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;
#ifndef QUIET
  cout << "closest(k="<< kclosest <<"): "<<flush;
#endif
  
  vector<CFG> vec1;
  _rm->m_pRoadmap->GetVerticesData(vec1);
  const int verticeSize = vec1.size();
  const int k = min(kclosest,verticeSize);
  vector<CFG> vec2 = vec1;

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  
  vector< pair<VID,VID> > kp;
  // Find k closest cfgs to each cfg in the roadmap
  if(k < vec2.size() - 1) {
    kp = dm->FindKClosestPairs(_rm, vec1, vec2, k); 
  } 
  else { // all the pairs
    for(int i=0; i<vec1.size(); ++i)
      for(int j=0; j<vec2.size(); ++j){
        if( vec1[i]==vec2[j] ) continue;
        kp.push_back(pair<VID,VID>(pMap->GetVID(vec1[i]), pMap->GetVID(vec2[j])));
      }
  }
  
  // for each pair identified
  LPOutput<CFG,WEIGHT> lpOutput;
  for (int j=0; j < kp.size(); j++) {
    if( _rm->m_pRoadmap->IsEdge(kp[j].first, kp[j].second)) continue;
#if CHECKIFSAMECC
    if(IsSameCC(*(_rm->m_pRoadmap), kp[j].first,kp[j].second)) continue;
#endif

    if (lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm,
                        _rm->m_pRoadmap->GetData(kp[j].first),
                        _rm->m_pRoadmap->GetData(kp[j].second),
                        &lpOutput,
                        connectionPosRes, connectionOriRes, 
                        (!addAllEdges) )) {
      _rm->m_pRoadmap->AddEdge(kp[j].first, kp[j].second, lpOutput.edge);
    }
  } //endfor j
}

#endif
