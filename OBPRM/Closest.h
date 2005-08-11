#ifndef Closest_h
#define Closest_h
#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "GraphAlgo.h"

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
class Closest: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  Closest();
  Closest(int k);
  ~Closest();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods
  void ParseCommandLine(std::istringstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);  
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();

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
             vector<CFG>& v1, vector<CFG>& v2);

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
             CollisionDetection*, DistanceMetric*,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<vector<CFG> >& verticesList);


 private:
  //////////////////////
  // Data

  int kclosest;
};


template <class CFG, class WEIGHT>
Closest<CFG,WEIGHT>::Closest():NodeConnectionMethod<CFG,WEIGHT>() { 
  element_name = "closest"; 
  SetDefault();
}


template <class CFG, class WEIGHT>
Closest<CFG,WEIGHT>::Closest(int k):NodeConnectionMethod<CFG,WEIGHT>() { 
  element_name = "closest"; 
  kclosest = k;
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
NodeConnectionMethod<CFG,WEIGHT>* 
Closest<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new Closest<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
Connect() {
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl ;
  //cout << "DOING NOTHING" << endl;
}



template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;
#ifndef QUIET
  cout << "closest(k="<< kclosest <<"): "<<flush;
#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<CFG> vertices;
  pMap->GetVerticesData(vertices);
  const int k = (int)min(kclosest,vertices.size());
  
  vector< pair<VID,VID> > kp;
  // Find k closest cfgs to each cfg in the roadmap
  if(k < vertices.size()) {// - 1) {
    //kp = dm->KClosest(_rm, vertices, vertices, k);
    kp = dm->FindKClosestPairs(_rm, vertices, vertices, k);     
  } 
  else { // all the pairs
    for(int i=0; i<vertices.size(); ++i)
      for(int j=0; j<vertices.size(); ++j){
        if( vertices[i]==vertices[j] ) continue;
        kp.push_back(pair<VID,VID>(pMap->GetVID(vertices[i]), pMap->GetVID(vertices[j])));
      }
  }
  
  // for each pair identified
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
}


template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<vector<CFG> >& verticesList)
{
  for(int i=0; i<verticesList.size()-1; ++i)
    for(int j=i+1; j<verticesList.size(); ++j)
      Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges,
            verticesList[i], verticesList[j]);
}


template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<CFG>& v1, vector<CFG>& v2) 
{
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;
#ifndef QUIET
  cout << "closest*(k="<< kclosest <<"): "<<flush;
#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;  
  vector< pair<VID,VID> > kp;

  if((v1.size() < kclosest) && (v2.size() < kclosest)) {//all pairs
    for(typename vector<CFG>::iterator I = v1.begin(); I != v1.end(); ++I)
      for(typename vector<CFG>::iterator J = v2.begin(); J != v2.end(); ++J)
    if(*I != *J)
      kp.push_back(make_pair<VID,VID>(pMap->GetVID(*I), pMap->GetVID(*J)));
  }

  if((v1.size() >= kclosest) && (v2.size() >= kclosest)) { //k closest each way
    kp = dm->FindKClosestPairs(_rm, v1, v2, kclosest);
    if(v1 != v2) {
      vector<pair<VID,VID> > kp2;
      kp2 = dm->FindKClosestPairs(_rm, v2, v1, kclosest);
      kp.insert(kp.end(), kp2.begin(), kp2.end());
    }
  } 

  if((v1.size() < kclosest) && (v2.size() >= kclosest)) { //k closest in v2
    kp = dm->FindKClosestPairs(_rm, v1, v2, kclosest);
  }

  if((v1.size() >= kclosest) && (v2.size() < kclosest)) { //k closest in v1
    kp = dm->FindKClosestPairs(_rm, v2, v1, kclosest);
  }

  // for each pair identified
  LPOutput<CFG,WEIGHT> lpOutput;
  for(vector<pair<VID,VID> >::iterator KP = kp.begin(); KP != kp.end(); ++KP) {
    if(_rm->m_pRoadmap->IsEdge(KP->first, KP->second)) 
      continue;
#if CHECKIFSAMECC
    if(IsSameCC(*(_rm->m_pRoadmap), KP->first, KP->second)) 
      continue;
#endif
    if(lp->IsConnected(_rm->GetEnvironment(), Stats, cd, dm,
               _rm->m_pRoadmap->GetData(KP->first),
               _rm->m_pRoadmap->GetData(KP->second),
               &lpOutput, connectionPosRes, connectionOriRes, 
               (!addAllEdges) )) {
      _rm->m_pRoadmap->AddEdge(KP->first, KP->second, lpOutput.edge);
    }
  } 
}

#endif
