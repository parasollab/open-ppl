#ifndef AllPairsNodeConnection_h
#define AllPairsNodeConnection_h
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
class AllPairsNodeConnection: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
   AllPairsNodeConnection();
   AllPairsNodeConnection(XMLNodeReader& in_Node, MPProblem* in_pProblem);
   //AllPairsNodeConnection(int k);
   virtual ~AllPairsNodeConnection();
 
  //////////////////////
  // Access
   void SetDefault() {};

  //////////////////////
  // I/O methods
   virtual void PrintUsage(ostream& _os) {};
   virtual void PrintValues(ostream& _os) {};  
  ///Used in new MPProblem framework.
   virtual void PrintOptions(ostream& out_os) {
     out_os << "    " << "AllPairsNodeConnection::" 
            << "    CheckIfSameCC = " << this->m_CheckIfSameCC << endl;;
  
   };  
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();
  virtual void ParseXML(XMLNodeReader& in_Node);

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
             vector<vector<VID> >& verticesList) {
    LOG_DEBUG_MSG("AllPairsNodeConnection::Connection(vector<vector<>>) not implemented yet");
    exit(-1);};

  
 private:
  //////////////////////
  // Data
   
 // int kclosest;
};


template <class CFG, class WEIGHT>
AllPairsNodeConnection<CFG,WEIGHT>::
AllPairsNodeConnection():NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closest"; 
  SetDefault();
}

template <class CFG, class WEIGHT>
AllPairsNodeConnection<CFG,WEIGHT>::
AllPairsNodeConnection(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem) { 
  LOG_DEBUG_MSG("AllPairsNodeConnection::AllPairsNodeConnection()"); 
  this->element_name = "AllPairsNodeConnection"; 
  //SetDefault();
  ParseXML(in_Node);
  
  
  LOG_DEBUG_MSG("~AllPairsNodeConnection::AllPairsNodeConnection()"); 
}




/*
template <class CFG, class WEIGHT>
Closest<CFG,WEIGHT>::Closest(int k):NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closest"; 
  kclosest = k;
}
*/

template <class CFG, class WEIGHT>
AllPairsNodeConnection<CFG,WEIGHT>::
~AllPairsNodeConnection() { 
}


template <class CFG, class WEIGHT>
void AllPairsNodeConnection<CFG,WEIGHT>::
ParseXML(XMLNodeReader& in_Node) { 
  

  
}



template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
AllPairsNodeConnection<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
      new AllPairsNodeConnection<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
void AllPairsNodeConnection<CFG,WEIGHT>::
Connect() {
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl ;
  //cout << "DOING NOTHING" << endl;
}



template <class CFG, class WEIGHT>
void AllPairsNodeConnection<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;

  cout << "AllPairsNodeConnection::Connect()" << endl;
  
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<VID> verticesVID;
  pMap->GetVerticesVID(verticesVID);
  LPOutput<CFG,WEIGHT> lpOutput;
  for(int i=0; i<verticesVID.size(); ++i) {
    for(int j=i+1; j<verticesVID.size(); ++j) {
      //cout << "(i,j) = (" << i << "," << j <<")" << endl;
      if(_rm->m_pRoadmap->IsEdge(verticesVID[i], verticesVID[j])) continue;
      
      if (this->m_CheckIfSameCC) {
        if(IsSameCC(*(_rm->m_pRoadmap), verticesVID[i], verticesVID[j])) continue; 
      }
      Stats.IncConnections_Attempted();
        if (lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm,
                            _rm->m_pRoadmap->GetData(verticesVID[i]),
                            _rm->m_pRoadmap->GetData(verticesVID[j]),
                            &lpOutput,
                            this->connectionPosRes, this->connectionOriRes, 
                            (!addAllEdges) )) {
            _rm->m_pRoadmap->AddEdge(verticesVID[i], verticesVID[j],
                                                      lpOutput.edge);
            Stats.IncConnections_Made();
          }
        }
     }
  
}

/*
template <class CFG, class WEIGHT>
void AllPairsNodeConnection<CFG,WEIGHT>::
AllPairsNodeConnection(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
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

*/
template <class CFG, class WEIGHT>
void AllPairsNodeConnection<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<VID>& v1, vector<VID>& v2) 
{ 


  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  LPOutput<CFG,WEIGHT> lpOutput;
  for(int i=0; i<v1.size(); ++i) {
    for(int j=0; j<v2.size(); ++j) {
      if(v1[i] == v2[j]) continue;
      if(_rm->m_pRoadmap->IsEdge(v1[i], v2[j])) continue;
      
      if (this->m_CheckIfSameCC) {
        if(IsSameCC(*(_rm->m_pRoadmap), v1[i], v2[j])) continue; 
      }
      Stats.IncConnections_Attempted();
        if (lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm,
                            _rm->m_pRoadmap->GetData(v1[i]),
                            _rm->m_pRoadmap->GetData(v2[j]),
                            &lpOutput,
                            this->connectionPosRes, this->connectionOriRes, 
                            (!addAllEdges) )) {
            _rm->m_pRoadmap->AddEdge(v1[i], v2[j],
                                                      lpOutput.edge);
            Stats.IncConnections_Made();
          }
        }
     }





/*

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
               &lpOutput, this->connectionPosRes, this->connectionOriRes, 
               (!addAllEdges) )) {
      _rm->m_pRoadmap->AddEdge(KP->first, KP->second, lpOutput.edge);
    }
  } 
*/
}

#endif
