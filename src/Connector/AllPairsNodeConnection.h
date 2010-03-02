#ifndef AllPairsNodeConnection_h
#define AllPairsNodeConnection_h


#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "GraphAlgo.h"


template <class CFG, class WEIGHT>
class AllPairsNodeConnection: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  AllPairsNodeConnection();
  AllPairsNodeConnection(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~AllPairsNodeConnection();

  virtual void SetDefault() {};

  virtual void PrintUsage(ostream& _os) {};
  virtual void PrintValues(ostream& _os) {};   
  
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();

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
             vector<vector<VID> >& verticesList) {
    LOG_DEBUG_MSG("AllPairsNodeConnection::Connection(vector<vector<>>) not implemented yet");
    exit(-1);
  };

 private:
};


template <class CFG, class WEIGHT>
AllPairsNodeConnection<CFG,WEIGHT>::
AllPairsNodeConnection():NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "AllPairsNodeConnection"; 
  SetDefault();
}


template <class CFG, class WEIGHT>
AllPairsNodeConnection<CFG,WEIGHT>::
AllPairsNodeConnection(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem) { 
  LOG_DEBUG_MSG("AllPairsNodeConnection::AllPairsNodeConnection()"); 
  this->element_name = "AllPairsNodeConnection"; 
  this->ParseXML(in_Node);
  LOG_DEBUG_MSG("~AllPairsNodeConnection::AllPairsNodeConnection()"); 
}


template <class CFG, class WEIGHT>
AllPairsNodeConnection<CFG,WEIGHT>::
~AllPairsNodeConnection() { 
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
AllPairsNodeConnection<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = new AllPairsNodeConnection<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void AllPairsNodeConnection<CFG,WEIGHT>::
Connect() {
}


template <class CFG, class WEIGHT>
void AllPairsNodeConnection<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
  cout << "AllPairsNodeConnection::Connect()" << endl;
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
  vector<VID> verticesVID;
  pMap->GetVerticesVID(verticesVID);
  LPOutput<CFG,WEIGHT> lpOutput;
  for(int i=0; i<verticesVID.size(); ++i) {
    for(int j=i+1; j<verticesVID.size(); ++j) {
      //cout << "(i,j) = (" << i << "," << j <<")" << endl;
      if(_rm->m_pRoadmap->IsEdge(verticesVID[i], verticesVID[j])) continue;
	
      cmap.reset();      
      if (this->m_CheckIfSameCC) 
        if(is_same_cc(*(_rm->m_pRoadmap), cmap, verticesVID[i], verticesVID[j])) continue; 
      
      Stats.IncConnections_Attempted();
      if (lp->IsConnected(_rm->GetEnvironment(),Stats,dm,
                          (*(_rm->m_pRoadmap->find_vertex(verticesVID[i]))).property(),
                          (*(_rm->m_pRoadmap->find_vertex(verticesVID[j]))).property(),
                          &lpOutput,
                          this->connectionPosRes, this->connectionOriRes, 
                          (!addAllEdges) )) {
        _rm->m_pRoadmap->AddEdge(verticesVID[i], verticesVID[j], lpOutput.edge);
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
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<VID>& v1, vector<VID>& v2) 
{
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
  LPOutput<CFG,WEIGHT> lpOutput;
  for(int i=0; i<v1.size(); ++i) {
    for(int j=0; j<v2.size(); ++j) {
      if(v1[i] == v2[j]) continue;
      
      if(_rm->m_pRoadmap->IsEdge(v1[i], v2[j])) continue;
      
      cmap.reset();
      if (this->m_CheckIfSameCC) 
        if(is_same_cc(*(_rm->m_pRoadmap), cmap, v1[i], v2[j])) continue; 
      
      Stats.IncConnections_Attempted();

      if (lp->IsConnected(_rm->GetEnvironment(),Stats,dm,
                          (*(_rm->m_pRoadmap->find_vertex(v1[i]))).property(),
                          (*(_rm->m_pRoadmap->find_vertex(v2[j]))).property(),
                          &lpOutput,
                          this->connectionPosRes, this->connectionOriRes, 
                          (!addAllEdges) )) {
        _rm->m_pRoadmap->AddEdge(v1[i], v2[j], lpOutput.edge);
        Stats.IncConnections_Made();
      }
    }
  }
}

#endif
