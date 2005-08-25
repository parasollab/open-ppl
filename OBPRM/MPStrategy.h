#ifndef MPStrategy_h
#define MPStrategy_h




#include "SwitchDefines.h"
#include<sys/time.h>

#include "OBPRMDef.h"
#include "Roadmap.h"
#include "Input.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"

#include "GeneratePartitions.h"

//#include "ExplicitInstantiation.h"

/* util.h defines EXIT used in initializing the environment*/
#include "util.h"
#include "MPProblem.h"

///Will be used to derive IMP,PRM,RRT,metaplanner, etc.
class MPStrategyMethod : public MPBaseObject 
{
  public:
    MPStrategyMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
      MPBaseObject(in_pNode,in_pProblem) { };
  virtual void ParseXML(TiXmlNode* in_pNode)=0;
  virtual void operator()()=0;
  virtual void PrintOptions(ostream& out_os)=0;
  private:
  
};



class MPStrategy : public MPBaseObject
{
public: 
  MPStrategy(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  
  void ParseStrategyMethod(TiXmlNode* in_pNode);
  
  LocalPlanners<CfgType, WeightType>* GetLocalPlanners() {return m_pLocalPlanners;};
  GenerateMapNodes<CfgType>* GetGenerateMapNodes() {return m_pNodeGeneration;};
  ConnectMap<CfgType, WeightType>* GetConnectMap(){return m_pConnection;};
  bool addPartialEdge, addAllEdges; //move to connect map class
  void PrintOptions(ostream& out_os);
  void Solve(); 
  ///@ToDo Move addPartialEdge, addAllEdges to ConnectMap
 private:
  GenerateMapNodes<CfgType>* m_pNodeGeneration;
  ConnectMap<CfgType, WeightType>* m_pConnection;
  LocalPlanners<CfgType, WeightType>* m_pLocalPlanners;
  
  //Map_Evaluation
  //Filtering
  vector< MPStrategyMethod* > all_MPStrategyMethod;
  MPStrategyMethod* selected_MPStrategyMethod;
};





class PRMRoadmap : public MPStrategyMethod {
  public:
    
    
  PRMRoadmap(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("PRMRoadmap::PRMRoadmap()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~PRMRoadmap::PRMRoadmap()");
    };
    
  virtual void PrintOptions(ostream& out_os) { };
  
  virtual void ParseXML(TiXmlNode* in_pNode) {
      LOG_DEBUG_MSG("PRMRoadmap::ParseXML()");
     
      LOG_DEBUG_MSG("~PRMRoadmap::ParseXML()");
  };
   
  virtual void operator()() {
      LOG_DEBUG_MSG("PRMRoadmap::()");
      Stat_Class Stats;
  
      Clock_Class        NodeGenClock;
      Clock_Class        ConnectionClock;
      vector<CfgType> nodes;
      nodes.erase(nodes.begin(),nodes.end());

  //---------------------------
  // Generate roadmap nodes
  //---------------------------
      NodeGenClock.StartClock("Node Generation");
      GetMPProblem()->GetMPStrategy()->
          GetGenerateMapNodes()->GenerateNodes(GetMPProblem()->GetRoadmap(),Stats,
                                           GetMPProblem()->GetCollisionDetection(),
                                           GetMPProblem()->GetDistanceMetric(),nodes);
      NodeGenClock.StopClock();


  //---------------------------
  // Connect roadmap nodes
  //---------------------------
      ConnectionClock.StartClock("Node Connection");
      GetMPProblem()->GetMPStrategy()->
          GetConnectMap()->Connect(GetMPProblem()->GetRoadmap(), Stats, 
                                 GetMPProblem()->GetCollisionDetection(),
                                 GetMPProblem()->GetDistanceMetric(), 
                                 GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                 GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                                 GetMPProblem()->GetMPStrategy()->addAllEdges);
      ConnectionClock.StopClock();
      GetMPProblem()->WriteRoadmapForVizmo();
      
      Stats.PrintAllStats(GetMPProblem()->GetRoadmap());
  
      LOG_DEBUG_MSG("~PRMRoadmap::()");
  };
   
};



#endif
