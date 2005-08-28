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
      
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "node_generation_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          string node_generation_method(in_char);
          m_vecStrNodeGenLabels.push_back(node_generation_method);
        }
      } else if(string(pChild->Value()) == "node_connection_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        LOG_DEBUG_MSG("PRMRoadmap::ParseXML() -- node_connection_method");
        if(in_char) {
          string connect_method(in_char);
          m_vecStrNodeConnectionLabels.push_back(connect_method);
        }
      } else if(string(pChild->Value()) == "component_connection_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        LOG_DEBUG_MSG("PRMRoadmap::ParseXML() -- component_connection_method");
        if(in_char) {
          string connect_method(in_char);
          m_vecStrComponentConnectionLabels.push_back(connect_method);
        }
      } else if(string(pChild->Value()) == "lp_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          m_strLocalPlannerLabel = string(in_char);
        }
      } else {
        LOG_WARNING_MSG("PRMRoadmap::  I don't know: "<< endl << *pChild);
      }
    }
      
    
    
    
    
    LOG_DEBUG_MSG("~PRMRoadmap::ParseXML()");
  };
   
  virtual void operator()() {
      LOG_DEBUG_MSG("PRMRoadmap::()");
      Stat_Class * pStatClass = GetMPProblem()->GetStatClass();
  
      Clock_Class        NodeGenClock;
      Clock_Class        ConnectionClock;
      vector<CfgType> nodes;
      nodes.erase(nodes.begin(),nodes.end());

  //---------------------------
  // Generate roadmap nodes
  //---------------------------
      NodeGenClock.StartClock("Node Generation");
      typedef vector<string>::iterator I;
      for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr)
      {
        vector< Cfg_free > vectorCfgs;
        NodeGenerationMethod<Cfg_free> * pNodeGen;
        pNodeGen = GetMPProblem()->GetMPStrategy()->
            GetGenerateMapNodes()->GetMethod(*itr);
        pNodeGen->GenerateNodes(vectorCfgs);
        cout << "Finished ... I did this many : " << vectorCfgs.size() << endl;
        GetMPProblem()->AddToRoadmap(vectorCfgs);
      }
      
      NodeGenClock.StopClock();


  //---------------------------
  // Connect roadmap nodes
  //---------------------------
      ConnectionClock.StartClock("Node Connection");
      
      ConnectMap<CfgType, WeightType>* connectmap = GetMPProblem()->GetMPStrategy()->GetConnectMap();
      typedef vector<string>::iterator J;
      for(J itr = m_vecStrNodeConnectionLabels.begin(); 
          itr != m_vecStrNodeConnectionLabels.end(); ++itr)
      {
        LOG_DEBUG_MSG("PRMRoadmap:: " << *itr);
        NodeConnectionMethod<CfgType,WeightType>* pConnection;
        pConnection = connectmap->GetNodeMethod(*itr);
      
        pConnection->Connect(GetMPProblem()->GetRoadmap(), *pStatClass, 
                                 GetMPProblem()->GetCollisionDetection(),
                                 GetMPProblem()->GetDistanceMetric(), 
                                 GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                 GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                                 GetMPProblem()->GetMPStrategy()->addAllEdges);
      
      }
      
      typedef vector<string>::iterator K;
      for(K itr = m_vecStrComponentConnectionLabels.begin(); 
          itr != m_vecStrComponentConnectionLabels.end(); ++itr)
      {
        LOG_DEBUG_MSG("PRMRoadmap:: " << *itr);
        ComponentConnectionMethod<CfgType,WeightType>* pConnection;
        pConnection = connectmap->GetComponentMethod(*itr);
      
        pConnection->Connect(GetMPProblem()->GetRoadmap(), *pStatClass, 
                             GetMPProblem()->GetCollisionDetection(),
                             GetMPProblem()->GetDistanceMetric(), 
                             GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                             GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                             GetMPProblem()->GetMPStrategy()->addAllEdges);
      
      }
      
      
      
      ConnectionClock.StopClock();
      GetMPProblem()->WriteRoadmapForVizmo();
      
      pStatClass->PrintAllStats(GetMPProblem()->GetRoadmap());
  
      LOG_DEBUG_MSG("~PRMRoadmap::()");
  };

private:
  vector<string> m_vecStrNodeGenLabels;
  vector<string> m_vecStrNodeConnectionLabels;
  vector<string> m_vecStrComponentConnectionLabels;
  string m_strLocalPlannerLabel;
   
};



#endif
