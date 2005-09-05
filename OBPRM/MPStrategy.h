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
#include "MPCharacterizer.h"

///Will be used to derive IMP,PRM,RRT,metaplanner, etc.

class MPStrategyMethod : public MPBaseObject 
{
  public:
    MPStrategyMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
      MPBaseObject(in_pNode,in_pProblem) { };
  virtual void ParseXML(TiXmlNode* in_pNode)=0;
  virtual void operator()()=0;
  virtual void operator()(int in_RegionID)=0;
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
  MPCharacterizer<CfgType, WeightType>* GetCharacterizer(){return m_pCharacterizer;};
  bool addPartialEdge, addAllEdges; //move to connect map class
  void PrintOptions(ostream& out_os);
  void Solve(); 
  MPStrategyMethod* GetMPStrategyMethod(string& );////////////////////////
  ///@ToDo Move addPartialEdge, addAllEdges to ConnectMap
 private:
  GenerateMapNodes<CfgType>* m_pNodeGeneration;
  ConnectMap<CfgType, WeightType>* m_pConnection;
  LocalPlanners<CfgType, WeightType>* m_pLocalPlanners;
  MPCharacterizer<CfgType, WeightType>* m_pCharacterizer;
  
  //Map_Evaluation
  vector< MPStrategyMethod* > all_MPStrategyMethod;
  string m_strController_MPStrategyMethod;
};

class MPCompare : public MPStrategyMethod {
  
public: 
  MPCompare(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("MPCompare::MPCompare()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~MPCompare::MPCompare()");
  };
  
  
  
  virtual void PrintOptions(ostream& out_os) { };
  
  virtual void ParseXML(TiXmlNode* in_pNode) {
    LOG_DEBUG_MSG("MPCompare::ParseXML()");
      
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; 
         pChild = pChild->NextSibling()) {
           
      if(string(pChild->Value()) == "input") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          string strategy(in_char);
          m_vecStrStrategyMethod.push_back(strategy);
        }
      } else {
        LOG_WARNING_MSG("MPCompare::  I don't know: "<< endl << *pChild);
      }
    }
      
    LOG_DEBUG_MSG("~MPCompare::ParseXML()");
  };
   
  virtual void operator()(int in_RegionID) { };
  virtual void operator()() {
    LOG_DEBUG_MSG("MPCompare::()");
    MPStrategyMethod* input1 = GetMPProblem()->GetMPStrategy()->
        GetMPStrategyMethod(m_vecStrStrategyMethod[0]);
    MPStrategyMethod* input2 = GetMPProblem()->GetMPStrategy()->
        GetMPStrategyMethod(m_vecStrStrategyMethod[1]);
    
    int Input1RegionId = GetMPProblem()->CreateMPRegion();
    int Input2RegionId = GetMPProblem()->CreateMPRegion();
    
    LOG_DEBUG_MSG("MPCompare::() -- executing "<< m_vecStrStrategyMethod[0]);
    (*input1)(Input1RegionId);
    LOG_DEBUG_MSG("MPCompare::() -- executing "<< m_vecStrStrategyMethod[1]);
    (*input2)(Input2RegionId); 
    LOG_DEBUG_MSG("MPCompare::()");
  }
  
  private:
    vector<string> m_vecStrStrategyMethod;

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
      } else if(string(pChild->Value()) == "NodeCharacterizer") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          string node_char(in_char);
          m_vecNodeCharacterizerLabels.push_back(node_char);
        }
      } else {
        LOG_WARNING_MSG("PRMRoadmap::  I don't know: "<< endl << *pChild);
      }
    }
      
   
    
    LOG_DEBUG_MSG("~PRMRoadmap::ParseXML()");
  };
   
  virtual void operator()(int in_RegionID) {
    LOG_DEBUG_MSG("PRMRoadmap::()");
    MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
    Stat_Class * pStatClass = region->GetStatClass();
    
  
    Clock_Class Allstuff;


    Allstuff.StartClock("Everything");
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
      vector< CfgType > vectorCfgs;
      NodeGenerationMethod<CfgType> * pNodeGen;
      pNodeGen = GetMPProblem()->GetMPStrategy()->
          GetGenerateMapNodes()->GetMethod(*itr);
      pNodeGen->GenerateNodes(region, vectorCfgs); /////////this needs fixing bad.
      cout << "Finished ... I did this many : " << vectorCfgs.size() << endl;
      region->AddToRoadmap(vectorCfgs);
    }
      
    NodeGenClock.StopClock();

    MPCharacterizer<CfgType, WeightType>* characterize =
            GetMPProblem()->GetMPStrategy()->GetCharacterizer();
    typedef vector<string>::iterator J;
    for(J itr = m_vecNodeCharacterizerLabels.begin(); 
        itr != m_vecNodeCharacterizerLabels.end(); ++itr)
    {
      NodeCharacterizerMethod<CfgType,WeightType>* pNodeChar;
      pNodeChar = characterize->GetNodeCharacterizerMethod(*itr);
      pNodeChar->Characterize(region);
    }
      
    
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
      
      pConnection->Connect(region->GetRoadmap(), *pStatClass, 
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
      
      pConnection->Connect(region->GetRoadmap(), *pStatClass, 
                           GetMPProblem()->GetCollisionDetection(),
                           GetMPProblem()->GetDistanceMetric(), 
                           GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                           GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                           GetMPProblem()->GetMPStrategy()->addAllEdges);
      
    }
      
      
      
    ConnectionClock.StopClock();
    region->WriteRoadmapForVizmo();
      
    pStatClass->PrintAllStats(region->GetRoadmap());



    cout << "I took this long" << endl;

    Allstuff.StopPrintClock();
  
    LOG_DEBUG_MSG("~PRMRoadmap::()");
  }
  
  virtual void operator()() {
    int newRegionId = GetMPProblem()->CreateMPRegion();
    (*this)(newRegionId);      
  };

private:
  vector<string> m_vecStrNodeGenLabels;
  vector<string> m_vecStrNodeConnectionLabels;
  vector<string> m_vecStrComponentConnectionLabels;
  vector<string> m_vecNodeCharacterizerLabels;
  string m_strLocalPlannerLabel;
   
};


class RoadmapInput : public MPStrategyMethod {
  public:
    
    
  RoadmapInput(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("RoadmapInput::RoadmapInput()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~RoadmapInput::RoadmapInput()");
    };
    
    virtual void PrintOptions(ostream& out_os) { };
  
    virtual void ParseXML(TiXmlNode* in_pNode) {
      LOG_DEBUG_MSG("RoadmapInput::ParseXML()");
      
          const char* in_char = in_pNode->ToElement()->Attribute("input_map");
          if(in_char) {
            m_strInputFileName = string(in_char);
          }
         else {
           LOG_ERROR_MSG("RoadmapInput::  I don't know: "<< endl << *in_pNode);exit(-1);
        }
    
        LOG_DEBUG_MSG("~RoadmapInput::ParseXML()");
    };
   
    virtual void operator()(int in_RegionID) {
      LOG_DEBUG_MSG("PRMRoadmap::() -- Reading in file: " << m_strInputFileName);
      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
      
      region->GetRoadmap()->ReadRoadmapGRAPHONLY(m_strInputFileName.c_str());
      
      LOG_DEBUG_MSG("~PRMRoadmap::()");
    }
  
    virtual void operator()() {
      int newRegionId = GetMPProblem()->CreateMPRegion();
      (*this)(newRegionId);      
    };

  private:
    string m_strInputFileName;
   
};




#endif
