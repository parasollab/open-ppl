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

#include "MapEvaluator.h"

#include "MPStrategy/MPStrategyMethod.h"


class MPStrategy : public MPBaseObject
{
public: 
  MPStrategy(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  
  void ParseStrategyMethod(TiXmlNode* in_pNode);
  
  LocalPlanners<CfgType, WeightType>* GetLocalPlanners() {return m_pLocalPlanners;};
  GenerateMapNodes<CfgType>* GetGenerateMapNodes() {return m_pNodeGeneration;};
  ConnectMap<CfgType, WeightType>* GetConnectMap(){return m_pConnection;};
  MPCharacterizer<CfgType, WeightType>* GetCharacterizer(){return m_pCharacterizer;};
  MapEvaluator< CfgType, WeightType > * GetMapEvaluator() { return m_Evaluator;}; 

  bool addPartialEdge, addAllEdges; //move to connect map class
  void PrintOptions(ostream& out_os);
  void Solve(); 
  MPStrategyMethod* GetMPStrategyMethod(string& );////////////////////////
  ///@ToDo Move addPartialEdge, addAllEdges to ConnectMap
 private:
  GenerateMapNodes<CfgType>* m_pNodeGeneration;
  ConnectMap<CfgType, WeightType>* m_pConnection;
  LocalPlanners<CfgType, WeightType>* m_pLocalPlanners;
  //Characterization and Filtering
  MPCharacterizer<CfgType, WeightType>* m_pCharacterizer;
  
  //Map_Evaluation
  MapEvaluator<CfgType, WeightType>* m_Evaluator;

  vector< MPStrategyMethod* > all_MPStrategyMethod;
  string m_strController_MPStrategyMethod;
};

#include "MPStrategy/PRMStrategy.h"
#include "MPStrategy/RoadmapToolsStrategy.h"
#include "MPStrategy/PRMIncrementalStrategy.h"

class MPComparer : public MPStrategyMethod {
  
public: 
  MPComparer(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("MPComparer::MPComparer()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~MPComparer::MPComparer()");
  };
  
  
  
  virtual void PrintOptions(ostream& out_os) { };
  
  virtual void ParseXML(TiXmlNode* in_pNode) {
    LOG_DEBUG_MSG("MPComparer::ParseXML()");
    
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "input") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        if(in_char) {
          string strategy(in_char);
          m_input_methods.push_back(strategy);
        }
      } else if (string(pChild->Value()) == "comparer_method") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        LOG_DEBUG_MSG("MPComparer::ParseXML() -- comparer_method");
        if (in_char) {
          string evaluator_method(in_char);
          m_comparer_methods.push_back(evaluator_method);
        }
      } else {
        LOG_WARNING_MSG("MPComparer::  I don't know: "<< endl << *pChild);
      }
    }
      
    LOG_DEBUG_MSG("~MPComparer::ParseXML()");
  }

  virtual void operator()(int in_RegionID) { 

  }
   
  // @todo make the parameter be a vector<int> in_region_ids and loop through it to map regions
  virtual void operator()(int in_RegionID_1, int in_RegionID_2) { 
    OBPRM_srand(getSeed()); 
    // mapping region 1
    LOG_DEBUG_MSG("MPComparer::() -- executing "<< m_input_methods[0]);
    MPStrategyMethod* input1 = GetMPProblem()->GetMPStrategy()->
        GetMPStrategyMethod(m_input_methods[0]);
    (*input1)(in_RegionID_1);

    // mapping region 2
    LOG_DEBUG_MSG("MPComparer::() -- executing "<< m_input_methods[1]);
    MPStrategyMethod* input2 = GetMPProblem()->GetMPStrategy()->
        GetMPStrategyMethod(m_input_methods[1]);
    (*input2)(in_RegionID_2); 

    // comparing region 1 to region 2 with each comparer
    typedef vector<string>::iterator Itrtr;
    for (Itrtr itrtr = m_comparer_methods.begin(); itrtr < m_comparer_methods.end(); itrtr++) {
      MPRegionComparerMethod< CfgType, WeightType > * region_comparer;
      region_comparer = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetComparerMethod(*itrtr);
      region_comparer->Compare(in_RegionID_1, in_RegionID_2);
    }

  }

  virtual void operator()() {
    LOG_DEBUG_MSG("MPComparer::()");
    
    int Input1RegionId = GetMPProblem()->CreateMPRegion();
    int Input2RegionId = GetMPProblem()->CreateMPRegion();
    
    (*this)(Input1RegionId, Input2RegionId);

    LOG_DEBUG_MSG("MPComparer::()");

  }
  
  private:
  vector<string> m_input_methods;
  vector<string> m_comparer_methods;
};



class MPMultiStrategy : public MPStrategyMethod {
  
public: 
  MPMultiStrategy(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("MPMultiStrategy::MPMultiStrategy()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~MPMultiStrategy::MPMultiStrategy()");
  };
  
  
  
  virtual void PrintOptions(ostream& out_os) { };
  
  virtual void ParseXML(TiXmlNode* in_pNode) {
    LOG_DEBUG_MSG("MPMultiStrategy::ParseXML()");
    
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if (string(pChild->Value()) == "strategy") {
        const char* in_char = pChild->ToElement()->Attribute("Method");
        LOG_DEBUG_MSG("MPMultiStrategy::ParseXML() -- strategy_method");
        if (in_char) {
          string strategy(in_char);
          m_strategy_methods.push_back(strategy);
        }
      } else {
        LOG_WARNING_MSG("MPMultiStrategy::  I don't know: "<< endl << *pChild);
      }
    }
      
    LOG_DEBUG_MSG("~MPMultiStrategy::ParseXML()");
  }

  virtual void operator()(int in_RegionID) { 

    // initializing region from input
    typedef vector< string >::iterator VITRTR;
    for (VITRTR s_itrtr = m_strategy_methods.begin(); s_itrtr < m_strategy_methods.end(); s_itrtr++) { 
      LOG_DEBUG_MSG("MPMultiStrategy::() -- executing "<< (*s_itrtr));
      MPStrategyMethod* strategy = GetMPProblem()->GetMPStrategy()->
        GetMPStrategyMethod(*s_itrtr);
      (*strategy)(in_RegionID);
    }
  }
   

  virtual void operator()() {
    LOG_DEBUG_MSG("MPMultiStrategy::()");
    
    int RegionId = GetMPProblem()->CreateMPRegion();    
    (*this)(RegionId);

    LOG_DEBUG_MSG("MPMultiStrategy::()");

  }
  
  private:
  vector< string > m_strategy_methods;
};





#endif
