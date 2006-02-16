#ifndef RoadmapToolsStrategy_h
#define RoadmapToolsStrategy_h




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
      LOG_DEBUG_MSG("PRMInput::() -- Reading in file: " << m_strInputFileName);
      OBPRM_srand(getSeed()); 
      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
      
      region->GetRoadmap()->ReadRoadmapGRAPHONLY(m_strInputFileName.c_str());
      
      LOG_DEBUG_MSG("~PRMInput::()");
    }
  
    virtual void operator()() {
      int newRegionId = GetMPProblem()->CreateMPRegion();
      (*this)(newRegionId);      
    };

  private:
    string m_strInputFileName;
   
};


class RoadmapClear : public MPStrategyMethod {
 public:
    
  RoadmapClear(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("RoadmapClear::RoadmapClear()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~RoadmapClear::RoadmapClear()");
    };
    
    virtual void PrintOptions(ostream& out_os) { };
  
    virtual void ParseXML(TiXmlNode* in_pNode) {
      LOG_DEBUG_MSG("RoadmapClear::ParseXML()");
    
      LOG_DEBUG_MSG("~RoadmapClear::ParseXML()");
    };
   
    virtual void operator()(int in_RegionID) {
      LOG_DEBUG_MSG("RoadmapClear::() ");
      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
      OBPRM_srand(getSeed()); 
      

      region->GetRoadmap()->m_pRoadmap->EraseGraph();
      region->GetStatClass()->ClearStats();
      
      LOG_DEBUG_MSG("~RoadmapClear::()");
    }
  
    virtual void operator()() {
      int newRegionId = GetMPProblem()->CreateMPRegion();
      (*this)(newRegionId);      
    };

  private:
   
};



#endif
