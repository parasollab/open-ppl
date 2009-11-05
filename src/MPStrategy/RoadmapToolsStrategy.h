#ifndef RoadmapToolsStrategy_h
#define RoadmapToolsStrategy_h




#include "SwitchDefines.h"
#include<sys/time.h>

#include "OBPRMDef.h"
#include "Roadmap.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
#include "Sampler.h"
#include "GeneratePartitions.h"


//#include "ExplicitInstantiation.h"

/* util.h defines PMPL_EXIT used in initializing the environment*/
#include "util.h"
#include "MPProblem.h"
#include "MPCharacterizer.h"

#include "MapEvaluator.h"

#include "MPStrategy/MPStrategyMethod.h"

class RoadmapInput : public MPStrategyMethod {
  public:
    
    
  RoadmapInput(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPStrategyMethod(in_Node,in_pProblem) {
    LOG_DEBUG_MSG("RoadmapInput::RoadmapInput()");
    ParseXML(in_Node);    
    LOG_DEBUG_MSG("~RoadmapInput::RoadmapInput()");
    };
   

    virtual ~RoadmapInput() {}

    virtual void PrintOptions(ostream& out_os) { };
  
    virtual void ParseXML(XMLNodeReader& in_Node) {
      LOG_DEBUG_MSG("RoadmapInput::ParseXML()");
      
      m_strInputFileName = in_Node.stringXMLParameter(string("input_map"), true,
                                      string(""), string("Input roadmap file"));
    
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
    
  RoadmapClear(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPStrategyMethod(in_Node,in_pProblem) {
    LOG_DEBUG_MSG("RoadmapClear::RoadmapClear()");
    ParseXML(in_Node);    
    LOG_DEBUG_MSG("~RoadmapClear::RoadmapClear()");
    };

    virtual ~RoadmapClear() {}

    virtual void PrintOptions(ostream& out_os) { };
  
    virtual void ParseXML(XMLNodeReader& in_Node) {
      LOG_DEBUG_MSG("RoadmapClear::ParseXML()");
    
      LOG_DEBUG_MSG("~RoadmapClear::ParseXML()");
    };
   
    virtual void operator()(int in_RegionID) {
      LOG_DEBUG_MSG("RoadmapClear::() ");
      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
      OBPRM_srand(getSeed()); 
      

      region->GetRoadmap()->m_pRoadmap->clear();
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
