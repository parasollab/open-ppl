#ifndef RoadmapToolsStrategy_h
#define RoadmapToolsStrategy_h

#include<sys/time.h>

#include "Roadmap.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "Sampler.h"
#include "GeneratePartitions.h"


//#include "ExplicitInstantiation.h"

/* util.h defines PMPL_EXIT used in initializing the environment*/
#include "util.h"
#include "MPProblem.h"
#include "MPCharacterizer.h"

#include "MapEvaluator.h"

#include "MPStrategy/MPStrategyMethod.h"

    class RIContainer : public MPSMContainer {
public:
   RIContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
   string m_strInputFileName;
   MPSMContainer parent;

};


class RoadmapInput : public MPStrategyMethod {
  public:
    
   RoadmapInput(RIContainer cont) : MPStrategyMethod(cont.parent) {
    m_strInputFileName = cont.m_strInputFileName;
} 
   RoadmapInput(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPStrategyMethod(in_Node,in_pProblem) {
    ParseXML(in_Node);    
    };
   

    virtual ~RoadmapInput() {}

    virtual void PrintOptions(ostream& out_os) { };
  
    virtual void ParseXML(XMLNodeReader& in_Node) {
      m_strInputFileName = in_Node.stringXMLParameter("input_map", true, "", "Input roadmap file");
    };
   
   virtual void Initialize(int in_RegionID){}
   virtual void Run(int in_RegionID){
      OBPRM_srand(getSeed()); 
    }
   virtual void Finalize(int in_RegionID){}

  private:
    string m_strInputFileName;
   
};


class RoadmapClear : public MPStrategyMethod {
 public:
  RoadmapClear(MPSMContainer cont) : MPStrategyMethod(cont) {}  
  RoadmapClear(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPStrategyMethod(in_Node,in_pProblem) {
    ParseXML(in_Node);    
    };

    virtual ~RoadmapClear() {}

    virtual void PrintOptions(ostream& out_os) { };
  
    virtual void ParseXML(XMLNodeReader& in_Node) {};
   
   virtual void Initialize(int in_RegionID){}
   virtual void Run(int in_RegionID){
      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
      OBPRM_srand(getSeed()); 
      region->GetRoadmap()->m_pRoadmap->clear();
      region->GetStatClass()->ClearStats();
    }
   virtual void Finalize(int in_RegionID){}


  private:
   
};



#endif
