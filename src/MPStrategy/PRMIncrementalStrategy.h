#ifndef PRMIncrementalStrategy_h
#define PRMIncrementalStrategy_h


#include "SwitchDefines.h"
#include <sys/time.h>

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

/* util.h defines EXIT used in initializing the environment*/
#include "util.h"
#include "MPProblem.h"
#include "MPCharacterizer.h"

#include "MapEvaluator.h"

#include "MPStrategy/MPStrategyMethod.h"


class PRMIncrementalStrategy : public MPStrategyMethod {
  public:

  PRMIncrementalStrategy(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("PRMIncrementalStrategy::PRMIncrementalStrategy()");
    ParseXML(in_pNode);    
    LOG_DEBUG_MSG("~PRMIncrementalStrategy::PRMIncrementalStrategy()");
    };
  virtual ~PRMIncrementalStrategy() {}

  virtual void PrintOptions(ostream& out_os) { };

  virtual void ParseXML(TiXmlNode* in_pNode);

  virtual void operator()(int in_RegionID);

  virtual void Characterize(MPRegion<CfgType,WeightType>* inout_pRegion, VID in_vid, Stat_Class& Stats);

  virtual void cc_local_area(MPRegion<CfgType,WeightType>* inout_pRegion, VID in_vid, Stat_Class& Stats);
  virtual void merge_node_stats(MPRegion<CfgType,WeightType>* inout_pRegion, VID in_vid, Stat_Class& Stats);

  bool CanConnectComponents(vector < CfgType > & cc_a, vector < CfgType > & cc_b, Stat_Class&);

  pair < unsigned int, unsigned int >
    ConnectionsWitnessToRoadmap(vector < CfgType > & witness_cfgs, Roadmap< CfgType, WeightType > *rdmp, Stat_Class&);

  virtual bool IsFinished();

  double cc_max_length(RoadmapGraph<CfgType,WeightType>* pGraph, VID _cc);
  double cc_diamater(RoadmapGraph<CfgType,WeightType>* pGraph, VID _cc);

  virtual void operator()() {
    int newRegionId = GetMPProblem()->CreateMPRegion();
    (*this)(newRegionId);      
  };

private:
  vector<string> m_vecStrNodeGenLabels;
  vector<string> m_vecStrNodeConnectionLabels;
  vector<string> m_vecStrComponentConnectionLabels;
  vector<string> m_vecNodeCharacterizerLabels;
  string m_strWitnessFilename;
  vector<CfgType> m_vecWitnessNodes;
  
  Stat_Class m_nodeOverheadStat, m_queryStat;
  string m_strBaseFilename;

  int m_totalSamples;
  bool m_query_solved;
   
};



#endif
