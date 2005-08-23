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

class MPStrategy_method : public MPBaseObject 
{
  //Will be used to derive IMP,PRM,RRT,metaplanner, etc.
};

class MPStrategy : public MPBaseObject
{
public: 
  MPStrategy(TiXmlNode* in_pNode, MPProblem* in_pproblem);
  
  LocalPlanners<CfgType, WeightType>* GetLocalPlanners() {return m_pLocalPlanners;};
  GenerateMapNodes<CfgType>* GetGenerateMapNodes() {return m_pNodeGeneration;};
  ConnectMap<CfgType, WeightType>* GetConnectMap(){return m_pConnection;};
  
  void GenerateMap();  
  ///@ToDo Move addPartialEdge, addAllEdges to ConnectMap
 private:
  MPProblem* m_pProblem;
  GenerateMapNodes<CfgType>* m_pNodeGeneration;
  ConnectMap<CfgType, WeightType>* m_pConnection;
  LocalPlanners<CfgType, WeightType>* m_pLocalPlanners;
  bool addPartialEdge, addAllEdges; //move to connect map class
  //Map_Evaluation
  //Filtering
  //vector< MPStrategy_method >
  //MPStrategy_method* selected method (only 1)
};
#endif
