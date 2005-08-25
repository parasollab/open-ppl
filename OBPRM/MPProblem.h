#ifndef MPProblem_h
#define MPProblem_h

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
#include "CfgTypes.h"

class MPStrategy;

typedef Cfg_free CfgType;
typedef DefaultWeight WeightType;

class MPProblem : public MPBaseObject
{
public:
  MPProblem(TiXmlNode* in_pNode);
  
  void PrintOptions();
  
  
private:
  ///\todo Create constructors for distance_metrics, collision_detection, MPRegions
  virtual void ParseXML(TiXmlNode* in_pNode); 
  void ParseXMLFileIO(TiXmlNode* in_pNode);
  
public:
  ///\todo Finish these interfaces.
 // void WriteRoadmap();
  void WriteRoadmapForVizmo();
  void SetMPStrategy(MPStrategy* in_pStrategy) {m_pMPStrategy = in_pStrategy;};
  inline DistanceMetric* GetDistanceMetric() {return m_pDistanceMetric; };
  inline CollisionDetection* GetCollisionDetection() {return m_pCollisionDetection; };
  inline Environment* GetEnvironment() {return m_pEnvironment; };
  inline string& GetEnvFileName() {return m_input_env;};
  inline string& GetOutputRoadmap() {return m_output_map;};
  inline string& GetOutputDir() {return m_output_dir;};
  inline MPStrategy* GetMPStrategy() {return m_pMPStrategy;};
  
  inline Roadmap<CfgType,WeightType>* GetRoadmap() {return &rmp;};
  inline Roadmap<CfgType,WeightType>* GetColRoadmap() {return &rmp_col;};
  void PrintOptions(ostream& out_os);
////////////
//
//Data
//
//////////////
  private:
 
  string m_input_env, m_output_map, m_output_dir;
  
  MPStrategy* m_pMPStrategy;
  DistanceMetric*     m_pDistanceMetric;
  CollisionDetection* m_pCollisionDetection;
  Environment* m_pEnvironment;
  Roadmap<CfgType,WeightType> rmp;
  Roadmap<CfgType,WeightType> rmp_col;
  vector< MPRegion<CfgType,WeightType> > regions; 
   
    
};



#endif
