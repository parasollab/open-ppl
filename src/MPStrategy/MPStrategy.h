#ifndef MPStrategy_h
#define MPStrategy_h

#include "util.h"
#include "CfgTypes.h"

#ifdef UAS
#include "PartitioningMethods.h"
#include "PartitioningEvaluators.h"
#include "Features.h"
#endif

class MPProblem;
template <class CFG, class WEIGHT> class LocalPlanners;
template <class CFG> class Sampler;
template <class CFG, class WEIGHT> class ConnectMap;
#ifndef _PARALLEL
template <class CFG, class WEIGHT> class MPCharacterizer;
template <class CFG, class WEIGHT> class MapEvaluator;
#endif
class MPStrategyMethod;

#ifdef UAS
class PartitioningMethods;
class PartitioningEvaluators;
class Features;
#endif

class MPStrategy : public MPBaseObject
{
public:
/*  MPStrategy(Sampler<CfgType> _m_pNodeGeneration, ConnectMap<CfgType, WeightType> _m_pConnection, LocalPlanners<CfgType, WeightType> _m_pLocalPlanners, MapEvaluator<CfgType, WeightType> _m_Evaluator, MPCharacterizer<CfgType, WeightType> _m_pCharacterizer, Features* _m_Features, PartitioningMethods* _m_PartitioningMethods, PartitioningEvaluators* _m_PartitioningEvaluators);
 */
  MPStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml = true);
  virtual ~MPStrategy () {}
  
  virtual void ParseStrategyMethod(XMLNodeReader& in_Node);
   MPStrategyMethod* CreateMPStrategyMethod(XMLNodeReader& citr);

  LocalPlanners<CfgType, WeightType>* GetLocalPlanners() {return m_pLocalPlanners;};
  Sampler<CfgType>* GetSampler() {return m_pNodeGeneration;};
  ConnectMap<CfgType, WeightType>* GetConnectMap(){return m_pConnection;};
  #ifndef _PARALLEL
  MPCharacterizer<CfgType, WeightType>* GetCharacterizer(){return m_pCharacterizer;};
  MapEvaluator< CfgType, WeightType > * GetMapEvaluator() { return m_Evaluator;}; 
  #endif

#ifdef UAS
  PartitioningMethods* GetPartitioningMethods(){return m_PartitioningMethods;}
  PartitioningEvaluators* GetPartitioningEvaluators(){return m_PartitioningEvaluators;}
  Features* GetFeatures(){return m_Features;}

  XMLNodeReader* GetXMLNodeForStrategy(string& s);
#endif

  bool addPartialEdge, addAllEdges; //move to connect map class
  void PrintOptions(ostream& out_os);
  virtual void Solve(); 
  MPStrategyMethod* GetMPStrategyMethod(string& );////////////////////////
  ///@ToDo Move addPartialEdge, addAllEdges to ConnectMap
 protected:
  Sampler<CfgType>* m_pNodeGeneration;
  ConnectMap<CfgType, WeightType>* m_pConnection;
  LocalPlanners<CfgType, WeightType>* m_pLocalPlanners;
  
  //Characterization and Filtering
  #ifndef _PARALLEL
  MPCharacterizer<CfgType, WeightType>* m_pCharacterizer;
  
  //Map_Evaluation
  MapEvaluator<CfgType, WeightType>* m_Evaluator;
  #endif

#ifdef UAS
  //UAS items
  PartitioningMethods* m_PartitioningMethods;
  PartitioningEvaluators* m_PartitioningEvaluators;
  Features* m_Features;
#endif

  vector< pair<MPStrategyMethod*, XMLNodeReader*> > all_MPStrategyMethod;
  string m_strController_MPStrategyMethod;
};
#ifdef _PARALLEL
#include "MPStrategy/ParallelPRMStrategy.h"
#else
#include "MPStrategy/BasicPRMStrategy.h"
#include "MPStrategy/BasicRRTStrategy.h"
#include "MPStrategy/ProbabilityPRMStrategy.h"
#include "MPStrategy/TogglePRMStrategy.h"
#include "MPStrategy/RoadmapToolsStrategy.h"
#include "MPStrategy/HybridPRM.h"
#include "MPStrategy/ExpanderStats.h"
#include "MPStrategy/TimingStats.h"
#include "MPStrategy/NFComparer.h"
#include "MPStrategy/BandsStrategy.h"
#include "MPStrategy/QueryStrategy.h"
#include "MPStrategy/SmoothQueryStrategy.h"
#endif

#ifdef UAS
#include "MPStrategy/UAStrategy.h"
#endif

class MPCContainer : public MPSMContainer {
public:
  MPCContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
   vector<string> m_input_methods;
   vector<string> m_comparer_methods;
   MPSMContainer parent;

};


class MPComparer : public MPStrategyMethod {
 


 
public: 
  MPComparer(MPCContainer cont) : MPStrategyMethod(cont.parent) {
  m_input_methods = cont.m_input_methods;
  m_comparer_methods = cont.m_comparer_methods;

}
  MPComparer(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~MPComparer() {}
  
  virtual void PrintOptions(ostream& out_os);  
  virtual void ParseXML(XMLNodeReader& in_Node);

  virtual void operator()(int in_RegionID);
  virtual void operator()(int in_RegionID_1, int in_RegionID_2);
  
  virtual void operator()();
   virtual void Initialize(int in_RegionID){}
   virtual void Run(int in_RegionID){}
   virtual void Finalize(int in_RegionID){}

  private:
  vector<string> m_input_methods;
  vector<string> m_comparer_methods;
};

class MPMSContainer : public MPSMContainer {
public:
  MPMSContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
  vector< string > m_strategy_methods; 
  MPSMContainer parent;

};


class MPMultiStrategy : public MPStrategyMethod {
  
public: 
  MPMultiStrategy(MPMSContainer cont) : MPStrategyMethod(cont.parent) {
  m_strategy_methods = cont.m_strategy_methods;
}
  MPMultiStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~MPMultiStrategy() {}
  
  virtual void PrintOptions(ostream& out_os);  
  virtual void ParseXML(XMLNodeReader& in_Node);

  virtual void operator()(int in_RegionID);
  virtual void operator()();
    virtual void Initialize(int in_RegionID){}
   virtual void Run(int in_RegionID){}
   virtual void Finalize(int in_RegionID){}

 
  private:
  vector< string > m_strategy_methods;
};

#endif
