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
template <class CFG, class WEIGHT> class MPCharacterizer;
template <class CFG, class WEIGHT> class MapEvaluator;
class MPStrategyMethod;

#ifdef UAS
class PartitioningMethods;
class PartitioningEvaluators;
class Features;
#endif

class MPStrategy : public MPBaseObject
{
public: 
  MPStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml = true);
  virtual ~MPStrategy () {}
  
  virtual void ParseStrategyMethod(XMLNodeReader& in_Node);
   MPStrategyMethod* CreateMPStrategyMethod(XMLNodeReader& citr);

  LocalPlanners<CfgType, WeightType>* GetLocalPlanners() {return m_pLocalPlanners;};
  Sampler<CfgType>* GetSampler() {return m_pNodeGeneration;};
  ConnectMap<CfgType, WeightType>* GetConnectMap(){return m_pConnection;};
  MPCharacterizer<CfgType, WeightType>* GetCharacterizer(){return m_pCharacterizer;};
  MapEvaluator< CfgType, WeightType > * GetMapEvaluator() { return m_Evaluator;}; 

#ifdef UAS
  PartitioningMethods* GetPartitioningMethods(){return m_PartitioningMethods;}
  PartitioningEvaluators* GetPartitioningEvaluators(){return m_PartitioningEvaluators;}
  Features* GetFeatures(){return m_Features;}

  XMLNodeReader* GetXMLNodeForStrategy(string& s);
#endif UAS

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
  MPCharacterizer<CfgType, WeightType>* m_pCharacterizer;
  
  //Map_Evaluation
  MapEvaluator<CfgType, WeightType>* m_Evaluator;

#ifdef UAS
  //UAS items
  PartitioningMethods* m_PartitioningMethods;
  PartitioningEvaluators* m_PartitioningEvaluators;
  Features* m_Features;
#endif

  vector< pair<MPStrategyMethod*, XMLNodeReader*> > all_MPStrategyMethod;
  string m_strController_MPStrategyMethod;
};

#include "MPStrategy/BasicPRMStrategy.h"
#include "MPStrategy/ProbabilityPRMStrategy.h"
#include "MPStrategy/RoadmapToolsStrategy.h"
#include "MPStrategy/HybridPRM.h"
#include "MPStrategy/ExpanderStats.h"
#include "MPStrategy/TimingStats.h"
#include "MPStrategy/NFComparer.h"
#include "MPStrategy/BandsStrategy.h"
#include "MPStrategy/QueryStrategy.h"
#include "MPStrategy/SmoothQueryStrategy.h"

#ifdef UAS
#include "MPStrategy/UAStrategy.h"
#endif

class MPComparer : public MPStrategyMethod {
  
public: 
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



class MPMultiStrategy : public MPStrategyMethod {
  
public: 
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
