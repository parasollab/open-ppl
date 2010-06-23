#ifndef MPStrategy_h
#define MPStrategy_h

#include "util.h"
#include "CfgTypes.h"

class MPProblem;
template <class CFG, class WEIGHT> class LocalPlanners;
template <class CFG> class Sampler;
template <class CFG, class WEIGHT> class ConnectMap;
template <class CFG, class WEIGHT> class MPCharacterizer;
template <class CFG, class WEIGHT> class MapEvaluator;
class MPStrategyMethod;


class MPStrategy : public MPBaseObject
{
public: 
  MPStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml = true);
  virtual ~MPStrategy () {}
  
  virtual void ParseStrategyMethod(XMLNodeReader& in_Node);
  
  LocalPlanners<CfgType, WeightType>* GetLocalPlanners() {return m_pLocalPlanners;};
  Sampler<CfgType>* GetSampler() {return m_pNodeGeneration;};
  ConnectMap<CfgType, WeightType>* GetConnectMap(){return m_pConnection;};
  MPCharacterizer<CfgType, WeightType>* GetCharacterizer(){return m_pCharacterizer;};
  MapEvaluator< CfgType, WeightType > * GetMapEvaluator() { return m_Evaluator;}; 

  bool addPartialEdge, addAllEdges; //move to connect map class
  void PrintOptions(ostream& out_os);
  void Solve(); 
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

  vector< MPStrategyMethod* > all_MPStrategyMethod;
  string m_strController_MPStrategyMethod;
};

#include "MPStrategy/PRMStrategy.h"
#include "MPStrategy/RoadmapToolsStrategy.h"
#include "MPStrategy/PRMIncrementalStrategy.h"
#include "MPStrategy/IncrementalPRMStrategy.h"
#include "MPStrategy/HybridPRM.h"
#include "MPStrategy/ExpanderStats.h"
#include "MPStrategy/TimingStats.h"
#include "MPStrategy/NFComparer.h"
#include "MPStrategy/BandsStrategy.h"
#include "MPStrategy/QueryStrategy.h"
#include "MPStrategy/SmoothQueryStrategy.h"


class MPComparer : public MPStrategyMethod {
  
public: 
  MPComparer(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~MPComparer() {}
  
  virtual void PrintOptions(ostream& out_os);  
  virtual void ParseXML(XMLNodeReader& in_Node);

  virtual void operator()(int in_RegionID);   
  virtual void operator()(int in_RegionID_1, int in_RegionID_2);
  virtual void operator()();
  
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
  
  private:
  vector< string > m_strategy_methods;
};

#endif
