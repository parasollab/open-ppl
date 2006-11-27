#ifndef MPStrategy_h
#define MPStrategy_h

#include "util.h"
#include "CfgTypes.h"

class MPProblem;
template <class CFG, class WEIGHT> class LocalPlanners;
template <class CFG> class GenerateMapNodes;
template <class CFG, class WEIGHT> class ConnectMap;
template <class CFG, class WEIGHT> class MPCharacterizer;
template <class CFG, class WEIGHT> class MapEvaluator;
class MPStrategyMethod;

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
#include "MPStrategy/HybridPRM.h"

class MPComparer : public MPStrategyMethod {
  
public: 
  MPComparer(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  
  virtual void PrintOptions(ostream& out_os);  
  virtual void ParseXML(TiXmlNode* in_pNode);

  virtual void operator()(int in_RegionID);   
  virtual void operator()(int in_RegionID_1, int in_RegionID_2);
  virtual void operator()();
  
  private:
  vector<string> m_input_methods;
  vector<string> m_comparer_methods;
};



class MPMultiStrategy : public MPStrategyMethod {
  
public: 
  MPMultiStrategy(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  
  virtual void PrintOptions(ostream& out_os);  
  virtual void ParseXML(TiXmlNode* in_pNode);

  virtual void operator()(int in_RegionID);
  virtual void operator()();
  
  private:
  vector< string > m_strategy_methods;
};





#endif
