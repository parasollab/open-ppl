#ifndef MPStrategy_h
#define MPStrategy_h

#include "MPUtils.h"
#include "CfgTypes.h"

class MPProblem;
template <class CFG, class WEIGHT> class LocalPlanners;
template <class CFG> class Sampler;
template <class CFG, class WEIGHT> class Connector;
#ifndef _PARALLEL
template <class CFG, class WEIGHT> class MPCharacterizer;
template <class CFG, class WEIGHT> class MapEvaluator;
class PartitioningMethods;
class PartitioningEvaluators;
class Features;
#endif
class MPStrategyMethod;

class MPStrategy : public MPBaseObject
{
public:
/*  MPStrategy(Sampler<CfgType> _m_pNodeGeneration, Connector<CfgType, WeightType> _m_pConnection, LocalPlanners<CfgType, WeightType> _m_pLocalPlanners, MapEvaluator<CfgType, WeightType> _m_Evaluator, MPCharacterizer<CfgType, WeightType> _m_pCharacterizer, Features* _m_Features, PartitioningMethods* _m_PartitioningMethods, PartitioningEvaluators* _m_PartitioningEvaluators);
 */
  MPStrategy(){}
  MPStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml = true);
  virtual ~MPStrategy () {}
  
  virtual void ParseStrategyMethod(XMLNodeReader& in_Node);
   MPStrategyMethod* CreateMPStrategyMethod(XMLNodeReader& citr);

  LocalPlanners<CfgType, WeightType>* GetLocalPlanners() {return m_pLocalPlanners;};
  void SetLocalPlanners(LocalPlanners<CfgType, WeightType>* _lp) {m_pLocalPlanners = _lp;};
  Sampler<CfgType>* GetSampler() {return m_pNodeGeneration;};
  void SetSamplers(Sampler<CfgType>* _s){m_pNodeGeneration = _s;};
  Connector<CfgType, WeightType>* GetConnector(){return m_pConnection;};
  void SetConnectors(Connector<CfgType, WeightType>* _c){m_pConnection = _c;};
  #ifndef _PARALLEL
  MPCharacterizer<CfgType, WeightType>* GetCharacterizer(){return m_pCharacterizer;};
  MapEvaluator< CfgType, WeightType > * GetMapEvaluator() { return m_Evaluator;}; 
  PartitioningMethods* GetPartitioningMethods(){return m_PartitioningMethods;}
  PartitioningEvaluators* GetPartitioningEvaluators(){return m_PartitioningEvaluators;}
  Features* GetFeatures(){return m_Features;}
  #endif
  XMLNodeReader* GetXMLNodeForStrategy(string& s);
  

  bool addPartialEdge, addAllEdges; //move to connect map class
  void PrintOptions(ostream& out_os);
  virtual void Solve(); 
  MPStrategyMethod* GetMPStrategyMethod(string& );////////////////////////
  ///@ToDo Move addPartialEdge, addAllEdges to Connector

  virtual void SetMPProblem(MPProblem* _mp);
protected:
  Sampler<CfgType>* m_pNodeGeneration;
  Connector<CfgType, WeightType>* m_pConnection;
  LocalPlanners<CfgType, WeightType>* m_pLocalPlanners;
  
  //Characterization and Filtering
#ifndef _PARALLEL
  MPCharacterizer<CfgType, WeightType>* m_pCharacterizer;
  
  //Map_Evaluation
  MapEvaluator<CfgType, WeightType>* m_Evaluator;
  
  //UAS items
  PartitioningMethods* m_PartitioningMethods;
  PartitioningEvaluators* m_PartitioningEvaluators;
  Features* m_Features;
#endif

  vector< pair<MPStrategyMethod*, XMLNodeReader*> > all_MPStrategyMethod;
  string m_strController_MPStrategyMethod;
};

#include "MPStrategyMethod.h"


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
