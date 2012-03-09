#ifndef QueryStrategy_h
#define QueryStrategy_h

#include "MPStrategy/MPStrategyMethod.h"
#include "MPStrategy/Query.h"

class QSContainer : public MPSMContainer {
public:
  QSContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
    vector<string> m_vecStrNodeConnectionLabels;
  //string m_strLocalPlannerLabel;
  string m_strMapFileLabel;
  string m_strQueryFileLabel;
  string m_strPathFileLabel;
  string m_lp_label;
  string dm_label;
  Query<CfgType, WeightType> query;
  Connector<CfgType, WeightType> m_Connector;
  MPSMContainer parent;

};

class QueryStrategy : public MPStrategyMethod 
{
 public:
  QueryStrategy(QSContainer cont) : MPStrategyMethod(cont.parent) {
  m_vecStrNodeConnectionLabels = cont.m_vecStrNodeConnectionLabels;
  //string m_strLocalPlannerLabel;
  m_strMapFileLabel = cont.m_strMapFileLabel;
  m_strQueryFileLabel = cont.m_strQueryFileLabel;
  m_strPathFileLabel = cont.m_strPathFileLabel;
  m_lp_label = cont.m_lp_label;
  dm_label = cont.dm_label;
  query = cont.query;
  m_Connector = cont.m_Connector;
}

  QueryStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml = true);
  virtual ~QueryStrategy();

  virtual void PrintOptions(ostream& out_os);
 
  virtual void ParseXML(XMLNodeReader& in_Node) {
    ParseXML(in_Node, true); 
  }
  virtual void ParseXML(XMLNodeReader& in_Node, bool warn_unknown); 
   
   virtual void Initialize(int in_RegionID){}
   virtual void Run(int in_RegionID);
   virtual void Finalize(int in_RegionID){}

protected:
  vector<string> m_vecStrNodeConnectionLabels;
  //string m_strLocalPlannerLabel;
  string m_strMapFileLabel;
  string m_strQueryFileLabel;
  string m_strPathFileLabel;
  string m_lp_label;
  string dm_label;
  Query<CfgType, WeightType> query;
  Connector<CfgType, WeightType> m_Connector;
};

#endif

