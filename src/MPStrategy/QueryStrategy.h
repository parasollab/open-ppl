#ifndef QueryStrategy_h
#define QueryStrategy_h

#include "MPStrategy/MPStrategyMethod.h"
#include "MPStrategy/Query.h"

class QueryStrategy : public MPStrategyMethod 
{
 public:   
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
  string dm_label;
  Query<CfgType, WeightType> query;
  ConnectMap<CfgType, WeightType> m_ConnectMap;
};

#endif

