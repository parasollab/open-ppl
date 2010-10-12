#ifndef SmoothQueryStrategy_h
#define SmoothQueryStrategy_h

#include "MPStrategy/QueryStrategy.h"

class SmoothQueryStrategy : public QueryStrategy 
{
 public:   
  SmoothQueryStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem); 
  virtual ~SmoothQueryStrategy();

  virtual void PrintOptions(ostream& out_os);
  
  virtual void ParseXML(XMLNodeReader& in_Node);

   virtual void Initialize(int in_RegionID){}
   virtual void Run(int in_RegionID);
   virtual void Finalize(int in_RegionID){}

 protected:
  string m_strSmoothPathFileLabel;
  vector<string> m_vecStrSmoothNodeConnectionLabels;
  string dm_label;
  ConnectMap<CfgType, WeightType> m_SmoothConnectMap;
};

#endif

