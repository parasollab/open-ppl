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
   
  virtual void operator()() 
  {
    int regionID = GetMPProblem()->CreateMPRegion(); 
    GetMPProblem()->GetMPRegion(regionID)->GetRoadmap()->ReadRoadmapGRAPHONLY(m_strMapFileLabel.c_str());
    (*this)(regionID); 
  }
  virtual void operator()(int in_RegionID); 

 protected:
  string m_strSmoothPathFileLabel;
  vector<string> m_vecStrSmoothNodeConnectionLabels;

  ConnectMap<CfgType, WeightType> m_SmoothConnectMap;
};

#endif

