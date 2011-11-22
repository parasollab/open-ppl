#ifndef SmoothQueryStrategy_h
#define SmoothQueryStrategy_h

#include "MPStrategy/QueryStrategy.h"

class  SQSContainer : public QSContainer {
public:
  SQSContainer (QSContainer cont = QSContainer()) : QSContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
  string m_strSmoothPathFileLabel;
  vector<string> m_vecStrSmoothNodeConnectionLabels;
  string dm_label;
  string m_lp_label;
  ConnectMap<CfgType, WeightType> m_SmoothConnectMap;
  MPSMContainer parent;

};


class SmoothQueryStrategy : public QueryStrategy 
{
 public:   
  SmoothQueryStrategy(SQSContainer cont) : QueryStrategy(cont.parent) {
  m_strSmoothPathFileLabel = cont.m_strSmoothPathFileLabel;
  m_vecStrSmoothNodeConnectionLabels = cont.m_vecStrSmoothNodeConnectionLabels;
  dm_label = cont.dm_label;
  m_SmoothConnectMap = cont.m_SmoothConnectMap;

}
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
  string m_lp_label;
  ConnectMap<CfgType, WeightType> m_SmoothConnectMap;
};

#endif

