#ifndef DMTESTSTRATEGY_H_
#define DMTESTSTRATEGY_H_


#include "MPStrategy/MPStrategyMethod.h"
#include "MPProblem/MPProblem.h"
#include "Roadmap.h"

class DMTestStrategy : public MPStrategyMethod 
{
 public:
  DMTestStrategy(XMLNodeReader& _node, MPProblem* _problem);
  virtual ~DMTestStrategy();

  virtual void ParseXML(XMLNodeReader& _node);
  virtual void PrintOptions(ostream& _out);

  virtual void Initialize(int _regionID){}
  virtual void Run(int _regionID);
  virtual void Finalize(int _regionID){}

 private:
  string m_inputRoadmapFilename;
  Roadmap<CfgType, WeightType>* m_rdmp;
  string m_dmMethod;
  size_t m_numToVerify;
};

#endif
