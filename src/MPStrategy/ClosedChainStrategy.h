#ifndef ClosedChainStrategy_h
#define ClosedChainStrategy_h

#include "MPStrategy/MPStrategy.h"
#include "MPProblem/ClosedChainProblem.h"
#include "MetricUtils.h"
#include "CDInfo.h"

class ClosedChainStrategy : public MPStrategy
{
 private:
  ClosedChainProblem *CCProblem;
  int num_iterations;
  bool is_gamma_random;
  double gamma;
  StatClass Stats;
  CDInfo _cdInfo;
 public:
  typedef RoadmapGraph<CfgType, WeightType>::VID VID;
  ClosedChainStrategy(XMLNodeReader& in_Node, ClosedChainProblem* in_pProblem);
  ~ClosedChainStrategy();

  void ParseXML(XMLNodeReader& in_Node);
  //void PrintOptions(ostream& out_os);

  /*
  void operator()()
  {
    (*this)(GetMPProblem()->CreateMPRegion());
  }
  void operator()(int in_RegionID);
  virtual void Solve();
  */
};

#endif
