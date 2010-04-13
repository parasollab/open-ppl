#ifndef ClosedChainStrategy_h
#define ClosedChainStrategy_h


#include "MPStrategy/MPStrategy.h"
#include "MPProblem/ClosedChainProblem.h"
//#include "MPProblem/RoadmapGraph.h" //for VID typedef


class ClosedChainStrategy : public MPStrategy
{
 private:
  ClosedChainProblem *CCProblem;
  int num_iterations;
  bool is_gamma_random;
  double gamma;
  Stat_Class Stats;
  CDInfo _cdInfo;
 public:
  typedef RoadmapGraph<CfgType, WeightType>::VID VID;
  ClosedChainStrategy(XMLNodeReader& in_Node, ClosedChainProblem* in_pProblem);
  ~ClosedChainStrategy();

  void ParseXML(XMLNodeReader& in_Node);
  void PrintOptions(ostream& out_os);

  void operator()()
  {
    (*this)(GetMPProblem()->CreateMPRegion());
  }
  void operator()(int in_RegionID);
  void Solve();
};

#endif
