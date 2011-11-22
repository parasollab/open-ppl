#include "NegateSampler.h"
#include "MPStrategy.h"
#include "Sampler.h"
#include "LocalPlanners.h"

boost::shared_ptr<SamplerMethod<CfgType> > GetSamplingMethod(MPProblem* mps, string s){
  return mps->GetMPStrategy()->GetSampler()->GetSamplingMethod(s);
};

boost::shared_ptr<LocalPlannerMethod<CfgType, WeightType> > GetLPMethod(MPProblem* mp, string s){
  return mp->GetMPStrategy()->GetLocalPlanners()->GetLocalPlannerMethod(s);
};
