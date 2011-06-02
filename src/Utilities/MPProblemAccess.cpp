#include "NegateSampler.h"
#include "MPStrategy.h"
#include "Sampler.h"

boost::shared_ptr<SamplerMethod<CfgType> > GetSamplingMethod(MPProblem* mps, string s){
  return mps->GetMPStrategy()->GetSampler()->GetSamplingMethod(s);
};
