#ifndef MPProblemAccess_H_
#define MPProblemAccess_H_

#include "MPProblem.h"
#include "SamplerMethod.h"

boost::shared_ptr<SamplerMethod<CfgType> > GetSamplingMethod(MPProblem* mps, string s);

LocalPlannerMethod<CfgType, WeightType>* GetLPMethod(MPProblem* mp, string s);

#endif
