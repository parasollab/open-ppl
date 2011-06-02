#ifndef MPProblemAccess_H_
#define MPProblemAccess_H_

#include "MPProblem.h"

boost::shared_ptr<SamplerMethod<CfgType> > GetSamplingMethod(MPProblem* mps, string s);

#endif
