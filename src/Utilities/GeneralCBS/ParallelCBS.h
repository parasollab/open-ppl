#ifndef _PPL_PARALLEL_CBS_H_
#define _PPL_PARALLEL_CBS_H_

#include "GeneralCBS.h"

CBSSolution
ParallelConflictBasedSearch(Decomposition* _decomposition, InitialPlanFunction _initial, 
										ValidationFunction _validation, StatClass* _statClass, 
										size_t _numIterations, bool _debug);

#endif
