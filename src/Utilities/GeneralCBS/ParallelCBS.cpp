#include "ParallelCBS.h"

CBSSolution
ParallelConflictBasedSearch(Decomposition* _decomposition, InitialPlanFunction _initial, 
										ValidationFunction _validation, StatClass* _statClass, 
										size_t _numIterations, bool _debug) {


	//Hannah - Change whatever you need here and add whatever parameters you think are necessary.
	//         It is currently using sum-of-cost. If you want to switch to make span, uncomment 
	//         the override in the GetCost function in GeneralCBSNode (in GeneralCBS.cpp line 196
	//         as of the writing of this comment).

	//Regular CBS call for demo puposes
	return ConflictBasedSearch(_decomposition, _initial, _validation, _statClass, _numIterations, _debug);
}
