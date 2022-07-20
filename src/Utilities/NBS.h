#ifndef PPL_NBS_H_
#define PPL_NBS_H_

#include <functional>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>
// #include "Utilities/CBS.h"


template <typename TaskType, typename PlanType>
using RelaxedPlanFunction =
	std::function<PlanType*(TaskType* _decomp)>;

template <typename PlanType>
using RelaxedCostFunction =
	std::function<double(PlanType* _plan)>;

template <typename SolutionType>
using ConstrainedPlanFunction =
	std::function<SolutionType(SolutionType& _node)>;

template <typename SolutionType>
using ConstrainedCostFunction =
	std::function<double(SolutionType& _node)>;


template <typename TaskType, typename SolutionType, typename PlanType>
void
NBS(
	TaskType*& _task,
	SolutionType& _solution,
	RelaxedPlanFunction<TaskType, PlanType>& _relaxedPlanner,
	RelaxedCostFunction<PlanType>& _relaxedCost,
	ConstrainedPlanFunction<SolutionType>& _constrainedPlan,
	ConstrainedCostFunction<SolutionType>& _constrainedCost)
{

	// Initialize bounds
	double lowerBound = 0;
	double upperBound = MAX_DBL;
	
	while(true) {
		auto relaxedPlan = _relaxedPlanner(_task);		
		lowerBound = _relaxedCost(relaxedPlan);

		if (upperBound < lowerBound)
			break;

		auto solution = _constrainedPlan(_solution);

		if(solution.cost < _solution.cost)
			_solution = solution;
			upperBound = _constrainedCost(_solution);
	}
}

#endif
