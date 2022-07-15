#ifndef PPL_NBS_H_
#define PPL_NBS_H_

#include <functional>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>
// #include "Utilities/CBS.h"


template <typename TaskType, typename SolutionType, typename PlanType>
using RelaxedPlanFunction =
  std::function<void(TaskType* _decomp)>;

template <typename TaskType, typename SolutionType, typename PlanType>
using RelaxedCostFunction =
  std::function<double()>;

template <typename TaskType, typename SolutionType, typename PlanType>
using ConstrainedPlanFunction =
  std::function<void(SolutionType& _node)>;

template <typename TaskType, typename SolutionType, typename PlanType>
using ConstrainedCostFunction =
  std::function<double(SolutionType& _node)>;


template <typename TaskType, typename SolutionType, typename PlanType>
void
NBS(
  PlanType& original_decomp,
  SolutionType& bestNode,
  RelaxedPlanFunction<TaskType,SolutionType,PlanType>& RelaxedPlanner,
  RelaxedCostFunction<TaskType,SolutionType,PlanType>& RelaxedCost,
  ConstrainedPlanFunction<TaskType,SolutionType,PlanType>& ConstrainedPlan,
  ConstrainedCostFunction<TaskType,SolutionType,PlanType>& ConstrainedCost)
{

  // Initialize bounds
  double lowerBound = 0;
  double upperBound = MAX_DBL;
  
  while(true) {
		RelaxedPlanner(original_decomp);		
		lowerBound = RelaxedCost();

		if (upperBound < lowerBound)
			break;

		ConstrainedPlan(bestNode);
		upperBound = ConstrainedCost(bestNode);
  }
}




#endif
