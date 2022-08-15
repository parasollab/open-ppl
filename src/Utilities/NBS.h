#ifndef PPL_NBS_H_
#define PPL_NBS_H_

template <typename TaskType, typename PlanType>
using RelaxedPlanFunction =
  std::function<PlanType*(TaskType* _task)>;

template <typename PlanType>
using RelaxedCostFunction =
  std::function<double(PlanType* _plan)>;

template <typename SolutionType, typename PlanType>
using ConstrainedPlanFunction =
  std::function<SolutionType*(PlanType* _plan)>;

template <typename SolutionType>
using ConstrainedCostFunction =
  std::function<double(SolutionType* _solution)>;


template <typename TaskType, typename SolutionType, typename PlanType>
void
NBS(
  TaskType* _task,
  SolutionType* _solution,
  RelaxedPlanFunction<TaskType, PlanType>& _relaxedPlanner,
  RelaxedCostFunction<PlanType>& _relaxedCost,
  ConstrainedPlanFunction<SolutionType, PlanType>& _constrainedPlanner,
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

    auto solution = _constrainedPlanner(relaxedPlan);
    double solutionCost = _constrainedCost(solution);

    if(solutionCost < upperBound) {
      _solution = solution;
      upperBound = solutionCost;
    }
  }
}

#endif
