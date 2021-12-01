#include "NextBestSearch.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "TMPLibrary/Solution/Plan.h"

/*----------------------- Construction -----------------------*/

NextBestSearch::
NextBestSearch() {
  this->SetName("NextBestSearch");
}

NextBestSearch::
NextBestSearch(XMLNode& _node) : TMPStrategyMethod(_node) {
  this->SetName("NextBestSearch");
}

NextBestSearch::
~NextBestSearch() {}

/*------------------------ Interface -------------------------*/

/*--------------------- Helper Functions ---------------------*/

void
NextBestSearch::
PlanTasks() {

  auto plan = this->GetPlan();
  auto originalDecomp = plan->GetDecomposition();

  // Initialize bounds
  double bestCost = MAX_DBL;
  double lowerBound = 0;

  //TODO::Store set of solutions
  // solutions = ?

  while(bestCost > lowerBound) {
    lowerBound = FindTaskPlan(originalDecomp);

    if(bestCost > lowerBound) {
      ComputeMotions(bestCost);
    }

    // TODO::Store solution in solution set
  }

  // TODO::Save plan in proper format
}
    
double
NextBestSearch::
FindTaskPlan(Decomposition* _decomp) {
  return 0;
}

double
NextBestSearch::
ComputeMotions(double&  _bestCost) {

  // TODO::Take decomp from Plan and compute motions

  // TODO::Check if new cost is better than input best cost

  // TODO::If so, replace task solutions in Plan and return new cost
  //       otherwise, return MAX_DBL

  return 0;
}

/*------------------------------------------------------------*/
