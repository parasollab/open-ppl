#include "SMART.h"

#include "TMPLibrary/StateGraphs/OCMG.h"
#include "TMPLibrary/Solution/Plan.h"

/*------------------------------- Construction -------------------------------*/

SMART::
SMART() {
  this->SetName("SMART");
}

SMART::
SMART(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("SMART");
}

/*---------------------------- Initialization --------------------------------*/

void
SMART::
Initialize() {

}

/*-------------------------------- Helpers -----------------------------------*/

bool
SMART::
Run(Plan* _plan) {
  return false;
}

/*--------------------------- Heuristic Functions ----------------------------*/

SMART::Mode
SMART::
ComputeMAPFHeuristic(Mode _mode) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ComputeMAPFHeuristic");
  
  if(m_cachedHeuristics.find(_mode) != m_cachedHeuristics.end()) {
    return m_cachedHeuristics.at(_mode);
  }

  auto sg = static_cast<OCMG*>(this->GetStateGraph(m_sgLabel).get());
  // auto omg = sg->GetSingleObjectModeGraph();

  // TODO::Get Each object's goal
  // TODO::Cache this

  
  // Configure CBS Functions
  CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution> lowLevel(
    [this](CBSNodeType& _node, Robot* _task) {
      return this->LowLevelPlanner(_node,_task);
    }
  );

  CBSValidationFunction<Robot,CBSConstraint,CBSSolution> validation(
    [this](CBSNodeType& _node) {
      return this->ValidationFunction(_node);
    }
  );

  CBSCostFunction<Robot,CBSConstraint,CBSSolution> cost(
    [this](CBSNodeType& _node) {
      return this->CostFunction(_node);
    }
  );

  CBSSplitNodeFunction<Robot,CBSConstraint,CBSSolution> splitNode(
    [this](CBSNodeType& _node, std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
           CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
           CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost) {
      return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
    }
  );

  std::vector<Robot*> objects = sg->GetObjects();

  CBSNodeType solution = CBS(objects,validation,splitNode,lowLevel,cost);

  if(m_debug) {
    std::cout << "Heuristic Paths" << std::endl;
    for(auto kv : solution.solutionMap) {
      auto robot = kv.first;
      auto path = *kv.second;
      std::cout << "\t" << robot->GetLabel() << ": ";
      for(auto vid : path) {
        std::cout << vid << ", ";
      }
      std::cout << std::endl;
    }
  }

  return Mode();
}

bool
SMART::
LowLevelPlanner(CBSNodeType& _node, Robot* _robot) {

  //TODO::Compute cost to go heuristic

  //TODO::Compute "time" extended path  

  return false;
}

std::vector<std::pair<Robot*,SMART::CBSConstraint>>
SMART::
ValidationFunction(CBSNodeType& _node) {
  return {};
}

double
SMART::
CostFunction(CBSNodeType& _node) {
  return 0;
}

std::vector<SMART::CBSNodeType> 
SMART::
SplitNodeFunction(CBSNodeType& _node,
        std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
        CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
        CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost) {
  return {};
}
/*----------------------------------------------------------------------------*/
