#include "TaskEvaluatorMethod.h"

#include "Simulator/Simulation.h"

/*------------------------------ Construction --------------------------------*/

TaskEvaluatorMethod::
TaskEvaluatorMethod(){}

TaskEvaluatorMethod::
TaskEvaluatorMethod(XMLNode& _node) : TMPBaseObject(_node) {
  m_sgLabel = _node.Read("sgLabel", false, "",
												 "Label for the state graph used by the TMPStrategy");
}

TaskEvaluatorMethod::
~TaskEvaluatorMethod(){}

bool 
TaskEvaluatorMethod::
operator()(std::vector<WholeTask*> _wholeTasks, std::shared_ptr<TaskPlan> _plan){
	Simulation::GetStatClass()->StartClock("TaskEvaluationTime");
	auto ret = Run( _wholeTasks, _plan);
	Simulation::GetStatClass()->StopClock("TaskEvaluationTime");
	return ret;
}

bool
TaskEvaluatorMethod::
Run(std::vector<WholeTask*> _wholeTasks, std::shared_ptr<TaskPlan> _plan){
	return false;
}
