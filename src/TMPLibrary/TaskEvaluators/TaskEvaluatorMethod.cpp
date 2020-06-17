#include "TaskEvaluatorMethod.h"

#include "TMPLibrary/TaskPlan.h"
#include "Simulator/Simulation.h"
#include "Utilities/MetricUtils.h"
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
	this->GetTaskPlan()->GetStatClass()->StartClock("TaskEvaluationTime");
	auto ret = Run( _wholeTasks, _plan);
	this->GetTaskPlan()->GetStatClass()->StopClock("TaskEvaluationTime");
	return ret;
}

bool
TaskEvaluatorMethod::
Run(std::vector<WholeTask*> _wholeTasks, std::shared_ptr<TaskPlan> _plan){
	return false;
}
