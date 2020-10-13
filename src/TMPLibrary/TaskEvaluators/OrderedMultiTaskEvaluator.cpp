#include "OrderedMultiTaskEvaluator.h"

#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/TMPTools/MultiAgentDijkstra.h"
#include "TMPLibrary/TMPTools/TMPTools.h"
#include "TMPLibrary/WholeTask.h"

OrderedMultiTaskEvaluator::
OrderedMultiTaskEvaluator() {
  this->SetName("OrderedMultiTaskEvaluator");
}

OrderedMultiTaskEvaluator::
OrderedMultiTaskEvaluator(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("OrderedMultiTaskEvaluator");
  m_madLabel = _node.Read("madLabel", true, "", "MultiAgentDijkstra search to use.");
}

OrderedMultiTaskEvaluator::
~OrderedMultiTaskEvaluator(){}

bool
OrderedMultiTaskEvaluator::
Run(std::vector<WholeTask*> _wholeTasks, std::shared_ptr<TaskPlan> _plan){
  if(_wholeTasks.empty()){
    _wholeTasks = this->GetTaskPlan()->GetWholeTasks();
  }

  std::shared_ptr<TaskPlan> savedPlan = nullptr;
  if(_plan){
    savedPlan = this->GetTaskPlan();
    this->GetTMPLibrary()->SetTaskPlan(_plan);
  }

  for(auto& wholeTask : _wholeTasks){

    //TaskPlan* taskPlan = MAMTDijkstra(wholeTask,query.first,query.second);
    this->GetTMPTools()->GetMultiAgentDijkstra(m_madLabel)->Run(wholeTask);
    //*this->GetTaskPlan() = *taskPlan;
    //m_taskPlans.push_back(taskPlan);
  }
  //TODO::Compress into a singular taskplan
  if(savedPlan)
    this->GetTMPLibrary()->SetTaskPlan(savedPlan);
  return true;
}
