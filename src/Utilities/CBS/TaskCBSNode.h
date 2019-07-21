#ifndef TASK_CBS_NODE_H_
#define TASK_CBS_NODE_H_

#include "Utilities/CBS/NewCBSNode.h"
#include "Utilities/CBS/TaskConflict.h"
#include "TMPLibrary/TaskPlan.h"


template <typename T, typename U>
class TaskCBSNode : public NewCBSNode<T, U> {
  public:

    ///@name Construction
    ///@{

    TaskCBSNode(TaskPlan* _plan, TaskPlan* _basePlan);

    TaskCBSNode(TaskCBSNode<WholeTask, OccupiedInterval>* _parentNode,
        TaskPlan* _plan, T* _toReplan);

    //@}
    ///@name Accessors
    ///@{

    double GetCost() const override;

    TaskPlan* GetTaskPlan();

    void SetTaskPlan(TaskPlan* _p);

    ///@}
    ///@name Helpers
    ///@{

    void CreateRAT();

    void AddConflict(T* _t, NewConflict<U>* _c);

    ///@}

  private:

    ///@name Internal State
    ///@{

    TaskPlan* m_plan; ///< Node's task plan

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template<typename T, typename U>
TaskCBSNode<T, U>::
TaskCBSNode(TaskPlan* _plan, TaskPlan* _basePlan){

  m_plan = new TaskPlan();

  *m_plan = *_basePlan;
  m_plan->InitializeRAT();

  m_plan->GetTIM() = _plan->GetTIM();
  m_plan->GetTaskCostMap() = _plan->GetTaskCostMap();

  //delete later
  std::cout << "ToReplan: " << this->m_toReplan << std::endl;
  for(auto& ti : m_plan->GetTIM()){
    std::cout << std::endl <<  "WholeTask Pointer: " << ti.first << std::endl;
    for(auto interval : ti.second){
      std::cout << interval.Print() << std::endl;
    }
  }

  for(auto& kv : _plan->GetSubtaskMap()){
    m_plan->SetWholeTaskOwner(kv.first,kv.second);
  }

}

template<typename T, typename U>
TaskCBSNode<T, U>::
TaskCBSNode(TaskCBSNode<WholeTask, OccupiedInterval>* _parentNode,
    TaskPlan* _basePlan, T* _toReplan){
  // copy base plan
  m_plan = new TaskPlan();
  *m_plan = *_basePlan;
  m_plan->InitializeRAT();

  for(auto& kv : _parentNode->GetTaskPlan()->GetSubtaskMap()){
    m_plan->SetWholeTaskOwner(kv.first,kv.second);
  }

  this->m_toReplan = _toReplan;

  // copy parent's tim, and remove information
  m_plan->GetTIM() = _parentNode->GetTaskPlan()->GetTIM();
  m_plan->ClearTaskIntervals(this->m_toReplan);

  //delete later
  std::cout << "ToReplan: " << this->m_toReplan << std::endl;
  for(auto& ti : m_plan->GetTIM()){
    std::cout << std::endl <<  "WholeTask Pointer: " << ti.first << std::endl;
    for(auto interval : ti.second){
      std::cout << interval.Print() << std::endl;
    }
  }

  m_plan->GetTaskCostMap() = _parentNode->GetTaskPlan()->GetTaskCostMap();

  *(this->m_conflicts) = *(_parentNode->GetConflicts());
}

/*-------------------------------- Accessors ---------------------------------*/

template <typename T, typename U>
double
TaskCBSNode<T, U>::
GetCost() const{
  return m_plan->GetEntireCost();
}

template<typename T, typename U>
TaskPlan*
TaskCBSNode<T, U>::
GetTaskPlan(){
  return m_plan;
}

template<typename T, typename U>
void
TaskCBSNode<T, U>::
SetTaskPlan(TaskPlan* _plan){
  m_plan = _plan;
}

/*-------------------------------- Helpers -----------------------------------*/

template<typename T, typename U>
void
TaskCBSNode<T, U>::
CreateRAT(){

  //if(!((m_plan->GetRAT()).empty())){
  //  throw RunTimeException(WHERE) << "Cannot create new RAT for current node. "
  //    << "Current node within CBS tree already has an assigned RAT. ";
  //}

  std::list<NewConflict<U>*> conflicts = (*(this->m_conflicts))[this->m_toReplan];

  for (NewConflict<U>* c : conflicts){
    TaskConflict<U>* t = static_cast<TaskConflict<U>*>(c);
    m_plan->UpdateRAT(t->GetAgent(), t->GetConstraint());

  }
}

template<typename T, typename U>
void
TaskCBSNode<T, U>::
AddConflict(T* _t, NewConflict<U>* _c){
  std::list<NewConflict<U>*> current = (*(this->m_conflicts))[_t];
  current.push_back(_c);
  current.sort();
  (*(this->m_conflicts))[_t] = current;
}


#endif
