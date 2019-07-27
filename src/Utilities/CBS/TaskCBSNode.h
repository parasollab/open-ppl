#ifndef TASK_CBS_NODE_H_
#define TASK_CBS_NODE_H_

#include "Behaviors/Agents/Coordinator.h"

#include "ConfigurationSpace/RoadmapGraph.h"

#include "TMPLibrary/TaskPlan.h"

#include "Utilities/CBS/NewCBSNode.h"
#include "Utilities/CBS/TaskConflict.h"


template <typename T, typename U>
class TaskCBSNode : public NewCBSNode<T, U> {
  public:

    ///@name Construction
    ///@{

    TaskCBSNode(TaskPlan* _plan, TaskPlan* _basePlan);

    TaskCBSNode(TaskCBSNode<WholeTask, OccupiedInterval>* _parentNode,
        TaskPlan* _plan, T* _toReplan);

		virtual ~TaskCBSNode();

    //@}
    ///@name Accessors
    ///@{

    double GetCost() const override;

    TaskPlan* GetTaskPlan();

    void SetTaskPlan(TaskPlan* _p);

		void UpdateValidVIDs(std::vector<size_t> _invalids, std::vector<size_t> _valids, WholeTask* _wholeTask); 
    ///@}
    ///@name Helpers
    ///@{

    void CreateRAT();

    void AddConflict(T* _t, NewConflict<U>* _c);

    size_t GetDepth();

    std::unordered_map<WholeTask*,std::set<size_t>>& GetValidVIDs();

    ///@}

  private:

    ///@name Internal State
    ///@{

		//TODO::Make all of these shared_ptr
    TaskPlan* m_plan; ///< Node's task plan

    size_t m_depth{0};

    std::unordered_map<WholeTask*,std::set<size_t>> m_validVIDs;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template<typename T, typename U>
TaskCBSNode<T, U>::
TaskCBSNode(TaskPlan* _plan, TaskPlan* _basePlan){

  m_plan = new TaskPlan();

  *m_plan = *_basePlan;
  m_plan->InitializeRAT();

  //m_plan->GetTIM() = _plan->GetTIM();
  for(auto& ti : _plan->GetTIM()) {
    m_validVIDs[ti.first] = {};
    //if(ti.first == this->m_toReplan)
    //  continue;

    m_plan->GetTIM()[ti.first] = {};

    for(auto it = ti.second.begin(); it != ti.second.end(); it++){
      m_plan->UpdateTIM(ti.first,*it);
    }
  }
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

  m_validVIDs = _parentNode->m_validVIDs;

  //m_graph = new RoadmapGraph<Cfg,DefaultWeight<Cfg>>(m_plan->GetCoordinator()->GetRobot());
  //*m_graph = *(_parentNode->m_graph);

  for(auto& kv : _parentNode->GetTaskPlan()->GetSubtaskMap()){
    m_plan->SetWholeTaskOwner(kv.first,kv.second);
  }

  this->m_toReplan = _toReplan;
  m_depth = _parentNode->m_depth+1;

  // copy parent's tim, and remove information
	this->m_toReplan->m_agentAssignment = {};
	this->m_toReplan->m_subtasks = {};
	m_plan->GetTIM()[this->m_toReplan] = {};
  //m_plan->GetTIM() = _parentNode->GetTaskPlan()->GetTIM();
  for(auto& ti : _parentNode->GetTaskPlan()->GetTIM()) {
    if(ti.first == this->m_toReplan)
      continue;
    m_plan->GetTIM()[ti.first] = {};
    for(auto it = ti.second.begin(); it != ti.second.end(); it++){
      m_plan->UpdateTIM(ti.first,*it);
    }
  }
  //m_plan->ClearTaskIntervals(this->m_toReplan);

  //delete later
/*
  std::cout << std::endl << std::endl << "=========================================================" << std::endl;
  std::cout << std::endl << "Parent node's TIM: " << std::endl << std::endl;
  std::cout << "=========================================================" << std::endl;
  for(auto& ti : _parentNode->GetTaskPlan()->GetTIM()){
    std::cout << std::endl <<  "WholeTask Pointer: " << ti.first << std::endl;
    for(auto interval : ti.second){
      std::cout << interval.Print() << std::endl;
    }
  }

  std::cout << std::endl << std::endl << "=========================================================" << std::endl;
  std::cout << "ToReplan: " << this->m_toReplan << std::endl;
  for (auto&ti : m_plan->GetTIM()){
    std::cout << ti.first << std::endl;
  }
  std::cout << "=========================================================" << std::endl;

  for(auto& ti : m_plan->GetTIM()){
    std::cout << std::endl <<  "WholeTask Pointer: " << ti.first << std::endl;
    for(auto interval : ti.second){
      std::cout << interval.Print() << std::endl;
    }
  }
*/
  m_plan->GetTaskCostMap() = _parentNode->GetTaskPlan()->GetTaskCostMap();

  *(this->m_conflicts) = *(_parentNode->GetConflicts());
}


template <typename T, typename U>
TaskCBSNode<T,U>::
~TaskCBSNode(){}
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

template<typename T, typename U>
std::unordered_map<WholeTask*,std::set<size_t>>&
TaskCBSNode<T,U>::
GetValidVIDs(){
  return m_validVIDs;
}

template<typename T, typename U>
size_t
TaskCBSNode<T, U>::
GetDepth() {
  return m_depth;
}

template<typename T, typename U>
void
TaskCBSNode<T,U>::
UpdateValidVIDs(std::vector<size_t> _invalids, std::vector<size_t> _valids, WholeTask* _wholeTask) {
  for(auto& invalid : _invalids) {
    m_validVIDs[_wholeTask].erase(invalid);
  }
  for(auto& valid : _valids) {
   	m_validVIDs[_wholeTask].insert(valid);
  }
}
#endif
