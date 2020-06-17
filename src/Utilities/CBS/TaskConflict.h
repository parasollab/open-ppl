#ifndef TASK_CONFLICT_H_
#define TASK_CONFLICT_H_

#include "Utilities/CBS/NewConflict.h"
#include "TMPLibrary/WholeTask.h"

template <typename T>
class TaskConflict : public NewConflict<T>{
  public:

    ///@name Construction
    ///@{

    TaskConflict();

    TaskConflict(HandoffAgent* _r, T _c);

    TaskConflict(T _c, HandoffAgent* _r, WholeTask* _t);

    ///@}
    ///@name Accessors
    ///@{

    HandoffAgent* GetAgent();

    WholeTask* GetTask();

    ///@}

  private:

    ///@name Internal State
    ///@{

    HandoffAgent* m_agent; ///< Handoff Agent that is causing the conflict

    WholeTask* m_task{nullptr}; ///< Task that the conflict occus in

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename T>
TaskConflict<T>::
TaskConflict(){ }

template <typename T>
TaskConflict<T>::
TaskConflict(HandoffAgent* _r, T _c) : m_agent(_r), NewConflict<T>(_c){  }

template <typename T>
TaskConflict<T>::
TaskConflict(T _c, HandoffAgent* _r, WholeTask* _t){
 // TODO: make the following a deep copy, there are some errors with the
 // OccupiedInterval copy constructors when doing a deep copy here.
 this->m_constraint = _c;

 m_agent = _r;
 m_task = _t;
}

/*------------------------------- Accessors ----------------------------------*/

template <typename T>
HandoffAgent*
TaskConflict<T>::
GetAgent(){
  return m_agent;
}

template <typename T>
WholeTask*
TaskConflict<T>::
GetTask(){
  return m_task;
}


#endif

