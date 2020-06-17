#ifndef AGENT_ALLOCATION_CONFLICT_H_
#define AGENT_ALLOCATION_CONFLICT_H_

#include "Utilities/CBS/NewConflict.h"
#include "TMPLibrary/WholeTask.h"

template <typename T>
class AgentAllocationConflict : public NewConflict<T>{
  public:

    ///@name Construction
    ///@{

    AgentAllocationConflict();

    AgentAllocationConflict(WholeTask* _r, T _c);

    ///@}
    ///@name Accessors
    ///@{

    WholeTask* GetTask();

    ///@}

  private:

    ///@name Internal State
    ///@{

    WholeTask* m_task; ///< Handoff Agent that is causing the conflict
    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename T>
AgentAllocationConflict<T>::
AgentAllocationConflict(){ }

template <typename T>
AgentAllocationConflict<T>::
AgentAllocationConflict(WholeTask* _t, T _c) : NewConflict<T>(_c){  
	m_task = _t;
}

/*------------------------------- Accessors ----------------------------------*/

template <typename T>
WholeTask*
AgentAllocationConflict<T>::
GetTask(){
  return m_task;
}

#endif

