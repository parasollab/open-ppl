#ifndef DISCRETE_MOTION_CONFLICT_H_
#define DISCRETE_MOTION_CONFLICT_H_

#include "Utilities/CBS/NewConflict.h"
#include "TMPLibrary/WholeTask.h"

template <typename T>
class DiscreteMotionConflict : public NewConflict<T>{
  public:

    ///@name Construction
    ///@{

    DiscreteMotionConflict();

    DiscreteMotionConflict(HandoffAgent* _r, T _c);

    ///@}
    ///@name Accessors
    ///@{

    HandoffAgent* GetAgent();

		size_t GetVID();

		int GetTimeStep();

    ///@}

  private:

    ///@name Internal State
    ///@{

    HandoffAgent* m_agent; ///< Handoff Agent that is causing the conflict
    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename T>
DiscreteMotionConflict<T>::
DiscreteMotionConflict(){ }

template <typename T>
DiscreteMotionConflict<T>::
DiscreteMotionConflict(HandoffAgent* _r, T _c) : NewConflict<T>(_c){  
	m_agent = _r;
}

/*------------------------------- Accessors ----------------------------------*/

template <typename T>
HandoffAgent*
DiscreteMotionConflict<T>::
GetAgent(){
  return m_agent;
}

#endif

