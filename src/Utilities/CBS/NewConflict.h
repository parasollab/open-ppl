#ifndef NEW_CONFLICT_H_
#define NEW_CONFLICT_H_

#include "MPLibrary/MPBaseObject.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "TMPLibrary/TaskPlan.h"
#include "Behaviors/Agents/Agent.h"

template <typename T>
class NewConflict{
  public:
    typedef T Constraint;

    ///@name Construction
    ///@{

    NewConflict();

    NewConflict(T _constraint);

    ///@}
    ///@name Accessors
    ///@{

    T GetConstraint();

    ///@}
    ///@name Helpers
    //@{

    bool operator<(NewConflict<T>* _c);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    Constraint m_constraint; ///< Templated constraint that is the duration of the conflict

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template<typename T>
NewConflict<T>::
NewConflict(){}

template<typename T>
NewConflict<T>::
NewConflict(T _constraint){
  m_constraint = _constraint;
}

/*-------------------------------- Accessors ---------------------------------*/

template <typename T>
T
NewConflict<T>::
GetConstraint(){
  return m_constraint;
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename T>
bool
NewConflict<T>::
operator<(NewConflict<T>* _c){
  return m_constraint < _c->GetConstraint();
}

#endif
