#include "Action.h"

Action::
Action(){
}

Action::
~Action() = default;

bool
Action::
CheckPreConditions(const FactLayer* _factLayer){
  return true;
}

void
Action::
SetLayerMembership(size_t _layerMembership){
  m_layerMembership = _layerMembership;
}

size_t
Action::
GetLayerMembership(){
  return m_layerMembership;
}

bool
Action::
Used(){
  return m_used;
}

void
Action::
SetUsed(bool _used){
  m_used = _used;
}

State&
Action::
GetStartState(){
  return m_startState;
}

State&
Action::
GetResultState(){
  return m_resultState;
}

std::vector<Robot*>
Action::
GetRobots(){
  return {};
}

std::string
Action::
PrintAction(){
  return "Base Action";
}

double
Action::
GetCost(){
  return m_cost;
}

/*----------------------Equality---------------------------*/

bool
Action::
operator==(const Action& _action) const {
  if(m_startState != _action.m_startState or
      m_resultState != _action.m_resultState)
    return false;
  return true;
}

bool
Action::
operator!=(const Action& _action) const {
  return !(*this == _action);
}


/*----------------------Helpers---------------------------*/

bool
Action::
CanBeInLocation(Robot* _robot, const Boundary* _location){

  std::vector<Cfg> goalPoints;

  MPSolution* originalSol = m_library->GetMPSolution();
  MPSolution* sol = new MPSolution(_robot);
  MPTask* originalTask = m_library->GetTask();
  MPTask* task = new MPTask(_robot);
  m_library->SetTask(task);
  m_library->SetMPSolution(sol);
  auto sampler = m_library->GetSampler("UniformRandomFreeTerrain");
  size_t numNodes = 1, numAttempts = 100;
  sampler->Sample(numNodes, numAttempts, _location, std::back_inserter(goalPoints));

  m_library->SetMPSolution(originalSol);
  m_library->SetTask(originalTask);
  delete sol;
  delete task;
  if(goalPoints.empty())
    return false;
  return true;
}
