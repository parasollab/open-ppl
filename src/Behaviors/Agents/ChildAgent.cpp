#include "ChildAgent.h"

#include "Coordinator.h"

#include "Simulator/Simulation.h"
/*---------------------------------- Construction ----------------------------------*/

ChildAgent::
ChildAgent(Robot* const _r) : PathFollowingAgent(_r) {

}

ChildAgent::
ChildAgent(Robot* const _r, XMLNode& _node) : PathFollowingAgent(_r,_node) {

}

 ChildAgent::
~ChildAgent() {
  Uninitialize();
}

/*----------------------------- Simulation Interface -------------------------------*/

void 
ChildAgent::
Initialize() {
  PathFollowingAgent::Initialize();
}

void 
ChildAgent::
Step(const double _dt) {
  if(m_debug and m_graphVisualID == (size_t(-1)) and m_solution.get()){
    m_graphVisualID = Simulation::Get()->AddRoadmap(m_solution->GetRoadmap(),
      glutils::color(0., 1., 0., 0.2));
  }
  if(m_debug and m_pathVisualID == (size_t(-1)) and !m_path.empty()) {
		m_pathVisualID = Simulation::Get()->AddPath(m_path, glutils::color::red);
	}

	PathFollowingAgent::Step(_dt);
}

/*------------------------------- Child Interface ---------------------------------*/

Coordinator* 
ChildAgent::
GetCoordinator() {
	return m_coordinator;
}

void 
ChildAgent::
SetCoordinator(Coordinator* const _coordinator) {
	m_coordinator = _coordinator;
}

/*---------------------------------- Accessors ------------------------------------*/
    
MPSolution* 
ChildAgent::
GetMPSolution() {
	return m_solution.get();
}

/*-------------------------------- Task Helpers -----------------------------------*/

bool 
ChildAgent::
SelectTask() {
	return true;
}

bool 
ChildAgent::
EvaluateTask() {
	return PathFollowingAgent::EvaluateTask();
}

void 
ChildAgent::
ExecuteTask(const double _dt) {
	PathFollowingAgent::ExecuteTask(_dt);
}
		
void 
ChildAgent::
GeneratePlan() {}

/*------------------------------- Controller Helpers ---------------------------------*/

void 
ChildAgent::
ExecuteControls(const ControlSet& _c, const size_t _steps) {
  this->Agent::ExecuteControls(_c, _steps);
  if(_c.size() > 1)
    throw RunTimeException(WHERE,
        "We are assuming that only one control will be passed in at a time.");
}

