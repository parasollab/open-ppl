#include "DummyAgent.h"

#include "Simulator/Simulation.h"

DummyAgent::
DummyAgent(Robot* const _r) : PathFollowingAgent(_r) { 
	m_pathID = MAX_INT;
}

DummyAgent::
DummyAgent(Robot* const _r, const DummyAgent& _a) :
					 PathFollowingAgent(_r, _a) {
	//m_path = {};
	m_pathID = MAX_INT;
}

DummyAgent::
DummyAgent(Robot* const _r, XMLNode& _node) : 
					 PathFollowingAgent(_r, _node) { 
	//m_path = {};
	m_pathID = MAX_INT;
}

DummyAgent::
~DummyAgent() {
  PathFollowingAgent::Uninitialize();
}

void
DummyAgent::
Step(const double _dt) {
  Initialize();
	
  // If the agent is planning or localizing, skip this step.
  /*if(IsPlanning() or IsLocalizing())
    return;

  // If the agent localized before this step, update the simulated state and
  // replan if necessary.
  if(!IsLocalizing() and m_localizeCount == 0)
    UpdateSimulatedState();

  // If the simulation has passed a set number of timesteps, localize.
  ++m_localizeCount;
  if(m_localizeCount > m_localizePeriod) {
    if(m_debug)
      std::cout << "Enqueueing localize command." << std::endl;
    Localize();
    m_localizeCount = 0;
  }
	*/
  // Wait for the previous controls to finish if they still have time remaining.
  if(ContinueLastControls())
    return;

	//If the agent has a path, it will follow it. Otherwise it's just chillin
  if(!this->m_path.empty() and EvaluateTask()) {
		if(m_pathID == MAX_INT) {
			m_pathID = Simulation::Get()->AddPath(m_path,glutils::color(1, 0, 0, 0.5));
		}
    ExecuteTask(_dt);
	}
  else
    PauseAgent(1);
}

bool
DummyAgent::
EvaluateTask() {
  // Get the current configuration.
  const Cfg current = m_robot->GetSimulationModel()->GetState();

  // We consider the robot to have reached the next subgoal if it is within a
  // threshold distance. Advance the path index until the next subgoal is
  // at least one threshold away.
  auto dm = m_library->GetDistanceMetric(m_waypointDm);

  double distance = dm->Distance(current, m_path[m_pathIndex]);

  if(m_debug)
    std::cout << "\tDistance from current configuration: "
              << distance << "/" << m_waypointThreshold
              << std::endl;

  // Advance our subgoal while we are within the distance threshold of the next
  // one.
  while(distance < m_waypointThreshold) {
    if(m_debug)
      std::cout << "\tReached waypoint " << m_pathIndex << " at "
                << distance << "/" << m_waypointThreshold
                << "\n\t\t" << m_path[m_pathIndex].PrettyPrint()
                << std::endl;

    // Move to next cfg in path since the distance is within the threshold.
    ++m_pathIndex;

    // Check if we have completed the path. If so, this task is complete.
    if(m_pathIndex == m_path.size())
    {
      if(m_debug)
        std::cout << "Reached the end of the path." << std::endl;
			Simulation::Get()->RemovePath(m_pathID);
			m_pathID = MAX_INT;
			m_path = {};
      return false;
    }

    distance = dm->Distance(current, m_path[m_pathIndex]);
  }

  return true;
}

