#include "CentralPlanner.h"

#include "Simulator/Simulation.h"

CentralPlanner::
CentralPlanner(Robot* const _r) : Agent(_r) {
}


CentralPlanner::
CentralPlanner(Robot* const _r, XMLNode& _node) : Agent(_r) {

  // Parse the labels of the group members.
  for(auto& child : _node) {
		const std::string memberLabel = child.Read("label", true, "",
			"The label of the member robot.");
		m_memberLabels.push_back(memberLabel);
  }

  if(m_memberLabels.empty())
    throw ParseException(_node.Where(), "Coordinator requires at "
        "least one member robot.");
}


CentralPlanner::
~CentralPlanner() {
  Uninitialize();
}


std::unique_ptr<Agent>
CentralPlanner::
Clone(Robot* const _r) const {
  throw RunTimeException(WHERE, "Not yet implemented.");
  return {nullptr};
}


void
CentralPlanner::
Initialize() {
  if(m_initialized)
    return;
  m_initialized = true;
  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();
  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);
	auto group = problem->GetRobotGroup(0);
	m_solution = new MPSolution(group);
  m_library->SetMPSolution(m_solution);
  // Set up the group members.
  for(const auto& memberLabel : m_memberLabels) {
    Robot* member = problem->GetRobot(memberLabel);
    // We are assuming that all member robots have a compatible agent type.
    // Throw an exception if not.
    Agent* memberAgent = member->GetAgent();
    DummyAgent* a = dynamic_cast<DummyAgent*>(memberAgent);
    if(!a)
      throw RunTimeException(WHERE, "Incompatible agent type specified for "
          "group member '" + memberLabel + "'.");
    m_memberAgents.push_back(a);	
		//Initialize Agent
		a->Initialize();
  }

	auto groupTask = problem->GetTasks(group)[0];

	//Using default solver
	m_library->Solve(problem, groupTask.get(), m_solution);            
	LoadMemberPlans();		
}


void
CentralPlanner::
LoadMemberPlans() {
	//auto group = m_robot->GetMPProblem()->GetRobotGroup(0);
	for(auto agent : m_memberAgents) {
		//Get whatever path you want from the solution object
		//Example: assuimg just the one member agent that was planner with
		//std::vector<Cfg> path;

		std::cout << "Loading path for " << agent->GetRobot()->GetLabel() << std::endl;

		//for(auto groupCfg : m_library->GetMPSolution(group)->GetGroupPath(group)->FullCfgs(m_library)) {
		//	path.push_back(groupCfg.GetRobotCfg(agent->GetRobot()));
		//}
		auto path = m_solution->GetPath(agent->GetRobot());
		//agent->SetPlan(path->FullCfgs(m_library));
		agent->SetPlan(path->Cfgs());
		//The agents are set to visualize their path while in debug. You can add other paths or roadmaps 
		//with the Simulator interfaces (ie. AddPath, AddRoadmap) described in the Simulator class.
	}
}


void
CentralPlanner::
Step(const double _dt) {
  Initialize();

  if(this->m_debug) {
    std::cout << "___________________________________________________________"
              << std::endl;
    for(auto agent : m_memberAgents)  {
        std::cout << agent->GetRobot()->GetLabel() << " has plan: "
                  << agent->HasPlan() << std::endl;
    }
  }
  for(auto agent : m_memberAgents)
    agent->Step(_dt); 
}


void
CentralPlanner::
Uninitialize() {
  if(!m_initialized)
    return;
  m_initialized = false;

  delete m_solution;
  delete m_library;

  m_solution = nullptr;
  m_library  = nullptr;
}
