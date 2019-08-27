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
	m_solution = new MPSolution(m_robot);
  m_library->SetMPSolution(m_solution);

  // Set up the group members.
  for(const auto& memberLabel : m_memberLabels) {
    Robot* member = problem->GetRobot(memberLabel);

    // We are assuming that all member robots have a compatible agent type.
    // Throw an exception if not.
    Agent* memberAgent = member->GetAgent();

    DummyAgent* a = dynamic_cast<DummyAgent*>(
        memberAgent);
    if(!a)
      throw RunTimeException(WHERE, "Incompatible agent type specified for "
          "group member '" + memberLabel + "'.");
    m_memberAgents.push_back(a);
		
		//Initialize Agent
		a->Initialize();
  }

	auto group = problem->GetRobotGroup(0);
	auto groupTask = problem->GetTasks(group)[0];

	//Do whatever solving you want here
	//This gets the first task assigned to the central planner in the XML file
	//auto task = problem->GetTasks(m_robot)[0];
	//The library will use whatever robot the task is assigned. If you want to use a specific member 
	//agent's robot then set the task robot to that specific robot.
	//task->SetRobot(m_memberAgents[0]->GetRobot());
	//The solution object needs to have an initialized roadmap for this robot as well
	//m_solution->SetRobot(m_memberAgents[0]->GetRobot());
	//Irving: I think you should be able to create/get your group task here and have the library plan for that.
	//You may need to update the solution object to be a group solution though. I am not as familiar with how 
	//you keep track of all that.
	
	
	//This will use the default solver 
	m_library->Solve(problem, groupTask.get(), m_solution);
  //You can also manually specify what you want to use
  //Make sure these labels are in your xml or replace them with member variables set in the xml file
  //Example:
	//m_library->Solve(problem, task.get(),
            //m_solution, "FixedPRM", LRand(), "FixedPRM");
            

	LoadMemberPlans();		

}

void
CentralPlanner::
LoadMemberPlans() {
	auto group = m_robot->GetMPProblem()->GetRobotGroup(0);
	for(auto agent : m_memberAgents) {
		//Get whatever path you want from the solution object
		//Example: assuimg just the one member agent that was planner with
		std::vector<Cfg> path;

		for(auto groupCfg : m_library->GetMPSolution()->GetGroupPath(group)->FullCfgs(m_library)) {
			path.push_back(groupCfg.GetRobotCfg(agent->GetRobot()));
		}
		
		agent->SetPlan(path);
		//The agents are set to visualize their path while in debug. You can add other paths or roadmaps 
		//with the Simulator interfaces (ie. AddPath, AddRoadmap) described in the Simulator class.

	}
}

void
CentralPlanner::
Step(const double _dt) {
  Initialize();

  if(this->m_debug)
    std::cout << "___________________________________________________________"
              << std::endl;
  for(auto agent : m_memberAgents)  {
    if(this->m_debug)
      std::cout << agent->GetRobot()->GetLabel()
                << " has plan: "
                << agent->HasPlan()
                << std::endl;
  }

  for(auto agent : m_memberAgents){
    agent->Step(_dt);
  }
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
