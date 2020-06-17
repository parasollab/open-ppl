#include "PlanningServer.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"
#include "Communication/Messages/Message.h"
#include "Simulator/Simulation.h"
#include "Utilities/PMPLExceptions.h"

/*--------------------------- Construction ---------------------------*/

PlanningServer::
PlanningServer(TMPLibrary* _library) : m_library(_library) {}

/*--------------------------- Interface ------------------------------*/

std::string
PlanningServer::
Solve(std::string _msg) const {
	PlanningProblem problem = ParseMessage(_msg);

	auto taskPlan = std::shared_ptr<TaskPlan>(new TaskPlan());
	
	auto decomp = std::unique_ptr<Decomposition>(problem.decomposition);
	m_library->GetMPProblem()->AddDecomposition(problem.coordinator->GetRobot(),std::move(decomp));

	problem.coordinator->InitializePlanningComponents();
	for(auto r : problem.team) {
		auto agent = dynamic_cast<PlanningAgent*>(r->GetAgent());
		agent->Initialize();
	}

  m_library->Solve(m_library->GetMPProblem(), 
									 problem.decomposition, 
									 taskPlan,
									 problem.coordinator, 
									 problem.team);

	return "A motion plan";
}
		
/*-------------------------- Helper Functions -----------------------*/
PlanningServer::PlanningProblem 
PlanningServer::
ParseMessage(std::string _msg) const {

	PlanningProblem problem;

	std::stringstream ss(_msg);
	std::string param;
	getline(ss,param,'/');
	if(param != "query")
		throw RunTimeException(WHERE) << "Invalid message passed to Planning Server." << std::endl;

	// dumping channel name
	getline(ss,param,'/');

	// parsing robot team
	getline(ss,param,'/');

	std::stringstream team(param);
	getline(team,param,'=');
	if(param != "robot_team") 
		throw RunTimeException(WHERE) << "Message "
																	<< _msg
																	<< " should contain 'robot_team=<team>."
																	<< std::endl;

	getline(team,param,'/');

	auto teamInfo = MessageToRobotTeam(param, m_library->GetMPProblem());
	problem.coordinator = static_cast<Coordinator*>(m_library->GetMPProblem()->GetRobot(teamInfo.second)->GetAgent());
	for(auto robot : teamInfo.first) {
		problem.team.push_back(robot);
	}

	getline(ss,param,'/');

	std::stringstream decomp(param);
	getline(decomp,param,'=');
	if(param != "decomp")
		throw RunTimeException(WHERE) << "Message " 
																	<< _msg
																	<< "should contain 'decomp=<decomp info>."
																	<< std::endl;
	getline(decomp,param,'/');

	problem.decomposition = MessageToDecomposition(param,m_library->GetMPProblem());
	return problem;
}
