#include "PlanningServer.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"
#include "Communication/Messages/Message.h"
#include "Simulator/Simulation.h"
#include "TMPLibrary/Solution/Plan.h"
#include "Utilities/PMPLExceptions.h"

/*--------------------------- Construction ---------------------------*/

PlanningServer::
PlanningServer(TMPLibrary* _library) : m_library(_library) {}

/*--------------------------- Interface ------------------------------*/

std::vector<std::string>
PlanningServer::
Solve(std::string _msg) const {
	PlanningProblem problem = ParseMessage(_msg);

	Plan* plan = new Plan();
	plan->SetCoordinator(problem.coordinator);
	plan->SetTeam(problem.team);
	plan->SetDecomposition(problem.decomposition);

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
									 plan,
									 problem.coordinator, 
									 problem.team);

	std::vector<std::string> messages;
	auto planMessage = PlanToMessage(plan);
	messages.push_back(planMessage);

	auto& ref1 = messages.back();
	ref1 += "<<roadmaps=";

	for(auto robot : problem.team) {
		auto allocs = plan->GetAllocations(robot);
		if(allocs.empty())
			continue;
		auto sol = plan->GetTaskSolution(*(allocs.begin()));
		auto roadmap = sol->GetMotionSolution()->GetRoadmap(robot);
		std::vector<std::string> rm = RoadmapToMessage(roadmap,robot);
		for(auto r : rm) {
			messages.push_back(r);
		}
	}

	auto& ref2 = messages.back();
	ref2 += ">><<motion_plans=";

	
	for(const auto& kv : plan->GetTaskSolutions()) {
		if(!kv.second)
			continue;
		if(!kv.second->GetMotionSolution())
			continue;
		std::string label = kv.first->GetLabel();
		std::vector<std::string> motionPlan = MotionSolutionToMessage(kv.second->GetMotionSolution(),label);
		for(const auto& mp : motionPlan) {
			messages.push_back(mp);
		}
	}

	auto& ref3 = messages.back();
	ref3 += ">>";

	auto& first = messages.front();
	first = "<" + first;
	auto& last = messages.back();
	last += ">";	

	return messages;
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
