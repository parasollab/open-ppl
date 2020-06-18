#include "Validation.h"

#include "GeneralCBS.h"

#include "TMPLibrary/TaskPlan.h"

Validation::
Validation() {}	

Validation::
Validation(MPLibrary* _library, LowLevelSearch* _lowLevel, TMPLibrary* _tmpLibrary) : 
					m_library(_library), m_lowLevel(_lowLevel), m_tmpLibrary(_tmpLibrary) {}	

Validation::
~Validation() {}

bool
Validation::
InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) {
	return true;
}

bool 
Validation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {
	return true;
}

CBSSolution
Validation::
CreatePostAssignment() {
	CBSSolution post;
	post.m_decomposition = nullptr;

	auto team = m_tmpLibrary->GetTaskPlan()->GetTeam();
	auto problem = m_tmpLibrary->GetMPProblem();

	for(auto agent : team) {
		auto mp = problem->GetTasks(agent->GetRobot())[0];
		auto task = new SemanticTask();
		auto child = new SemanticTask(task,nullptr,mp,false);

		task->AddSubtask(child);

		Assignment a(agent, child, {}, {}, 0, 0, 0);
		post.m_agentAssignments[agent].push_back(a);
	}
	return post;
}
