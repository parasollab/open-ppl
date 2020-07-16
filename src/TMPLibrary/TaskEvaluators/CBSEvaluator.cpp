#include "CBSEvaluator.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "Utilities/GeneralCBS/GeneralCBS.h"
#include "Utilities/GeneralCBS/ParallelCBS.h"
#include "Utilities/GeneralCBS/MotionValidation.h"
#include "Utilities/GeneralCBS/MPLowLevelSearch.h"
/*------------------------------------- Construction ---------------------------------*/

CBSEvaluator::
CBSEvaluator() {
	this->SetName("CBSEvaluator");
}

CBSEvaluator::		
CBSEvaluator(XMLNode& _node) : TaskEvaluatorMethod(_node) {
	this->SetName("CBSEvaluator");
	
	m_vcLabel = _node.Read("vcLabel", true, "", "Validity Checker label to pass to "
												 "underlying CBS components.");
	m_lowLevelDebug = _node.Read("llDebug", false, m_lowLevelDebug, 
																"Debug flag for low level search.");
	m_parallel = _node.Read("parallel", false, m_parallel,
													"Flag for executing the parallel implementation.");
	m_avoidancePaths = _node.Read("avoidancePaths", false, m_avoidancePaths,
																"Flag for post task dummy path to get out of the way.");
}

CBSEvaluator::
~CBSEvaluator() {}

/*---------------------------------- Evaluator Overrides -----------------------------*/

bool 
CBSEvaluator::
Run(std::vector<WholeTask*> _wholeTasks, std::shared_ptr<TaskPlan> _plan) {

	//TODO::Temporary to structure decomp as needed by MPLowLevel and MotionValidation
	Decomposition* decomp = new Decomposition();
	auto top = std::shared_ptr<SemanticTask>(new SemanticTask("top",nullptr,decomp));
	decomp->SetMainTask(top.get());
	decomp->SetCoordinator(this->GetPlan()->GetCoordinator()->GetRobot());

	std::vector<std::shared_ptr<SemanticTask>> subtasks;

	for(auto task : this->GetPlan()->GetDecomposition()->GetMotionTasks()) {
		auto child = std::shared_ptr<SemanticTask>(new SemanticTask(task->GetLabel()+"_1",
														task,decomp,task->GetMotionTask(),false));
		subtasks.push_back(child);
		task->SetParent(top.get());
		top->AddSubtask(task);
	}

	MPLowLevelSearch lowLevel(this->GetTMPLibrary(), m_sgLabel, m_vcLabel,m_lowLevelDebug);

	auto motion = new MotionValidation(this->GetMPLibrary(),&lowLevel,m_avoidancePaths,
																		 this->GetTMPLibrary(),m_sgLabel,m_vcLabel);

	InitialPlanFunction init = [this,motion](Decomposition* _decomp,GeneralCBSTree& _tree) {
		return motion->InitialPlan(_decomp,_tree);
	};
	ValidationFunction valid = [this,motion](GeneralCBSNode& _node,GeneralCBSTree& _tree) {
		return motion->ValidatePlan(_node, _tree);
	};

	CBSSolution solution;

	if(!m_parallel)
		solution = ConflictBasedSearch(decomp,
				init,valid,this->GetPlan()->GetStatClass(),MAX_INT,m_debug);
	else
		solution = ParallelConflictBasedSearch(decomp,
				init,valid,this->GetPlan()->GetStatClass(),MAX_INT,m_debug);

	ConvertCBSSolutionToPlan(solution);

	this->GetPlan()->Print();

	return true;
}

/*----------------------------------- Helper Functions -------------------------------*/

void
CBSEvaluator::
ConvertCBSSolutionToPlan(const CBSSolution& _solution) {
	auto plan = this->GetPlan();

	for(auto tp : _solution.m_taskPlans) {
		if(!_solution.m_solutionTasks.count(tp.first))
			continue;
		for(auto assign : tp.second) {
			plan->AddAllocation(assign.m_agent->GetRobot(),assign.m_task);
			plan->SetTaskSolution(assign.m_task,ConvertAssignmentToTaskSolution(assign));
		}
	}
}

std::shared_ptr<TaskSolution>
CBSEvaluator::
ConvertAssignmentToTaskSolution(Assignment& _assign) {
	auto sol = std::shared_ptr<TaskSolution>(new TaskSolution(_assign.m_task));

	sol->SetRobot(_assign.m_agent->GetRobot());

	// Configure MPSolution
	MPSolution* mp = new MPSolution(_assign.m_agent->GetRobot());
	auto sg = static_cast<CombinedRoadmap*>(this->GetStateGraph(m_sgLabel).get());
	auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(_assign.m_agent));
	mp->SetRoadmap(_assign.m_agent->GetRobot(),roadmap.get());

	auto vids = _assign.m_setupPath;
	for(auto& vid : _assign.m_execPath) {
		vids.push_back(vid);
	}

	auto path = new PathType<MPTraits<Cfg>>(roadmap.get());
	(*path) += vids;
	mp->SetPath(_assign.m_agent->GetRobot(),path);

	sol->SetMotionSolution(mp);

	sol->SetStartTime(_assign.m_setupStartTime);

	return sol;
}
