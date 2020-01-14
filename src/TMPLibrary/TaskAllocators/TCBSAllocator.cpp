#include "TCBSAllocator.h"

#include "Simulator/Simulation.h"

#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"

#include "Utilities/GeneralCBS/AllocationValidation.h"
#include "Utilities/GeneralCBS/ComposeValidation.h"
#include "Utilities/GeneralCBS/GeneralCBS.h"
#include "Utilities/GeneralCBS/MotionValidation.h"
#include "Utilities/GeneralCBS/MPLowLevelSearch.h"
#include "Utilities/GeneralCBS/TCBSValidation.h"

TCBSAllocator::
TCBSAllocator() {
	this->SetName("TCBSAllocator");
}

TCBSAllocator::
TCBSAllocator(XMLNode& _node) : TaskAllocatorMethod(_node) {
	this->SetName("TCBSAllocator");
	m_sgLabel = _node.Read("sgLabel", true, "", "State graph label for TCBSAllocator.");
	m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label for TCBSAllocator.");
	m_lowLevelDebug = _node.Read("llDebug", false, false, "Debug flag for low level search.");
	m_decoupled = _node.Read("decoupled", false, m_decoupled, 
														"Indicates decoupled allocation and motion planning.");
}

TCBSAllocator::
~TCBSAllocator() {} 	

void 
TCBSAllocator::
AllocateTasks() {

	auto decomp = CreateDecomposition(this->GetTaskPlan()->GetWholeTasks());
	
	MPLowLevelSearch lowLevel(this->GetTMPLibrary(), m_sgLabel, m_vcLabel,m_lowLevelDebug);

	auto tcbs = new TCBSValidation(this->GetMPLibrary(),&lowLevel, this->GetTMPLibrary());
	auto motion = new MotionValidation(this->GetMPLibrary(),&lowLevel,this->GetTMPLibrary(),m_sgLabel,m_vcLabel);
	auto compose = new ComposeValidation({motion,tcbs});

	InitialPlanFunction init = [this,tcbs](Decomposition* _decomp,GeneralCBSTree& _tree) {
		return tcbs->InitialPlan(_decomp,_tree);
	};

	CBSSolution solution;

	if(!m_decoupled) {
		ValidationFunction composeValid = [this,compose](GeneralCBSNode& _node,GeneralCBSTree& _tree) {
			return compose->ValidatePlan(_node, _tree);
		};

		solution = ConflictBasedSearch(decomp, init, composeValid,MAX_INT,m_debug);	
	}
	else {
		InitialPlanFunction motionInit = [this,motion](Decomposition* _decomp,GeneralCBSTree& _tree) {
			return motion->InitialPlan(_decomp,_tree);
		};
		ValidationFunction tcbsValid = [this,tcbs](GeneralCBSNode& _node,GeneralCBSTree& _tree) {
			return tcbs->ValidatePlan(_node, _tree);
		};

		ValidationFunction motionValid = [this,motion](GeneralCBSNode& _node,GeneralCBSTree& _tree) {
			return motion->ValidatePlan(_node, _tree);
		};

		solution = ConflictBasedSearch(decomp, init, tcbsValid,MAX_INT,m_debug);
		
		auto top = decomp->GetMainTask();
		for(auto task : top->GetSubtasks()) {
			auto taskPlan = solution.m_taskPlans[task];
			auto subtasks = task->GetSubtasks();
			for(size_t i = 0; i < taskPlan.size(); i++) {
				if(subtasks[i] != taskPlan[i].m_task)
					throw RunTimeException(WHERE,"Mismatched subtask orderings.");
				subtasks[i]->GetMotionTask()->SetRobot(taskPlan[i].m_agent->GetRobot());
			}
		}

		for(auto agentAssignments : solution.m_agentAssignments) {
			auto assigns = agentAssignments.second;
			for(size_t i = 1; i < assigns.size(); i++) {
				assigns[i].m_task->AddDependency(assigns[i-1].m_task,SemanticTask::DependencyType::Completion);
			}
		}

		solution = ConflictBasedSearch(decomp,motionInit,motionValid, MAX_INT,m_debug);

	}

	if(m_debug) {
		auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
		for(auto plan : solution.m_taskPlans) {
			auto task = plan.first;
			auto assignments = plan.second;
			std::cout << std::endl << std::endl << "Task: " << task << std::endl;
			for(auto a : assignments) {
				auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(a.m_agent)).get();
				std::cout << "\tAgent: " << a.m_agent->GetRobot()->GetLabel() << std::endl;
				std::cout << "\t\tSetup Path" << std::endl;
				for(auto vid : a.m_setupPath) {
					std::cout << "\t\t\t" << roadmap->GetVertex(vid).PrettyPrint() << std::endl;
				}
				std::cout << "\t\tExec Path    (" << a.m_execStartTime << "->"  << a.m_execEndTime << ")" << std::endl;
				for(auto vid : a.m_execPath) {
					std::cout << "\t\t\t" << roadmap->GetVertex(vid).PrettyPrint() << std::endl;
				}
			}
		}
	}
  Simulation::Get()->PrintStatFile();
}

Decomposition* 
TCBSAllocator::
CreateDecomposition(std::vector<WholeTask*> _wholeTasks) {
	auto top = std::shared_ptr<SemanticTask>(new SemanticTask());
	Decomposition* decomp = new Decomposition(top);

	for(auto wholeTask : _wholeTasks) {
		auto semanticTask = std::shared_ptr<SemanticTask>(new SemanticTask(top.get(),wholeTask->m_task));
		top->AddSubtask(semanticTask.get());
		decomp->AddTask(semanticTask);
		this->GetTaskPlan()->SetSemanticWholeTask(semanticTask.get(),wholeTask);

		std::shared_ptr<SemanticTask> previous = nullptr;
		for(auto subtask : wholeTask->m_subtasks) {
			auto sub = std::shared_ptr<SemanticTask>(new SemanticTask(semanticTask.get(),subtask));
			decomp->AddSimpleTask(semanticTask.get());
			semanticTask->AddSubtask(sub.get());
			if(previous) {
				sub->AddDependency(previous.get(),SemanticTask::DependencyType::Initiation);
			}
			previous = sub;
		}
	}
	return decomp;
}
