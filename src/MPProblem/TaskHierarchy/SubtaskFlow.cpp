#include "SubtaskFlow.h"

/*----------------------------------------- Construction ------------------------------------*/

SubtaskFlow::
SubtaskFlow(SemanticTask* _task) {
	FlowNode node;
	node.m_task = _task;
	auto vid = this->add_vertex(node);
	TBDFunction blank;
	std::vector<size_t> vids = {vid};
	ParentInfo parent = std::make_pair(vids,blank);
	EvalNode(_task, parent);
}

SubtaskFlow::
~SubtaskFlow() { }

/*-------------------------------------------- Debug ----------------------------------------*/
		
void 
SubtaskFlow::
Print() {
	for(auto vit = this->begin(); vit != this->end(); vit++) {
		auto task = vit->property().m_task;
		std::cout << "Task: " << task->GetLabel() << std::endl;
		for(auto eit = vit->begin(); eit != vit->end(); eit++) {
			std::string depType;
			switch(eit->property()) {
				case SemanticTask::DependencyType::Completion : 
					depType = "Completion";
					break;
				case SemanticTask::DependencyType::Initiation : 
					depType = "Initiation";
					break;
				case SemanticTask::DependencyType::Synchronous : 
					depType = "Synchronus";
					break;
				case SemanticTask::DependencyType::Asynchronous : 
					depType = "Asynchronous";
					break;
				case SemanticTask::DependencyType::None : 
					depType = "None";
					break;
				default :
					depType = "Unknown type. Need to update print statement.";
					break;
			}
			auto child = this->find_vertex(eit->target());
			if(child == this->end())
				throw RunTimeException(WHERE, "Child node is not in the Flow");
			std::cout << "\t\tChild: " 
								<< child->property().m_task->GetLabel() 
								<< " DepType: " 
								<< depType 
								<< std::endl;
		}
	}
}
/*--------------------------------------- Helper Function------------------------------------*/

std::pair<std::vector<size_t>,TBDFunction>
SubtaskFlow::
EvalNode(SemanticTask* _task, ParentInfo _parentInfo) {

	// If this is a simple task and a thus a leaf in the decomposition tree, create a node for it
	if(_task->GetSubtasks().empty()) {
		FlowNode node;
		node.m_task = _task;
		node.m_initiationFunction = _parentInfo.second;
		auto vid = this->add_vertex(node);

		m_taskNodeMap[_task] = vid;

		//for(auto p : _parentInfo.first) {
		//	this->add_edge(p,vid,);
		//}
		TBDFunction simple;
		simple.m_elems.push_back(vid);
		std::vector<size_t> vids = {vid};
		return std::make_pair(vids,simple);
	}

	auto subtasks = _task->GetSubtasks();
	if(subtasks.size() != 2) {
		std::cout << "Number of subtasks: " << subtasks.size() << std::endl;
		throw RunTimeException(WHERE, 
				"SubtaskFlow is operating under the assumption that the decomp tree is binary.");
	}


	// Check if first subtask is dependent on second.
	ParentInfo children;
	for(auto subtask : subtasks) {
		children = HandleDependencies(subtask, _parentInfo);
		if(!children.first.empty() 
				or !children.second.m_subFunctions.empty() 
				or !children.second.m_elems.empty())
			return children;
	}

	auto one = EvalNode(subtasks[0], _parentInfo);
	auto two = EvalNode(subtasks[1], _parentInfo);
	
	TBDFunction func;
	func.m_subFunctions = {one.second,two.second};
	
	auto& childVIDs = one.first;
	for(auto vid : two.first) {
		childVIDs.push_back(vid);
	}

	for(auto source : _parentInfo.first) {
		for(auto target : childVIDs) {
			this->add_edge(source, target, SemanticTask::DependencyType::None);
		}
	}

	// Check if the decomposition splits into two options

	if(_task->GetSubtaskRelation() == SemanticTask::SubtaskRelation::XOR) {

		func.m_operator = TBDFunction::MIN;
		return std::make_pair(childVIDs,func);
	}
	// Last remaining scenario is two required independent subtasks
	func.m_operator = TBDFunction::MAX;
	return std::make_pair(childVIDs,func);	
}

std::pair<std::vector<size_t>,TBDFunction>
SubtaskFlow::
HandleDependencies(SemanticTask* _task, ParentInfo _parentInfo) {
	auto& depMap = _task->GetDependencies();

	ParentInfo children;

	if(!depMap.empty()) {
		for(auto dep : depMap) {
			if(dep.second.size() > 1) {
				throw RunTimeException(WHERE, "Decomp tree is not binary.");
			}
			auto depTask = dep.second[0];
			switch(dep.first) {
				case SemanticTask::Completion :
						throw RunTimeException(WHERE, "Dependency type not handled."); 
						break;
				case SemanticTask::Initiation : // Standard sequential dependency 
					{
						children = EvalNode(depTask, _parentInfo);
						ParentInfo grandChildren = EvalNode(_task, children);
						for(auto source : children.first) {
							for(auto target : grandChildren.first) {
								this->add_edge(source, target, dep.first);
							}
						}
						break;
					}
				case SemanticTask::Synchronous :
					children = MergeNodes(_task,depTask, _parentInfo);
					break;
				case SemanticTask::Asynchronous :
					throw RunTimeException(WHERE, "Dependency type not handled."); 
					break;
				case SemanticTask::None :
					throw RunTimeException(WHERE, "Dependency type not handled."); 
					break;
				default:
					throw RunTimeException(WHERE, "Dependency type not handled."); 
					break;
			}
		}		
	}
	return children;
}

std::pair<std::vector<size_t>,TBDFunction>
SubtaskFlow::
MergeNodes(SemanticTask* _one, SemanticTask* _two, ParentInfo _parentInfo) {
	auto left = EvalNode(_one,_parentInfo);
	auto right = EvalNode(_two,_parentInfo);

	FlowNode merge;

	//Collect all of the parent vids into the merged node.
	merge.m_subNodes = left.first;
	for(auto vid : right.first) {
		merge.m_subNodes.push_back(vid);
	}

	TBDFunction func;
	func.m_operator = TBDFunction::MAX;
	func.m_subFunctions = {left.second,right.second};

	merge.m_initiationFunction = func;

	auto vid = this->add_vertex(merge);

	for(auto v : merge.m_subNodes) {
		auto iter = this->find_vertex(v);
		if(iter == this->end())
			throw RunTimeException(WHERE, "Looking for non-existant node.");
		for(auto eit = iter->begin(); eit != iter->end(); eit++) {
			this->add_edge(vid,eit->target(),SemanticTask::DependencyType::None);
		}
	}

	TBDFunction retFunc;
	retFunc.m_elems = {vid};

	std::vector<size_t> vids = {vid};
	return std::make_pair(vids,retFunc);
}
/*-------------------------------------------------------------------------------------------*/
