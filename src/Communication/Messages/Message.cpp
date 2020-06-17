#include "Message.h"

#include <list>
#include <sstream>
#include <typeinfo>

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Boundaries/Boundary.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"

//Helper
std::vector<std::string>
ParseSubMessage(std::string _msg) {
	if(_msg[0] != '<')
		throw RunTimeException(WHERE) << _msg 
																	<< " is not a sub message." 
																	<< std::endl;
	if(_msg[_msg.size()-1] != '>')
		throw RunTimeException(WHERE) << "Sub messages " 
																	<< _msg 
																	<< " does not have a closing character." 
																	<< std::endl;

	std::vector<std::string> parsed;
	parsed.push_back("");

	size_t open = 0;
	size_t close = 0;
	size_t current = 0;

	for(size_t i = 1; i < _msg.size()-1; i++) {
		char c = _msg[i];
		if(c == '<') {
			open++;
			if(open == close+1) {
				parsed.push_back("");
				current++;
				continue;
			}
		}
		else if(c == '>') {
			close++;
			if(open == close)
				continue;
		}
	
		if(open == close and open > 0)
			throw RunTimeException(WHERE) << "Characters between sub messages in "
																		<< _msg
																		<< std::endl;

		parsed[current] = parsed[current] + c;	
	}

	if(open != close)
		throw RunTimeException(WHERE) << _msg 
																	<< " has uneven open/close characters." 
																	<< std::endl;

	std::cout << "Debug Info: parsed submessages" << std::endl;
	for(auto str : parsed) {
		std::cout << str << std::endl;
	}

	return parsed;
}

//Robot Team
std::string
RobotTeamToMessage(std::vector<Robot*> _robots, Robot* _coordinator) {
	std::string msg = "robot_team=<";
	if(_coordinator) {
		std::string name = _coordinator->GetLabel();
		msg = msg + "coordinator=" + name;
	}
	else {
		msg = msg + "coordinator=none";
	}

	msg += ",";
	
	for(size_t i = 0; i < _robots.size(); i++) {
		auto robot = _robots[i];
		std::string name = robot->GetLabel();
		msg += ("<label=" + name + ",");

		auto cfg = robot->GetMPProblem()->GetInitialCfg(robot);
		std::string initialCfg;
		for(size_t i = 0; i < cfg.DOF(); i++) {
			initialCfg += std::to_string(cfg[i]);
			//if(i < cfg.DOF()-1)
				initialCfg += ":";
		}
		msg += ("initial=" + initialCfg + ">");
	}

	msg = msg + ">/";
	return msg;		
}

std::pair<std::vector<Robot*>,std::string>
MessageToRobotTeam(std::string _msg, MPProblem* _problem) {
	std::string coordinator = "";
	std::vector<Robot*> robots;

	auto parsed = ParseSubMessage(_msg);

	std::stringstream ss(parsed[0]);
	std::string param;
	while(ss) {
		getline(ss,param,',');
		std::stringstream keyvalue(param);
		getline(keyvalue,param,'=');
		if(param == "coordinator") {
			getline(keyvalue,param);
			coordinator = param;
		}
	}
	
	for(size_t i = 1; i < parsed.size(); i++) {
		
		std::stringstream robotinfo(parsed[i]);

		//if(param == "member") {
			//getline(keyvalue,param);
			//auto submessages = ParseSubMessage(param);
			//std::stringstream robotinfo(submessages[0]);
			Robot* robot;
			while(robotinfo) {
				getline(robotinfo,param,',');

				std::stringstream info(param);
				getline(info,param,'=');

				if(param == "label") {
					getline(info,param);
					robot = _problem->GetRobot(param);
				}
				else if(param == "initial") {
					std::vector<double> dofs;
					while(info) {
						getline(info,param,':');
						if(param == "")
							continue;
						dofs.push_back(atof(param.c_str()));
					}
					Cfg cfg(robot);
					cfg.SetData(dofs);
					_problem->SetInitialCfg(robot,cfg);
				}
			}
			robots.push_back(robot);
		//}
	}

	std::cout << "Coordinator: " << coordinator << std::endl;
	for(auto robot : robots) {
		std::cout << "Member: " << robot << std::endl;
	}

	return std::make_pair(robots,coordinator);
}

//Decomposition
std::string
DecompositionToMessage(Decomposition* _decomp) {
	std::string info = "<task=" 
									 + _decomp->GetMainTask()->GetLabel()
									 + ",coordinator="
									 + _decomp->GetCoordinator()->GetLabel()
									 + ">";

	std::string msg;

	std::list<SemanticTask*> queue;
	std::unordered_set<SemanticTask*> seen;
	std::unordered_set<SemanticTask*> added;

	auto taskMap = _decomp->GetTaskMap();

	for(auto task : _decomp->GetSimpleTasks()) {
		queue.push_back(task);
	}

	auto iter = queue.begin();
	if(iter == queue.end() and taskMap.size() > 0)
		throw RunTimeException(WHERE) << "Error in decomp task structure." << std::endl;

	while(added.size() < taskMap.size()) {
		auto task = *iter;
		msg = "<<" + SemanticTaskToMessage(task) + ">>" + msg;
		added.insert(task);
		auto parent = task->GetParent();
		if(parent and !seen.count(parent))
			queue.push_back(parent);
		seen.insert(parent);
		iter++;
	}

	msg = "decomp=<" + info + msg + ">/";
	return msg;
}

Decomposition*
MessageToDecomposition(std::string _msg, MPProblem* _problem) {
	Decomposition* decomp = new Decomposition();

	auto submessages = ParseSubMessage(_msg);


	for(size_t i = 2; i < submessages.size(); i++) {
		std::shared_ptr<SemanticTask> task = std::shared_ptr<SemanticTask>(
																					MessageToSemanticTask(submessages[i],decomp,_problem));
		decomp->AddTask(task);
	}

	std::stringstream info(submessages[1]);
	while(info) {
		std::string param;
		getline(info,param,',');

		if(param == "")
			continue;

		std::stringstream keyvalue(param);
		getline(keyvalue,param,'=');
		
		if(param == "task") {
			getline(keyvalue,param,'=');
			decomp->SetMainTask(decomp->GetTask(param));
		}
		else if(param == "coordinator") {
			getline(keyvalue,param,'=');
			decomp->SetCoordinator(_problem->GetRobot(param));
		}
		else {
			throw RunTimeException(WHERE) << "Unknown decomp info parameter: " 
																		<< param 
																		<< std::endl;
		}
	}
	return decomp;
}

//Semantic Task
std::string
SemanticTaskToMessage(SemanticTask* _task) {
	std::string msg = "label="
									+ _task->GetLabel()
									+ ",subtask_relation=";
	switch(_task->GetSubtaskRelation()) {
		case SemanticTask::SubtaskRelation::AND: msg += "AND,";
							break;
		case SemanticTask::SubtaskRelation::XOR: msg += "XOR,";
							break;
		default: msg += "NONE,";
	}

	msg = msg + "decomposable="
						+ (_task->IsDecomposable() ? "true" : "false")
						+ ",fixed_assign="
						+ (_task->IsFixedAssignment() ? "true" : "false")
						+ "<dependencies=<";

	for(auto depSet : _task->GetDependencies()) {
		std::string depMsg;
		switch(depSet.first) {
			case SemanticTask::DependencyType::Completion:   
												 depMsg = "completion=<";
											   break;
			case SemanticTask::DependencyType::Initiation:   
												 depMsg = "initiation=<";
											   break;
			case SemanticTask::DependencyType::Synchronous:  
												 depMsg = "synchronous=<";
												 break;
			case SemanticTask::DependencyType::Asynchronous: 
 												 depMsg = "asynchronous=<";
												 break;
			case SemanticTask::DependencyType::None:         
												 depMsg = "none=<";
												 break;
		}

		for(auto task : depSet.second) {
			depMsg += task->GetLabel();
			//if(task != depSet.second.back())
				depMsg += ",";
		}
		depMsg += ">";
		msg += depMsg;
	}
	
	msg += ">><motion_task=<";

	auto motionTask = _task->GetMotionTask();
	if(motionTask)
		msg += MPTaskToMessage(motionTask.get());

	msg += ">>";
						
	std::cout << "Converted Semantic Task Message:::::::" << std::endl << msg << std::endl;
		
	return msg;
}

SemanticTask*
MessageToSemanticTask(std::string _msg, Decomposition* _decomp, MPProblem* _problem) {

	std::string label;
	SemanticTask* parent = nullptr;
	SemanticTask::SubtaskRelation relation;
	bool decomposable;
	bool fixed;
	std::shared_ptr<MPTask> motionTask;
	

	auto submessages = ParseSubMessage(_msg);

	std::stringstream ss(submessages[0]);
	std::string param;
	while(ss) {
		getline(ss,param,',');
		
		std::stringstream keyvalue(param);
		getline(keyvalue,param,'=');
	
		if(param == "label") {
			getline(keyvalue,param);
			label = param;
		}
		else if(param == "parent") {
			getline(keyvalue,param);
			if(param != "")
				parent = _decomp->GetTask(param);
		}
		else if(param == "subtask_relation") {
			getline(keyvalue,param);
			if(param == "AND") {
				relation = SemanticTask::SubtaskRelation::AND;
			}
			else if(param == "XOR") {
				relation = SemanticTask::SubtaskRelation::XOR;
			}
			else if(param != "NONE") {
				throw RunTimeException(WHERE) << "Unsupported subtask relationship in message." << std::endl;
			}
		}
		else if(param == "decomposable") {
			if(param == "true") {
				decomposable = true;
			}
			else {
				decomposable = false;
			}
		}
		else if(param == "fixed_assignment") {
			if(param == "true") {
				fixed = true;
			}
			else {
				fixed = false;
			}
		}
		/*else if(param == "motion_task") {
			getline(keyvalue,param);
			motionTask = std::shared_ptr<MPTask>(MessageToMPTask(param,_problem));
		}*/
	}
	

	std::stringstream dep(submessages[1]);
	getline(dep,param,'=');
	if(param != "dependencies")
		throw RunTimeException(WHERE) << "Expected dependencies in message." << std::endl;
	getline(dep,param);
	auto sub = ParseSubMessage(param);
	//TODO::Parse dependencies

	std::stringstream mpTask(submessages[2]);
	getline(mpTask,param,'=');
	if(param != "motion_task")
		throw RunTimeException(WHERE) << "Expected motion task in message." << std::endl;
	getline(mpTask,param);
	//sub = ParseSubMessage(param);
	auto mp = MessageToMPTask(param,_problem);
	if(mp) {
		motionTask = std::shared_ptr<MPTask>(mp);
		SemanticTask* task = new SemanticTask(label,parent,_decomp,relation,decomposable,fixed,motionTask);
		return task;
	}
	SemanticTask* task = new SemanticTask(label,parent,_decomp,relation,decomposable,fixed);
	return task;
}

//Motion Planning Task
std::string
MPTaskToMessage(MPTask* _mpTask) {
	std::string msg = "label="
									+ _mpTask->GetLabel()
									+ ",robot="
									+ _mpTask->GetRobot()->GetLabel()
									+ ",<start_constraint=<"
									+ ConstraintToMessage(_mpTask->GetStartConstraint())
									+ ">><goal_constraints=<";

	auto& goalConstraints = _mpTask->GetGoalConstraints();

	for(size_t i = 0; i < goalConstraints.size(); i++) {
		msg += ("<" + ConstraintToMessage(goalConstraints[i].get()) + ">");
		if(i < goalConstraints.size()-1)
			msg += ",";
	}
	msg += ">>";
	
	std::cout << "MPTask Message: " << msg << std::endl;
	return msg;
}

MPTask*
MessageToMPTask(std::string _msg, MPProblem* _problem) {

	auto submessages = ParseSubMessage(_msg);

	if(submessages[0] == "")
		return nullptr;

	MPTask* mpTask = new MPTask(nullptr);

	std::string label;
	std::string robot;

	std::stringstream ss(submessages[0]);
	std::string param;

	while(ss) {
		getline(ss,param,',');

		std::stringstream keyvalue(param);
		getline(keyvalue,param,'=');
		
		if(param == "label"){
			getline(keyvalue,label);
			mpTask->SetLabel(label);
		}
		else if(param == "robot"){
			getline(keyvalue,robot);
			mpTask->SetRobot(_problem->GetRobot(robot));
		}
	}

	std::stringstream ss2(submessages[1]);
	//while(ss2) {
		getline(ss2,param,'=');
		
		if(param == "start_constraint") {
			getline(ss2,param);

			auto subs = ParseSubMessage(param);
			auto constraint = MessageToConstraint(subs[0],mpTask->GetRobot());
			mpTask->SetStartConstraint(std::move(constraint));
		}
		else {
			throw RunTimeException(WHERE) << "Expected start constraint in message." << std::endl;
		}
	//}

	std::stringstream ss3(submessages[2]);

	//while(ss3) {
		getline(ss3,param,'=');		

		if(param == "goal_constraints") {
			getline(ss3,param);

			auto subs = ParseSubMessage(param);
			for(size_t i = 1; i < subs.size(); i++) {
				auto constraint = MessageToConstraint(subs[i], mpTask->GetRobot());
				mpTask->AddGoalConstraint(std::move(constraint));
			}
		}
		else {
			throw RunTimeException(WHERE) << "Expected goal constraints in message." << std::endl;
		}
	//}

	return mpTask;
}

std::string
ConstraintToMessage(const Constraint* _constraint) {
	std::string msg = "type=";
	std::string type(typeid(*_constraint).name());
	msg += (type + ",data=[");

	auto boundary = _constraint->GetBoundary();
	for(size_t i = 0; i < boundary->GetDimension(); i++) {
		auto range = boundary->GetRange(i);
		msg += std::to_string(range.min) + ":" + std::to_string(range.max);
		if(i < boundary->GetDimension()-1)
			msg += ";";
	}
	msg += "]";
	return msg;
}

std::unique_ptr<Constraint>
MessageToConstraint(std::string _msg, Robot* _robot) {
	std::stringstream ss(_msg);
	
	std::string type;
	std::string data;

	while(ss) {
		std::string param;
		getline(ss,param,',');

		std::stringstream keyvalue(param);
		getline(keyvalue,param,'=');
		
		if(param == "type") {
			getline(keyvalue,param);
			type = param;
		}
		else if(param == "data") {
			getline(keyvalue,param);
			data = param;
		}
	}

	if(type.find("CSpaceConstraint") != std::string::npos) {
		CSpaceConstraint* constraint = new CSpaceConstraint(_robot, "", data);
		return std::unique_ptr<CSpaceConstraint>(constraint);
	}
	else if(type.find("BoundaryConstraint") != std::string::npos) {
		throw RunTimeException(WHERE) << "Boundary Constraints are not yet supported in the message system." << std::endl;
		return nullptr;
	}
	else {
		throw RunTimeException(WHERE) << "Unrecognized constraint type." << std::endl;
	}
	
	return nullptr;
}
	
