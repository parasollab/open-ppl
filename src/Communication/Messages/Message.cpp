#include "Message.h"

#include <list>
#include <sstream>
#include <typeinfo>

#include <containers/sequential/graph/graph_util.h>

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

/*--------------------------------------- Query Info --------------------------------*/

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

	for(auto task : _decomp->GetMotionTasks()) {
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
									+ ",parent="
									+ (_task->GetParent() ? _task->GetParent()->GetLabel() : "")
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
	
/*---------------------------------- Plan Info ---------------------------------------*/

//Plan
std::string
PlanToMessage(Plan* _plan) {
	std::string msg = "plan/<";
	
	auto top = _plan->GetDecomposition()->GetMainTask();

	std::vector<SemanticTask*> pq;
	pq.push_back(top);

	size_t total = _plan->GetTaskSolutions().size();
	
	msg += "<task_solutions=";
	//for(size_t i = 0; i < total; i++) {
	size_t counter = 0;
	size_t sols = 0;
	while(counter < pq.size()) {
		auto task = pq[counter];
		for(auto sub : task->GetSubtasks()) {
			pq.push_back(sub);
		}
	
		auto sol = _plan->GetTaskSolution(task);
		if(sol) {
			msg += ("<" + TaskSolutionToMessage(sol) + ">");
			sols++;
		}
		counter++;
	}

	if(sols != total)
		throw RunTimeException(WHERE) << "Unequal number of solution messages and solutions." << std::endl;

	msg += ">>";

	return msg;
}

Plan*
MessageToPlan(std::string _msg, Decomposition* _decomp, MPProblem* _problem) {
	Plan* plan = new Plan();
	plan->SetDecomposition(_decomp);
	plan->SetMPProblem(_problem);

	auto submessages = ParseSubMessage(_msg);

	std::cout << "Parsed Plan Message: " << std::endl;
	for(auto sub : submessages) {
		std::cout << std::endl << sub << std::endl;
	}

	if(submessages[0] != "plan/")
		throw RunTimeException(WHERE) << "Expected plan header in message." << std::endl;

	auto taskSolutions = ParseSubMessage(submessages[1]);
	if(taskSolutions[0] != "task_solutions=")
		throw RunTimeException(WHERE) << "Expected task solutions." << std::endl;

	for(size_t i = 1; i < taskSolutions.size(); i++) {
		auto sol = std::shared_ptr<TaskSolution>(MessageToTaskSolution(taskSolutions[i],plan));
		plan->SetTaskSolution(sol->GetTask(),sol);
		if(!_decomp->GetTask(sol->GetTask()->GetLabel())) {
			_decomp->AddTask(std::shared_ptr<SemanticTask>(sol->GetTask()));
		}
	}
	
	//Motion Plans
	auto roadmaps = ParseSubMessage(submessages[2]);
	if(roadmaps[0] != "roadmaps=")
		throw RunTimeException(WHERE) << "Expected robot roadmaps." << std::endl;

	std::unordered_map<Robot*,RoadmapGraph<Cfg,DefaultWeight<Cfg>>*> roadmapMap;

	std::cout << "Roadmaps"  << std::endl << std::endl;
	for(size_t i = 1; i < roadmaps.size(); i++) {
		auto subs = ParseSubMessage(roadmaps[i]);
		std::stringstream ss(subs[0]);
		//std::stringstream ss(roadmaps[i]);
		std::string robotLabel;
		getline(ss,robotLabel,'=');
		if(robotLabel != "robot")
			throw RunTimeException(WHERE) << "Expected robot label." << std::endl;
		getline(ss,robotLabel);
		auto robot = _problem->GetRobot(robotLabel);
		std::string roadmapMsg;
		getline(ss,roadmapMsg); 
		auto roadmap = MessageToRoadmap(subs[1],robot);
		roadmapMap[robot] = roadmap;
	}

	auto motionPlans = ParseSubMessage(submessages[3]);
	if(motionPlans[0] != "motion_plans=")
		throw RunTimeException(WHERE) << "Expected motion plans." << std::endl;

	std::cout << "Motion Plans: " << std::endl;
	for(size_t i = 1; i < motionPlans.size(); i++) {
		//std::cout << motionPlans[i] << std::endl;
		std::string task;
		std::string robot;

		auto subs = ParseSubMessage(motionPlans[i]);
		std::stringstream ss(subs[0]);
		std::string label;
		getline(ss,label,',');

		std::stringstream taskLabel(label);
		getline(taskLabel,label,'=');
		if(label != "task")
			throw RunTimeException(WHERE) << "Expected task label in motion plan." << std::endl;
		getline(taskLabel,task);

		getline(ss,label);
		std::stringstream robotLabel(label);
		getline(robotLabel,label,'=');
		if(label != "robot") 
			throw RunTimeException(WHERE) << "Expected robot label in motion plan." << std::endl;
		getline(robotLabel,robot);	

		//TODO:: Extrcact path from subs[1]
		std::stringstream vidSS(subs[1]);
		std::vector<size_t> vids;
		std::string vid;
		while(vidSS) {
			getline(vidSS,vid,',');
			if(vid == "")
				break;
			vids.push_back(std::atoi(vid.c_str()));
		}
		
		MPSolutionType<MPTraits<Cfg>>* motionSolution = new MPSolutionType<MPTraits<Cfg>>(_problem->GetRobot(robot));
		motionSolution->SetRoadmap(_problem->GetRobot(robot),roadmapMap[_problem->GetRobot(robot)]);
		PathType<MPTraits<Cfg>>* path = new PathType<MPTraits<Cfg>>(roadmapMap[_problem->GetRobot(robot)]);
		*path += vids;
		motionSolution->SetPath(_problem->GetRobot(robot),path);		

		auto semanticTask = _decomp->GetTask(task);
		auto taskSolution = plan->GetTaskSolution(semanticTask);
		taskSolution->SetMotionSolution(motionSolution);
	}
	
	plan->Print();
	return plan;
}

//Task Solution
std::string
TaskSolutionToMessage(TaskSolution* _solution) {
	auto task = _solution->GetTask();
	std::string msg = "label="
									+	task->GetLabel()
									+ ",parent=";

	if(task->GetParent())
		msg += (task->GetParent()->GetLabel());
	else 
		msg += "none";
	
	msg += (",start_time=" + std::to_string(_solution->GetStartTime()) + ",<");

	if(_solution->GetRobot()) {
		msg += ("<" + _solution->GetRobot()->GetLabel() + ">");
	}
	else if(_solution->GetRobotGroup()) {
		for(auto robot : _solution->GetRobotGroup()->GetRobots()) {
			msg += ("<" + robot->GetLabel() + ">");
		}
	}
	msg += ">";
	return msg;
}

TaskSolution*
MessageToTaskSolution(std::string _msg, Plan* _plan) {
	//TaskSolution* solution = new TaskSolution(_task);

	std::stringstream ss(_msg);

	//Parse task label
	std::string label;
	getline(ss,label,',');
	
	std::stringstream labelSS(label);
	getline(labelSS,label,'=');

	if(label != "label")
		throw RunTimeException(WHERE) << "Expected a task label." << std::endl;

	getline(labelSS,label);
	
	//Parse parent task label 
	std::string parentLabel;
	getline(ss,parentLabel,',');
	std::stringstream parentSS(parentLabel);
	getline(parentSS,parentLabel,'=');

	if(parentLabel != "parent")
		throw RunTimeException(WHERE) << "Expected a parent label." << std::endl;

	getline(parentSS,parentLabel);	

	//Fetch or create parsed semantic task
	auto decomp = _plan->GetDecomposition();

	auto parent = decomp->GetTask(parentLabel);
	if(!parent and parentLabel != "none")
		throw RunTimeException(WHERE) << "Parent task does not exist in the decomposition." << std::endl;

	auto task = decomp->GetTask(label);
	if(!task)
		task = new SemanticTask(label,parent,decomp);
	
	TaskSolution* solution = new TaskSolution(task);

	//Parse task start time
	std::string startTimeLabel;
	getline(ss,startTimeLabel,',');
	std::stringstream startTimeSS(startTimeLabel);
	getline(startTimeSS,startTimeLabel,'=');
	
	if(startTimeLabel != "start_time")
		throw RunTimeException(WHERE) << "Expected a task start time." << std::endl;

	getline(startTimeSS,startTimeLabel);
	double startTime = std::atoi(startTimeLabel.c_str());
	solution->SetStartTime(startTime);

	//Parse allocated robots
	std::string robotLabels;
	getline(ss,robotLabels);

	auto robots = ParseSubMessage(robotLabels);

	if(robots.size() == 2) {
		solution->SetRobot(_plan->GetMPProblem()->GetRobot(robots[1]));
	}
	else if(robots.size() > 2) {
		std::vector<Robot*> group;
		for(size_t i = 1; i < robots.size(); i++) {
			group.push_back(_plan->GetMPProblem()->GetRobot(robots[i]));
		}
		RobotGroup* robotGroup = new RobotGroup(_plan->GetMPProblem(),task->GetLabel(),group);
		solution->SetRobotGroup(robotGroup);
	}
	
	return solution;
}

//Motion Solution
std::vector<std::string>
MotionSolutionToMessage(MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>* _solution, std::string _label) {
	std::vector<std::string> messages;

	auto robot = _solution->GetRobot();
	std::string msg = "<<task="
									+ _label
									+ ",robot="
									+ robot->GetLabel()
									+ "<";

	messages.push_back(msg);

	//auto roadmap = RoadmapToMessage(_solution->GetRoadmap(robot));
	//for(auto m : roadmap) {
	//	messages.push_back(m);
	//}

	std::string path;
	for(auto vid : _solution->GetPath()->VIDs()) {
		path += (std::to_string(vid) + ",");
	}
	path += ">>>";
	messages.push_back(path);

	return messages;
}

MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>*
MessageToMotionSolution(std::string _msg, Robot* _robot, RobotGroup* _group) {
	MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>* motion = nullptr;
	if(_robot) 
		motion = new MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>(_robot);
	else if(_group)
		motion = new MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>(_group);
	else
		throw RunTimeException(WHERE) << "Need a robot or robot group for the motion solution." << std::endl;

	

	return motion;
}

//Roadmap
std::vector<std::string>
RoadmapToMessage(RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _roadmap, Robot* _robot) {

	std::ostringstream oss;
	stapl::sequential::write_graph(*_roadmap,oss);
	std::string msg = oss.str();

	std::cout << msg << std::endl;

	std::vector<std::string> messages;
	std::string chunk = "<<robot=" 
										+ _robot->GetLabel()
										+"<";
	size_t counter = chunk.size();
	for(size_t i = 0; i < msg.size(); i++) {
		if(counter == 1021) {
			messages.push_back(chunk);
			chunk = "";
			counter = 0;
		}
		chunk += msg[i];
		counter++;
	}
	chunk += ">>>";
	messages.push_back(chunk);
	return messages;
}

RoadmapGraph<Cfg,DefaultWeight<Cfg>>*
MessageToRoadmap(std::string _msg, Robot* _robot) {
	auto roadmap = new RoadmapGraph<Cfg,DefaultWeight<Cfg>>(_robot);
	ReadMessage(roadmap,_msg);
	return roadmap;
}

/*------------------------------- Control Info  -------------------------------------*/

//Control 
std::string
ControlSetToMessage(ControlSet& _c, size_t _steps) {
	std::string msg = "control_set/<steps="
									+ std::to_string(_steps);

	for(auto control : _c) {
		auto actuator = control.actuator;
		auto robot = actuator->GetRobot();
		
		msg += ("<robot=" 
						+ robot->GetLabel()
						+ ",actuator="
						+ actuator->GetLabel()
						+ ",signal="
					 );

		for(auto d : control.signal) {
			msg += (std::to_string(d) + " ");
		}

		msg += ">";
	}

	msg += ">";	

	return msg;
}

std::pair<size_t,ControlSet>
MessageToControlSet(std::string _msg, MPProblem* _problem) {
	ControlSet c;

	std::stringstream ss(_msg);
	std::string label;
	getline(ss,label,'/');
	if(label != "control_set")
		throw RunTimeException(WHERE) << "Expected a control set title." << std::endl;

	getline(ss,label);
	auto submessages = ParseSubMessage(label);

	std::stringstream stepsSS(submessages[0]);
	getline(stepsSS,label,'=');
	if(label != "steps")
		throw RunTimeException(WHERE) << "Expected number of steps for controls." << std::endl;

	size_t steps;
	stepsSS >> steps;

	for(size_t i = 1; i < submessages.size(); i++) {
		std::stringstream controlSS(submessages[i]);
		std::string robotLabel;
		getline(controlSS,robotLabel,',');
		std::stringstream rlSS(robotLabel);
		getline(rlSS,robotLabel,'=');
		if(robotLabel != "robot")
			throw RunTimeException(WHERE) << "Expected a robot label." << std::endl;
		getline(rlSS,robotLabel);
		auto robot = _problem->GetRobot(robotLabel);

		std::string actuatorLabel;
		getline(controlSS,actuatorLabel,',');
		std::stringstream alSS(actuatorLabel);
		getline(alSS,actuatorLabel,'=');
		if(actuatorLabel != "actuator")
			throw RunTimeException(WHERE) << "Expected an acutator label." << std::endl;
		getline(alSS,actuatorLabel);
		auto actuator = robot->GetActuator(actuatorLabel);
	
		std::string signalLabel;
		getline(controlSS,signalLabel,'=');
		if(signalLabel != "signal")
			throw RunTimeException(WHERE) << "Expected a control signal." << std::endl;

		std::vector<double> signal;
		while(controlSS) {
			double d;
			controlSS >> d;
			signal.push_back(d);
		}

		Control control(actuator,signal);
		c.push_back(control);
	}

	return std::make_pair(steps,c);
}
