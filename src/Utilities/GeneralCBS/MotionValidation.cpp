#include "MotionValidation.h"

#include "Behaviors/Agents/HandoffAgent.h"
#include "ConfigurationSpace/Path.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "Traits/CfgTraits.h"

MotionValidation::
MotionValidation() {}	

MotionValidation::
MotionValidation(MPLibrary* _library, LowLevelSearch* _lowLevel, bool _avoidancePaths,
								TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel) :
								Validation(_library,_lowLevel, _tmpLibrary), 
								m_avoidancePaths(_avoidancePaths), m_sgLabel(_sgLabel), m_vcLabel(_vcLabel) {}	

MotionValidation::
~MotionValidation() {}

bool
MotionValidation::
InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) {

	CBSSolution solution;
	solution.m_decomposition = _decomposition;

	std::unordered_map<Agent*,std::list<Assignment>> agentAssignments;

	auto top = _decomposition->GetMainTask();
	for(auto task : top->GetSubtasks()) {
		solution.m_solutionTasks.insert(task);
		for(auto subtask : task->GetSubtasks()) {
			Assignment a;
			a.m_task = subtask;
			a.m_agent = subtask->GetMotionTask()->GetRobot()->GetAgent();
			solution.m_taskPlans[task].push_back(a);

			// Looking for correct ordering of agent assignments.
			
			if(agentAssignments[a.m_agent].empty()) {
				agentAssignments[a.m_agent].push_back(a);
				continue;
			}

			bool found = false;
			auto iter = agentAssignments[a.m_agent].begin();
			for(;iter != agentAssignments[a.m_agent].end(); iter++) {
				auto depends = (*iter).m_task->GetDependencies()[SemanticTask::Completion];
				for(auto d : depends) {
					if(d == subtask) {
						found = true;
						break;
					}	
				}
				if(found)
					break;
			}
			if(iter == agentAssignments[a.m_agent].end())
				agentAssignments[a.m_agent].push_back(a);
			else 
				agentAssignments[a.m_agent].insert(iter,a);
		}
	}

	for(auto& kv : agentAssignments) {
		for(auto iter = kv.second.begin(); iter != kv.second.end(); iter++) {
			solution.m_agentAssignments[kv.first].push_back(*iter);
		}
	}

	GeneralCBSNode node(solution);
	if(m_lowLevel->UpdateSolution(node,top->GetSubtasks()[0])) {
		_tree.push(node);
		return true;
	}
	else 
		return false;
}

bool 
MotionValidation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {

	if(m_avoidancePaths)
		AvoidancePaths(_node);

	auto constraints = FindMotionConflict(_node);
	
	if(constraints.first.m_timestep != MAX_INT){
		AddMotionChildren(_node, _tree, constraints);
		return false;
	}

	return true;
}
	
std::pair<MotionConstraint,MotionConstraint>
MotionValidation::
FindMotionConflict(GeneralCBSNode& _node) {
	MotionConstraint one;
	MotionConstraint two;

	const auto& agentAssignments = _node.GetSolution().m_agentAssignments;

	//TODO::Need this to get the agent roadmap - later replace by accessing the MPSolution 
	//Currently don't store anything in the MPLibrary's solution object
	std::unordered_map<Agent*,std::vector<Cfg>> agentPaths;

	std::unordered_map<Agent*,std::vector<std::pair<
					std::pair<size_t,size_t>,SemanticTask*>>> pathTaskMap;

	auto sg = static_cast<CombinedRoadmap*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

	for(auto kv : agentAssignments) {
		auto agent = kv.first;
		agentPaths[agent] = {};

		auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(agent));

		auto assigns = kv.second;
		if(m_avoidancePaths) {
			auto postAssignment = _node.GetPostAssignmentRef().m_agentAssignments[agent][0];
			assigns.push_back(postAssignment);
		}
	
		//Cfg previousCfg = agent->GetRobot()->GetSimulationModel()->GetState();
		Cfg previousCfg = m_library->GetMPProblem()->GetInitialCfg(agent->GetRobot());
		size_t previousTimeStep = 0;
	
		for(size_t i = 0; i < assigns.size(); i++) {
			auto a = assigns[i];
			if(i != assigns.size()-1
							and !_node.GetSolutionRef().m_solutionTasks.count(a.m_task->GetParent())
							and !_node.GetSolutionRef().m_solutionTasks.empty())
				continue;

			std::vector<Cfg> interim;


			PathType<MPTraits<Cfg,DefaultWeight<Cfg>>> setupPath(roadmap.get());
			//remove last element of setup as it is the same as the exec path
			auto setup = a.m_setupPath;
			//setup.pop_back();
			setupPath += setup;
			setupPath.SetFinalWaitTimeSteps(a.m_setupWaitTimeSteps);
			//path += a.m_execPath;
			std::vector<Cfg> cfgs;
			if(!sg->m_discrete) 
				cfgs = setupPath.FullCfgs(m_library);
			else { 
				auto verts = setupPath.Cfgs();

				if(!verts.empty())
					cfgs.push_back(verts[0]);
				for(size_t j = 1; j < verts.size(); j++) {
					Cfg middle = verts[j];
					middle.SetData({(verts[j-1][0]+verts[j][0])/2, (verts[j-1][1]+verts[j][1])/2, 
													(verts[j-1][2]+verts[j][2])/2});
					cfgs.push_back(middle);
					cfgs.push_back(verts[j]);
				}
				for(size_t j = 0; j < a.m_setupWaitTimeSteps; j++) {
					cfgs.push_back(cfgs.back());
				}
			}

			size_t interimSteps = 0;
			if(a.m_execStartTime != a.m_setupStartTime)
				interimSteps = a.m_execStartTime - (cfgs.size()+previousTimeStep);
			if(!cfgs.empty() and interimSteps > 0)
				interimSteps++;

			if(m_debug) 
				std::cout << "Interim Steps: " << interimSteps << std::endl;

			//for(size_t j = 0; j < a.m_execStartTime - (cfgs.size()-1+previousTimeStep); j++) {
			for(size_t j = 0; j < interimSteps; j++) {
				interim.push_back(previousCfg);
			}

			if(!cfgs.empty())
				cfgs.pop_back();

			PathType<MPTraits<Cfg,DefaultWeight<Cfg>>> execPath(roadmap.get());
			//if(a.m_execStartTime != a.m_execEndTime and a.m_finalWaitTimeSteps > 0) {
			execPath += a.m_execPath;
			execPath.SetFinalWaitTimeSteps(a.m_finalWaitTimeSteps);
			//}

			std::vector<Cfg> execCfgs;
			if(!sg->m_discrete)
				execCfgs = execPath.FullCfgs(m_library);
			else {
				auto verts = execPath.Cfgs();

				if(!verts.empty())
					execCfgs.push_back(verts[0]);
				for(size_t j = 1; j < verts.size(); j++) {
					Cfg middle = verts[j];
					middle.SetData({(verts[j-1][0]+verts[j][0])/2, (verts[j-1][1]+verts[j][1])/2, 
													(verts[j-1][2]+verts[j][2])/2});
					execCfgs.push_back(middle);
					execCfgs.push_back(verts[j]);
				}
				for(size_t j = 0; j < a.m_finalWaitTimeSteps; j++) {
					execCfgs.push_back(execCfgs.back());
				}
			}

			previousCfg = execCfgs.back();

			//if(i < assigns.size()-1) {
				execCfgs.pop_back();
				//previousTimeStep = 1;
			//}	

			size_t start = agentPaths[agent].size();
			size_t end = start+interim.size()+cfgs.size()+execCfgs.size();

			if(i > 0 and i == assigns.size()-1 and a.m_execStartTime != a.m_execEndTime)
				end = end-1;

			previousTimeStep = end;

			if(end != a.m_execEndTime) {
				std::cout << "End: " << end << " execEndTime: " << a.m_execEndTime << std::endl;

				throw RunTimeException(WHERE, "Path lengths do not match up in Motion Validation for ." +
																			agent->GetRobot()->GetLabel());
			}
			
			agentPaths[agent].insert(agentPaths[agent].end(),interim.begin(),interim.end());
			agentPaths[agent].insert(agentPaths[agent].end(),cfgs.begin(),cfgs.end());
			agentPaths[agent].insert(agentPaths[agent].end(),execCfgs.begin(),execCfgs.end());

			pathTaskMap[agent].push_back(std::make_pair(std::make_pair(start,end-1),a.m_task->GetParent()));
		}
		
	}

	//Find the last timestep a robot is moving.
	size_t lastTimestep = 0;
	for(auto& kv : agentPaths) {
		lastTimestep = std::max(lastTimestep,kv.second.size());
	}

	auto vc = static_cast<CollisionDetectionValidity<MPTraits<Cfg,DefaultWeight<Cfg>>>*>(
			m_library->GetValidityChecker(m_vcLabel));

	for(size_t t = 0; t < lastTimestep; ++t) {
		// Collision check each robot against all other at this timestep.
		for(auto iter1 = agentPaths.begin(); iter1 != agentPaths.end();) {
			// Configure the first robot at the appropriate configuration.
			auto robot1 				= iter1->first->GetRobot();
			const auto& path1 	= iter1->second;
			const size_t step1 	= std::min(t, path1.size() - 1);
			// If the agent has no assignments, check against its initial location
			auto cfg1 					= (!path1.empty()) ? path1[step1] : 
																m_library->GetMPProblem()->GetInitialCfg(robot1);
																//robot1->GetSimulationModel()->GetState();
			//auto cfg1 = path1[step1];
			cfg1.SetRobot(robot1);
			auto multibody1 		= robot1->GetMultiBody();
			multibody1->Configure(cfg1);

			// Compare to all remaining robots.
			for(auto iter2 = ++iter1; iter2 != agentPaths.end(); ++iter2) {
				//Configure the second rbot at the appropriate configuration.
				auto robot2 				= iter2->first->GetRobot();
				const auto& path2		= iter2->second;
				const size_t step2 	= std::min(t, path2.size() - 1);
				// If the agent has no assignments, check against its initial location
				auto cfg2						= (!path2.empty()) ? path2[step2] :
																m_library->GetMPProblem()->GetInitialCfg(robot2);
																//robot2->GetSimulationModel()->GetState();
				//auto cfg2 = path2[step2];
				cfg2.SetRobot(robot2);
				auto multibody2 		= robot2->GetMultiBody();
				multibody2->Configure(cfg2);

				// Check for collitision. If none, move on.
				if(!sg->m_discrete) {
					CDInfo cdInfo;
					const bool collision = vc->IsMultiBodyCollision(cdInfo,
						multibody1, multibody2, "MotionValidation");
					if(!collision)
						continue;
				}
				else {
					const bool collision = (cfg1[0] == cfg2[0]
											and cfg1[1] == cfg2[1]
											and cfg1[2] == cfg2[2]);
					if(!collision)
						continue;
				}

				if(m_debug) {
					std::cout << "\t\tConflict detected at timestep " << t
										<< " (time " << m_library->GetMPProblem()->GetEnvironment()->GetTimeRes() * t
										<< ")."
										<< "\n\t\t\tRobot " << robot1->GetLabel() << ": "
										<< cfg1.PrettyPrint()
										<< "\n\t\t\tRobot " << robot2->GetLabel() << ": "
										<< cfg2.PrettyPrint()
										<< std::endl;
				}

				one.m_agent = robot1->GetAgent();
				one.m_conflictCfg = cfg2;
				one.m_timestep = t;
				if(!path1.empty()) {
					for(auto task : pathTaskMap[one.m_agent]) {
						if(t >= task.first.first and t <= task.first.second) {
							one.m_task = task.second;
						}
					}

					if(t >= path2.size()-1 or path2.empty()) {
						one.m_duration = std::numeric_limits<size_t>::infinity();
					}

				}

				two.m_agent = robot2->GetAgent();
				two.m_conflictCfg = cfg1;
				two.m_timestep = t;
				if(!path2.empty()) {
					for(auto task : pathTaskMap[two.m_agent]) {
						if(t >= task.first.first and t <= task.first.second) {
							two.m_task = task.second;
						}
					}
					if(t >= path1.size()-1 or path1.empty()) {
						two.m_duration = std::numeric_limits<size_t>::infinity();
					}
				}
				return std::make_pair(one,two);
			}
		}
	}
	
	//Reset multibodies at initial location - not sure if this affects SimulatedState
	for(auto kv : agentPaths) {
		if(kv.second.empty())
			continue;
		auto cfg = kv.second[0];
		auto robot = kv.first->GetRobot();
		cfg.SetRobot(robot);
		robot->GetMultiBody()->Configure(cfg);
	}

	return std::make_pair(one,two);
}

void 
MotionValidation::
AddMotionChildren(GeneralCBSNode& _node, GeneralCBSTree& _tree, MotionConstraintPair _constraints) {
		
	if(_constraints.first.m_task) {
		GeneralCBSNode one(_node);
		one.AddMotionConstraint(_constraints.first,_constraints.first.m_agent);

		auto& tp = one.GetSolutionRef().m_taskPlans[_constraints.first.m_task];
		auto& aa = one.GetSolutionRef().m_agentAssignments[_constraints.first.m_agent];
		for(auto& a : tp) {
			if(a.m_agent != _constraints.first.m_agent)
				continue;

			if(a.m_setupStartTime > _constraints.first.m_timestep
					or a.m_execEndTime < _constraints.first.m_timestep)
				continue;

			for(auto& assign : aa) {
				if(assign == a) {
					assign.m_execPath = {};
				}
			}

			a.m_execPath = {};
			
			break;
		}

		if(m_lowLevel->UpdateSolution(one, _constraints.first.m_task))
			_tree.push(one);

		if(one.GetCost() < _node.GetCost())
			throw RunTimeException(WHERE,"Child node cost less than parnt.");


 	}

	if(_constraints.second.m_task) {
		GeneralCBSNode two(_node);
		two.AddMotionConstraint(_constraints.second,_constraints.second.m_agent);

		auto& tp = two.GetSolutionRef().m_taskPlans[_constraints.second.m_task];
		auto& aa = two.GetSolutionRef().m_agentAssignments[_constraints.second.m_agent];
		for(auto& a : tp) {
			if(a.m_agent != _constraints.second.m_agent)
				continue;

			if(a.m_setupStartTime > _constraints.second.m_timestep
					or a.m_execEndTime < _constraints.second.m_timestep)
				continue;

			for(auto& assign : aa) {
				if(assign == a) {
					assign.m_execPath = {};
				}
			}

			a.m_execPath = {};
			break;
		}

		if(m_lowLevel->UpdateSolution(two, _constraints.second.m_task))
			_tree.push(two);

		if(two.GetCost() < _node.GetCost())
			throw RunTimeException(WHERE,"Child node cost less than parnt.");

	}
}

void
MotionValidation::
AvoidancePaths(GeneralCBSNode& _node) {

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	
	auto solution = _node.GetSolution();
	auto agentAssigns = solution.m_agentAssignments;

	auto& post = _node.GetPostAssignmentRef();
	auto& postAA = post.m_agentAssignments;

	for(auto aa : agentAssigns) {
		auto agent = aa.first;
		auto& assign = postAA[agent][0];

		size_t start = 0;
		if(!aa.second.empty()) {
			//start = aa.second.back().m_execEndTime;
			for(int i = aa.second.size()-1; i >= 0; i--) {
				auto& lastAssign = aa.second[i];
				if(_node.GetSolutionRef().m_solutionTasks.empty() 
						or _node.GetSolutionRef().m_solutionTasks.count(lastAssign.m_task->GetParent()))
					start = lastAssign.m_execEndTime;
			}
		}

		auto g = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(agent));

		double maxTime = 0;

		auto& constraints = _node.GetMotionConstraints(assign.m_task->GetParent(),agent);
		for(auto& c : constraints) {
			m_lowLevel->m_motionConstraintMap[agent].insert(std::make_pair(c.m_timestep,
				std::make_pair(c.m_conflictCfg,c.m_duration)));
			if(c.m_timestep > maxTime)
				maxTime = c.m_timestep;
		}
	
		Cfg lastCfg(agent->GetRobot());
		if(start == 0) {
				//lastCfg = agent->GetRobot()->GetSimulationModel()->GetState();
				lastCfg = m_library->GetMPProblem()->GetInitialCfg(agent->GetRobot());
		}
		else {
				lastCfg = g->GetVertex(aa.second.back().m_execPath.back());
		}	
		lastCfg.SetRobot(agent->GetRobot());

		auto plan = m_lowLevel->LowLevelSearch::MotionPlan(lastCfg, lastCfg, start, maxTime, 
																			assign.m_task->GetParent());

		m_lowLevel->m_motionConstraintMap.clear();	

		if(plan.second.first.empty())
			throw RunTimeException(WHERE, "Problem with post assignment path.");

		assign.m_setupStartTime = start;
		assign.m_execStartTime = start;
		assign.m_execEndTime = plan.first;

		assign.m_setupPath = {};		
		assign.m_execPath = plan.second.first;
		assign.m_finalWaitTimeSteps = plan.second.second;
	}	
}

size_t 
MotionValidation::
CountConflicts(GeneralCBSNode& _node) {

	size_t conflictCount = 0;

	const auto& agentAssignments = _node.GetSolution().m_agentAssignments;

	//TODO::Need this to get the agent roadmap - later replace by accessing the MPSolution 
	//Currently don't store anything in the MPLibrary's solution object
	std::unordered_map<Agent*,std::vector<Cfg>> agentPaths;

	std::unordered_map<Agent*,std::vector<std::pair<
					std::pair<size_t,size_t>,SemanticTask*>>> pathTaskMap;

	auto sg = static_cast<CombinedRoadmap*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

	for(auto kv : agentAssignments) {
		auto agent = kv.first;
		agentPaths[agent] = {};

		auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(agent));

		auto assigns = kv.second;
		auto postAssignment = _node.GetPostAssignmentRef().m_agentAssignments[agent][0];
		assigns.push_back(postAssignment);
	
		//Cfg previousCfg = agent->GetRobot()->GetSimulationModel()->GetState();
		Cfg previousCfg = m_library->GetMPProblem()->GetInitialCfg(agent->GetRobot());
		size_t previousTimeStep = 0;
	
		for(size_t i = 0; i < assigns.size(); i++) {
			auto a = assigns[i];
			if(i != assigns.size()-1
							and !_node.GetSolutionRef().m_solutionTasks.count(a.m_task->GetParent())
							and !_node.GetSolutionRef().m_solutionTasks.empty())
				continue;

			std::vector<Cfg> interim;


			PathType<MPTraits<Cfg,DefaultWeight<Cfg>>> setupPath(roadmap.get());
			//remove last element of setup as it is the same as the exec path
			auto setup = a.m_setupPath;
			//setup.pop_back();
			setupPath += setup;
			setupPath.SetFinalWaitTimeSteps(a.m_setupWaitTimeSteps);
			//path += a.m_execPath;
			std::vector<Cfg> cfgs;
			if(!sg->m_discrete) 
				cfgs = setupPath.FullCfgs(m_library);
			else { 
				auto verts = setupPath.Cfgs();

				if(!verts.empty())
					cfgs.push_back(verts[0]);
				for(size_t j = 1; j < verts.size(); j++) {
					Cfg middle = verts[j];
					middle.SetData({(verts[j-1][0]+verts[j][0])/2, (verts[j-1][1]+verts[j][1])/2, 
													(verts[j-1][2]+verts[j][2])/2});
					cfgs.push_back(middle);
					cfgs.push_back(verts[j]);
				}
				for(size_t j = 0; j < a.m_setupWaitTimeSteps; j++) {
					cfgs.push_back(cfgs.back());
				}
			}

			size_t interimSteps = 0;
			if(a.m_execStartTime != a.m_setupStartTime)
				interimSteps = a.m_execStartTime - (cfgs.size()+previousTimeStep);
			if(!cfgs.empty() and interimSteps > 0)
				interimSteps++;

			if(m_debug) 
				std::cout << "Interim Steps: " << interimSteps << std::endl;

			//for(size_t j = 0; j < a.m_execStartTime - (cfgs.size()-1+previousTimeStep); j++) {
			for(size_t j = 0; j < interimSteps; j++) {
				interim.push_back(previousCfg);
			}

			if(!cfgs.empty())
				cfgs.pop_back();

			PathType<MPTraits<Cfg,DefaultWeight<Cfg>>> execPath(roadmap.get());
			//if(a.m_execStartTime != a.m_execEndTime and a.m_finalWaitTimeSteps > 0) {
			execPath += a.m_execPath;
			execPath.SetFinalWaitTimeSteps(a.m_finalWaitTimeSteps);
			//}

			std::vector<Cfg> execCfgs;
			if(!sg->m_discrete)
				execCfgs = execPath.FullCfgs(m_library);
			else {
				auto verts = execPath.Cfgs();

				if(!verts.empty())
					execCfgs.push_back(verts[0]);
				for(size_t j = 1; j < verts.size(); j++) {
					Cfg middle = verts[j];
					middle.SetData({(verts[j-1][0]+verts[j][0])/2, (verts[j-1][1]+verts[j][1])/2, 
													(verts[j-1][2]+verts[j][2])/2});
					execCfgs.push_back(middle);
					execCfgs.push_back(verts[j]);
				}
				for(size_t j = 0; j < a.m_finalWaitTimeSteps; j++) {
					execCfgs.push_back(execCfgs.back());
				}
			}

			previousCfg = execCfgs.back();

			//if(i < assigns.size()-1) {
				execCfgs.pop_back();
				//previousTimeStep = 1;
			//}	

			size_t start = agentPaths[agent].size();
			size_t end = start+interim.size()+cfgs.size()+execCfgs.size();

			if(i == assigns.size()-1 and a.m_execStartTime != a.m_execEndTime)
				end = end-1;

			previousTimeStep = end;

			if(end != a.m_execEndTime) {
				std::cout << "End: " << end << " execEndTime: " << a.m_execEndTime << std::endl;

				throw RunTimeException(WHERE, "Path lengths do not match up in Motion Validation for ." +
																			agent->GetRobot()->GetLabel());
			}
			
			agentPaths[agent].insert(agentPaths[agent].end(),interim.begin(),interim.end());
			agentPaths[agent].insert(agentPaths[agent].end(),cfgs.begin(),cfgs.end());
			agentPaths[agent].insert(agentPaths[agent].end(),execCfgs.begin(),execCfgs.end());

			pathTaskMap[agent].push_back(std::make_pair(std::make_pair(start,end-1),a.m_task->GetParent()));
		}
		
	}

	//Find the last timestep a robot is moving.
	size_t lastTimestep = 0;
	for(auto& kv : agentPaths) {
		lastTimestep = std::max(lastTimestep,kv.second.size());
	}

	auto vc = static_cast<CollisionDetectionValidity<MPTraits<Cfg,DefaultWeight<Cfg>>>*>(
			m_library->GetValidityChecker(m_vcLabel));

	for(size_t t = 0; t < lastTimestep; ++t) {
		// Collision check each robot against all other at this timestep.
		for(auto iter1 = agentPaths.begin(); iter1 != agentPaths.end();) {
			// Configure the first robot at the appropriate configuration.
			auto robot1 				= iter1->first->GetRobot();
			const auto& path1 	= iter1->second;
			const size_t step1 	= std::min(t, path1.size() - 1);
			// If the agent has no assignments, check against its initial location
			auto cfg1 					= (!path1.empty()) ? path1[step1] : 
															robot1->GetSimulationModel()->GetState();
			//auto cfg1 = path1[step1];
			cfg1.SetRobot(robot1);
			auto multibody1 		= robot1->GetMultiBody();
			multibody1->Configure(cfg1);

			// Compare to all remaining robots.
			for(auto iter2 = ++iter1; iter2 != agentPaths.end(); ++iter2) {
				//Configure the second rbot at the appropriate configuration.
				auto robot2 				= iter2->first->GetRobot();
				const auto& path2		= iter2->second;
				const size_t step2 	= std::min(t, path2.size() - 1);
				// If the agent has no assignments, check against its initial location
				auto cfg2						= (!path2.empty()) ? path2[step2] :
																m_library->GetMPProblem()->GetInitialCfg(robot2);
																//robot2->GetSimulationModel()->GetState();
				//auto cfg2 = path2[step2];
				cfg2.SetRobot(robot2);
				auto multibody2 		= robot2->GetMultiBody();
				multibody2->Configure(cfg2);

				// Check for collitision. If none, move on.
				if(!sg->m_discrete) {
					CDInfo cdInfo;
					const bool collision = vc->IsMultiBodyCollision(cdInfo,
						multibody1, multibody2, "MotionValidation");
					if(collision)
						conflictCount++;
				}
				else {
					const bool collision = (cfg1[0] == cfg2[0]
											and cfg1[1] == cfg2[1]
											and cfg1[2] == cfg2[2]);
					if(collision)
						conflictCount++;
				}
			}
		}
	}

	return conflictCount;
}

