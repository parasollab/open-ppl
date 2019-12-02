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
MotionValidation(MPLibrary* _library, LowLevelSearch* _lowLevel, TMPLibrary* _tmpLibrary, 
								std::string _sgLabel, std::string _vcLabel) :
								Validation(_library,_lowLevel), m_tmpLibrary(_tmpLibrary), 
								m_sgLabel(_sgLabel), m_vcLabel(_vcLabel) {}	

MotionValidation::
~MotionValidation() {}

bool
MotionValidation::
InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) {
	return true;
}

bool 
MotionValidation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {

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
					std::pair<size_t,size_t>,std::shared_ptr<SemanticTask>>>> m_pathTaskMap;

	auto sg = static_cast<CombinedRoadmap*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

	for(auto kv : agentAssignments) {
		auto agent = kv.first;
		agentPaths[agent] = {};

		auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(agent));

		auto& assigns = kv.second;
		
		for(auto a : assigns ) {
			PathType<MPTraits<Cfg,DefaultWeight<Cfg>>> path(roadmap.get());
			//remove last element of setup as it is the same as the exec path
			auto setup = a.m_setupPath;
			setup.pop_back();
			path += setup;
			path += a.m_execPath;
			auto cfgs = path.FullCfgs(m_library);

			size_t start = agentPaths.size();
			size_t end = start+cfgs.size()-1;
			agentPaths[agent].insert(agentPaths[agent].end(),cfgs.begin(),cfgs.end());

			m_pathTaskMap[agent].push_back(std::make_pair(std::make_pair(start,end),a.m_task->GetParent()));
		}
	}

	//Find the last timestep a robot is moving.
	size_t lastTimestep = 0;
	for(auto& kv : agentPaths) {
		lastTimestep = std::max(lastTimestep,kv.second.size());
	}

	auto vc = static_cast<CollisionDetectionValidity<MPTraits<Cfg,DefaultWeight<Cfg>>>*>(
			m_library->GetValidityChecker(m_vcLabel).get());

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
																robot2->GetSimulationModel()->GetState();
				//auto cfg2 = path2[step2];
				cfg2.SetRobot(robot2);
				auto multibody2 		= robot2->GetMultiBody();
				multibody2->Configure(cfg2);

				// Check for collitision. If none, move on.
				CDInfo cdInfo;
				const bool collision = vc->IsMultiBodyCollision(cdInfo,
					multibody1, multibody2, "MotionValidation");
				if(!collision)
					continue;

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
				for(auto task : m_pathTaskMap[one.m_agent]) {
					if(t >= task.first.first and t <= task.first.second) {
						one.m_task = task.second;
					}
				}

				two.m_agent = robot2->GetAgent();
				two.m_conflictCfg = cfg1;
				two.m_timestep = t;
				for(auto task : m_pathTaskMap[two.m_agent]) {
					if(t >= task.first.first and t <= task.first.second) {
						two.m_task = task.second;
					}
				}
				return std::make_pair(one,two);
			}
		}
	}
	
	//Reset multibodies at initial location - not sure if this affects SimulatedState
	for(auto kv : agentPaths) {
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
		
	if(_constraints.first.m_task.get()) {
		GeneralCBSNode one(_node);
		one.AddMotionConstraint(_constraints.first,_constraints.first.m_agent);
		if(m_lowLevel->UpdateSolution(one, _constraints.first.m_task))
			_tree.push(one);
 	}

	if(_constraints.second.m_task.get()) {
		GeneralCBSNode two(_node);
		two.AddMotionConstraint(_constraints.second,_constraints.second.m_agent);
		if(m_lowLevel->UpdateSolution(two, _constraints.second.m_task))
			_tree.push(two);
	}
}
