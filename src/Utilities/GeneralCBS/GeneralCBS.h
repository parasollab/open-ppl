#ifndef GENERAL_CBS_H_
#define GENERAL_CBS_H_

#include "Behaviors/Agents/Agent.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"


struct Assignment {
	Agent* 													m_agent{nullptr};
	std::shared_ptr<SemanticTask>		m_task;
	std::vector<size_t>							m_setupPath;
	std::vector<size_t>							m_execPath;
	double													m_setupStartTime;
	double													m_execStartTime;
	double													m_execEndTime;
	size_t													m_setupWaitTimeSteps;
	size_t													m_finalWaitTimeSteps;

	Assignment() {}

	Assignment(Agent* _agent, std::shared_ptr<SemanticTask> _task, 
							std::vector<size_t> _setup, std::vector<size_t> _exec,
							double _setupStart, double _startTime, double _endTime,
							size_t _setupWaitTimeSteps = 0, size_t _waitTimeSteps = 0) :
							m_agent(_agent), m_task(_task), m_setupPath(_setup), m_execPath(_exec),
							m_setupStartTime(_setupStart), m_execStartTime(_startTime), m_execEndTime(_endTime), 
							m_setupWaitTimeSteps(_setupWaitTimeSteps), m_finalWaitTimeSteps(_waitTimeSteps) {}

	bool operator>(const Assignment& _other) const {
		//return m_setupStartTime > _other.m_setupStartTime;
		return m_execStartTime > _other.m_execStartTime;
	}
	bool operator<(const Assignment& _other) const {
		//return m_setupStartTime < _other.m_setupStartTime;
		return m_execStartTime < _other.m_execStartTime;
	}

	bool operator==(const Assignment& _other) const {
		return (m_agent 					== _other.m_agent and
						m_task  					== _other.m_task and
						m_setupPath 			== _other.m_setupPath and
						m_execPath  			== _other.m_execPath and
						m_setupStartTime 	== _other.m_setupStartTime and
						m_execStartTime 	== _other.m_execStartTime and 
						m_execEndTime 		== _other.m_execEndTime and
						m_finalWaitTimeSteps == _other.m_finalWaitTimeSteps and
						m_setupWaitTimeSteps == _other.m_setupWaitTimeSteps);
	}

	bool operator!=(const Assignment& _other) const {
		return !((*this) == _other);
	}

};


struct CBSSolution {

	Decomposition* 																														m_decomposition;
	std::unordered_map<std::shared_ptr<SemanticTask>,Agent*>									m_allocationMap;
	std::unordered_map<Agent*,std::vector<Assignment>>												m_agentAssignments;
	std::unordered_map<std::shared_ptr<SemanticTask>,std::vector<Assignment>> m_taskPlans;

};


struct MotionConstraint {
	Agent* 													m_agent;
	Cfg															m_conflictCfg;
	size_t													m_timestep{MAX_INT};
	std::shared_ptr<SemanticTask>		m_task{nullptr};
	size_t 													m_duration{0};

	bool operator==(const MotionConstraint& _c) {
		return m_conflictCfg 	== _c.m_conflictCfg 
				and m_timestep 		== _c.m_timestep
				and m_task 				== _c.m_task
				and m_duration		== _c.m_duration;
	}
};


struct AllocationConstraint {
	Agent* 													m_agent{nullptr};
	Cfg															m_startLocation;
	Cfg 														m_endLocation;
	double													m_startTime{MAX_DBL};
	double													m_endTime{MAX_DBL};
	std::shared_ptr<SemanticTask>		m_task{nullptr};

	AllocationConstraint() {}

	AllocationConstraint(Agent* _agent, Cfg _startL, Cfg _endL, double _startT=MAX_DBL, 
												double _endT=MAX_DBL, std::shared_ptr<SemanticTask> _task = nullptr) :
												m_agent(_agent), m_startLocation(_startL), m_endLocation(_endL),
												m_startTime(_startT), m_endTime(_endT), m_task(_task) {}

	bool operator==(const AllocationConstraint& _c) {
		return m_startLocation == _c.m_startLocation
				and m_endLocation == _c.m_endLocation
				and m_startTime == _c.m_startTime 
				and m_endTime == _c.m_endTime
				and m_task == _c.m_task;
	}
};


class GeneralCBSNode {
  public:
	///@name Local Types
	///@{

	typedef std::unordered_map<std::shared_ptr<SemanticTask>, std::unordered_map<Agent*, std::vector<MotionConstraint>>> MotionConstraints;
	typedef std::unordered_map<std::shared_ptr<SemanticTask>, std::unordered_map<Agent*, std::vector<AllocationConstraint>>> AllocationConstraints;

	///@}
	///@name Constructors
	///@{

	GeneralCBSNode() = default;

	GeneralCBSNode(const CBSSolution _solution);

	GeneralCBSNode(const CBSSolution _solution, const CBSSolution _postAssignment);

	GeneralCBSNode(const GeneralCBSNode& _parent);

	~GeneralCBSNode() = default;

	///@name Operators
	///@{
	
	bool operator>(const GeneralCBSNode& _other) const noexcept;
	bool operator<(const GeneralCBSNode& _other) const noexcept;

	///@}
	//@name Accessors
	///@{
	
	CBSSolution GetSolution() const;
	
	//TODO::Find a different way to do this - only use it for one purpose rn
	CBSSolution& GetSolutionRef();

	void AddMotionConstraint(MotionConstraint& _c, Agent* _agent);

	const std::vector<MotionConstraint>& GetMotionConstraints(
														std::shared_ptr<SemanticTask> _task, Agent* _agent);

	void AddAllocationConstraint(AllocationConstraint& _c, Agent* _agent);

	const std::vector<AllocationConstraint>& GetAllocationConstraints(
														std::shared_ptr<SemanticTask> _task, Agent* _agent);

	void UpdateTaskPlan(std::shared_ptr<SemanticTask> _task, std::vector<Assignment> _assignments);

	void Debug();

	void SetCost(double _cost);

	double GetCost();

	CBSSolution& GetPostAssignmentRef();
	///@}

  private:
	///@name Interal State
	///@{

	CBSSolution m_solution;
	double m_cost{MAX_DBL};
	MotionConstraints m_motionConstraints;
	AllocationConstraints m_allocationConstraints;

	CBSSolution m_postAssignment;

	///@}

};

using GeneralCBSTree = std::priority_queue<GeneralCBSNode,std::vector<GeneralCBSNode>,std::greater<GeneralCBSNode>>;

using InitialPlanFunction = std::function<bool(Decomposition* _decomposition, GeneralCBSTree& _tree)>;

using ValidationFunction = std::function<bool(GeneralCBSNode& _node, GeneralCBSTree& _tree)>;

//using LowLevelSearch = std::function<void(GeneralCBSNode& _node)>;

CBSSolution
ConflictBasedSearch(Decomposition* _decomposition, InitialPlanFunction _initial, 
										ValidationFunction _validation, size_t _numIterations = MAX_INT, bool _debug=false);

#endif
