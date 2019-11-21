#ifndef GENERAL_CBS_H_
#define GENERAL_CBS_H_

#include "Behaviors/Agents/Agent.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"


struct Assignment {
	Agent* 													m_agent;
	std::shared_ptr<SemanticTask>		m_task;
	std::vector<size_t>							m_setupPath;
	std::vector<size_t>							m_execPath;
};


struct CBSSolution {

	Decomposition* 																						m_decomposition;
	std::unordered_map<std::shared_ptr<SemanticTask>,Agent*>	m_allocationMap;
	std::unordered_map<Agent*,std::shared_ptr<Assignment>>		m_agentAssignments;
	std::unordered_map<std::shared_ptr<SemanticTask>,std::vector<Assignment>> m_taskPlans;

};


struct MotionConstraint {
	Cfg															m_conflictCfg;
	double													m_time{MAX_DBL};
	std::shared_ptr<SemanticTask>		m_task{nullptr};
};


struct AllocationConstraint {
	Cfg															m_startLocation;
	Cfg 														m_endLocation;
	double													m_startTime{MAX_DBL};
	double													m_endTime{MAX_DBL};
	std::shared_ptr<SemanticTask>		m_task{nullptr};
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

	GeneralCBSNode(const GeneralCBSNode& _parent, const CBSSolution _solution);

	~GeneralCBSNode() = default;

	///@name Operators
	///@{
	
	bool operator>(const GeneralCBSNode& _other) const noexcept;

	///@}
	//@name Accessors
	///@{
	
	CBSSolution GetSolution() const;

	void AddMotionConstraint(MotionConstraint& _c, Agent* _agent);

	void AddAllocationConstraint(AllocationConstraint& _c, Agent* _agent);

	void UpdateTaskPlan(std::shared_ptr<SemanticTask> _task, std::vector<Assignment> _assignments);

	///@}

  private:
	///@name Interal State
	///@{

	CBSSolution m_solution;
	double m_cost{MAX_DBL};
	MotionConstraints m_motionConstraints;
	AllocationConstraints m_allocationConstraints;

	///@}

};

using GeneralCBSTree = std::priority_queue<GeneralCBSNode,std::vector<GeneralCBSNode>,std::greater<GeneralCBSNode>>;

using InitialPlanFunction = std::function<bool(Decomposition* _decomposition, GeneralCBSTree& _tree)>;

using ValidationFunction = std::function<bool(GeneralCBSNode& _node, GeneralCBSTree& _tree)>;

//using LowLevelSearch = std::function<void(GeneralCBSNode& _node)>;

CBSSolution
ConflictBasedSearch(Decomposition* _decomposition, InitialPlanFunction _initial, 
										ValidationFunction _validation, size_t _numIterations = MAX_INT);

#endif
