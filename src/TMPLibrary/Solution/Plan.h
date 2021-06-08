#ifndef _PMPL_PLAN_H_
#define _PMPL_PLAN_H_

#include <list>
#include <memory>
#include <vector>
#include <unordered_map>

class Coordinator;
class Decomposition;
class MPProblem;
class Robot;
class RobotGroup;
class SemanticTask;
class StatClass;
class TaskSolution;

class Plan {
  public:
    ///@name Construction
    ///@{

    Plan();

    ~Plan();

    ///@}
    ///@name Accessors
    ///@{

    /// Coordinator
    void SetCoordinator(Coordinator* _coordinator);

    Coordinator* GetCoordinator() const;

    /// Robot Team
    void SetTeam(std::vector<Robot*> _team);

    const std::vector<Robot*>& GetTeam() const;

    /// Decomposition
    void SetDecomposition(Decomposition* _decomp);

    Decomposition* GetDecomposition() const;

    /// Task Allocations
    void ClearAllocations(Robot* _robot);

    void ClearAllocations(RobotGroup* _group);

    void AddAllocation(Robot* _robot, SemanticTask* _task);

    void AddAllocation(RobotGroup* _group, SemanticTask* _task);

    std::list<SemanticTask*> GetAllocations(Robot* _robot);

    std::list<SemanticTask*> GetAllocations(RobotGroup* _group);

    const std::unordered_map<SemanticTask*,std::shared_ptr<TaskSolution>>& GetTaskSolutions();

    /// Task Plans
    void SetTaskSolution(SemanticTask* _task, std::shared_ptr<TaskSolution> _solution);

    TaskSolution* GetTaskSolution(SemanticTask* _task);

    std::vector<TaskSolution*> GetRobotTaskSolutions(Robot* _robot);

    /// MPProblem
    void SetMPProblem(MPProblem* _problem);

    MPProblem* GetMPProblem();

    /// StatClass
    StatClass* GetStatClass();

    ///@}
    ///@name Print
    ///@{

    void Print();

    ///@}

  private:
    ///@name Internal State
    ///@{

    Coordinator* m_coordinator{nullptr};

    std::vector<Robot*> m_team;

    Decomposition* m_decomposition{nullptr};

    std::unordered_map<Robot*,std::list<SemanticTask*>> m_allocations;

    std::unordered_map<RobotGroup*,std::list<SemanticTask*>> m_groupAllocations;

    std::unordered_map<SemanticTask*,std::shared_ptr<TaskSolution>> m_taskSolutions;

    std::unique_ptr<StatClass> m_statClass;

    MPProblem* m_problem{nullptr};

    ///@}

};

#endif
