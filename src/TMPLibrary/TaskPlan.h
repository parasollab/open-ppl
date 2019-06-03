#ifndef TASK_PLAN_H_
#define TASK_PLAN_H_

#include <list>
#include <unordered_map>

#include "Behaviors/Agents/Agent.h"

#include "MPProblem/MPTask.h"

#include "TMPLibrary/WholeTask.h"

class TaskPlan {

  public:

    typedef typename std::unordered_map<Agent*,std::list<std::shared_ptr<MPTask>>> AgentTaskMap;
    typedef typename std::unordered_map<std::shared_ptr<MPTask>,
                                std::list<std::shared_ptr<MPTask>>>        TaskDependencyMap;

    ///@name Construction
    ///@{

    TaskPlan() = default;

    ~TaskPlan() = default;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the tasks assigned to the input agent.
    std::list<std::shared_ptr<MPTask>> GetAgentTasks(Agent* _agent);

    /// Get the tasks that need to be completed before the input task can be
    /// completed.
    std::list<std::shared_ptr<MPTask>> GetTaskDependencies(std::shared_ptr<MPTask> _task);

	void AddSubtask(Agent* _agent, std::shared_ptr<MPTask> _task);

	void AddDependency(std::shared_ptr<MPTask> _first, std::shared_ptr<MPTask> _second);

	void RemoveLastDependency(std::shared_ptr<MPTask> _task);

    ///@}
    ///@name WholeTask interactions
    ///@{

    /// Gets the whole set of wholeTasks
    std::vector<WholeTask*>& GetWholeTasks();

    /// Generates the whole task object for each input task
    void CreateWholeTasks(std::vector<std::shared_ptr<MPTask>> _tasks);

    /// Returns the owning wholeTask of the input subtask
    WholeTask* GetWholeTask(std::shared_ptr<MPTask> _subtask);

    std::shared_ptr<MPTask> GetNextSubtask(WholeTask* _wholeTask);

    void AddSubtasktoWholeTask(std::shared_ptr<MPTask> _subtask);

    /// Returns the agent that as assiged the prior subtask in the whole task
    /// @param _wholeTask The WholeTask object containing the subtasks in
    /// question
    /// @return The HandoffAgent assigned to the preceeding subtask
    HandoffAgent* GetLastAgent(WholeTask* _wholeTask);

    ///@}

  private:

    /// The list of WholeTasks, which need to be divided into subtasks
    std::vector<WholeTask*> m_wholeTasks;

    /// Map subtasks to the WholeTask that they are included in to access the
    /// next subtask.
    std::unordered_map<std::shared_ptr<MPTask>, WholeTask*> m_subtaskMap;

    /// Robot Availablility Table keeps track of where/when each robot will finish
    /// its last assigned subtask and be available again
    std::unordered_map<Agent*,std::pair<Cfg,double>> m_RAT; 

    /// TODO::Adapt to robot groups later
    /// Maps each agent to the set of tasks assigned to it.
    AgentTaskMap m_agentTaskMap;

    /// TODO:: Adapt to task groups later
    /// Maps each task to the tasks that must be completed before it.
    TaskDependencyMap m_taskDependencyMap;

};

#endif
