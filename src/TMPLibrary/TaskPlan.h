#ifndef TASK_PLAN_H_
#define TASK_PLAN_H_

#include <list>
#include <unordered_map>


#include "MPProblem/MPTask.h"

#include "TMPLibrary/TMPTools/InteractionTemplate.h"
#include "TMPLibrary/WholeTask.h"

#include "ConfigurationSpace/Cfg.h"

class Coordinator;
class HandoffAgent;

class OccupiedInterval{
  public:
    OccupiedInterval(Robot* _r, Cfg _sL, Cfg _eL, double _sT, double _eT);

    Robot* GetRobot();
    Cfg GetStartLocation();
    Cfg GetEndLocation();
    double GetStartTime();
    double GetEndTime();
    std::pair<Cfg, double> GetStart();
    std::pair<Cfg, double> GetEnd();

    void SetStartTime(double _start);
    void SetEndTime(double _end);
    void SetStartLocation(Cfg _start);
    void SetEndLocation(Cfg _end);
    void SetStart(Cfg _startLoc, double _startTime);
    void SetEnd(Cfg _endLoc, double _endTime);

    bool CheckTimeOverlap(OccupiedInterval _interval);
    static void MergeIntervals(std::list<OccupiedInterval>& _intervals);
    bool operator<(OccupiedInterval _interval);


  private:
    Robot* m_robot;
    Cfg m_startLocation;
    Cfg m_endLocation;
    double m_startTime;
    double m_endTime;


};


class TaskPlan {

  public:

    typedef typename std::unordered_map<Agent*,std::list<std::shared_ptr<MPTask>>> AgentTaskMap;
    typedef typename std::unordered_map<std::shared_ptr<MPTask>,
                                std::list<std::shared_ptr<MPTask>>>        TaskDependencyMap;

    ///@name Construction
    ///@{

    TaskPlan() = default;

    ~TaskPlan();

		void Initialize();

    ///@}
    ///@name Accessors
    ///@{

    /// Get the tasks assigned to the input agent.
    std::list<std::shared_ptr<MPTask>> GetAgentTasks(Agent* _agent);

    /// Get the tasks that need to be completed before the input task can be
		/// completed.
		std::list<std::shared_ptr<MPTask>> GetTaskDependencies(std::shared_ptr<MPTask> _task);

		void AddSubtask(HandoffAgent* _agent, std::shared_ptr<MPTask> _task);

		void AddSubtask(HandoffAgent* _agent, std::shared_ptr<MPTask> _task, WholeTask* _wholeTask);

		/// first must occur before second
		void AddDependency(std::shared_ptr<MPTask> _first, std::shared_ptr<MPTask> _second);

		void RemoveLastDependency(std::shared_ptr<MPTask> _task);

		///@}
    ///@name RAT Functions
    ///@{

    void InitializeRAT();

		std::unordered_map<std::string,std::pair<Cfg,double>>& GetRAT();

		std::pair<Cfg,double> GetRobotAvailability(HandoffAgent* _agent);

		void UpdateRAT(HandoffAgent* _agent, std::pair<Cfg,double> _avail);

		///@}
		///@name WholeTask interactions
    ///@{

		/// Adds a whole task to the set of wholeTasks
		void AddWholeTask(WholeTask* _wholeTask);

    /// Gets the whole set of wholeTasks
    std::vector<WholeTask*>& GetWholeTasks();

    /// Generates the whole task object for each input task
    void CreateWholeTasks(std::vector<std::shared_ptr<MPTask>> _tasks);

		/// Sets the wholeTask owner of the subtask in the subtask map
		void SetWholeTaskOwner(std::shared_ptr<MPTask> _subtask, WholeTask* _wholeTask);

		/// Returns the owning wholeTask of the input subtask
    WholeTask* GetWholeTask(std::shared_ptr<MPTask> _subtask);

    std::shared_ptr<MPTask> GetNextSubtask(WholeTask* _wholeTask);

    void AddSubtaskToWholeTask(std::shared_ptr<MPTask> _subtask, WholeTask* _wholeTask);

    /// Returns the agent that as assiged the prior subtask in the whole task
    /// @param _wholeTask The WholeTask object containing the subtasks in
    /// question
    /// @return The HandoffAgent assigned to the preceeding subtask
    HandoffAgent* GetLastAgent(WholeTask* _wholeTask);

    ///@}
    ///@name Agent Accessors
    ///@{

		void SetCoordinator(Coordinator* _c);

		Coordinator* GetCoordinator();

		void LoadTeam(std::vector<HandoffAgent*> _team);

		/// Returns the set of all the agents available for the task
		std::vector<HandoffAgent*>& GetTeam();

		/// Generates the dummy agents of each capabiltiy used for planning
    void GenerateDummyAgents();

		HandoffAgent* GetCapabilityAgent(std::string _robotType);

		std::unordered_map<std::string,HandoffAgent*>& GetDummyMap();

        ///@}
    ///@name Interaction Templates
    ///@{
    /// @todo Document what these are. Does it include one copy of each abstract
    ///       IT, or each transformed instance (or something else)? If the
    ///       information is duplicated in the roadmaps, we should annotate the
    ///       roadmap cfgs instead to avoid duplicating data (which we then have
    ///       to keep synchronized).

    std::vector<std::shared_ptr<InteractionTemplate>>& GetInteractionTemplates();

    void AddInteractionTemplate(InteractionTemplate*);



    ///@}

  private:

		///Cordinator for the task plan.//TODO::make a distributed option where this is a leader
		Coordinator* m_coordinator{nullptr};

    /// The list of WholeTasks, which need to be divided into subtasks
    std::vector<WholeTask*> m_wholeTasks;

    /// Map subtasks to the WholeTask that they are included in to access the
    /// next subtask.
    std::unordered_map<std::shared_ptr<MPTask>, WholeTask*> m_subtaskMap;

    /// Robot Availablility Table keeps track of where/when each robot will finish
    /// its last assigned subtask and be available again
    std::unordered_map<std::string,std::pair<Cfg,double>> m_RAT;

    /// TODO::Adapt to robot groups later
    /// Maps each agent to the set of tasks assigned to it.
    AgentTaskMap m_agentTaskMap;

    /// TODO:: Adapt to task groups later
    /// Maps each task to the tasks that must be completed before it.
    TaskDependencyMap m_taskDependencyMap;

		/// Maps agent capabilities to a dummy agent used for planning.
    std::unordered_map<std::string, HandoffAgent*> m_dummyMap;

		/// All robots available for the task..
		std::vector<HandoffAgent*> m_memberAgents;

		/// The set of Interaction Templates
    std::vector<std::shared_ptr<InteractionTemplate>> m_interactionTemplates;

};

#endif
