#ifndef TMP_STRATEGY_METHOD_H
#define TMP_STRATEGY_METHOD_H

#include "Behaviors/Agents/HandoffAgent.h"

#include "MPProblem/MPTask.h"

#include "TMPLibrary/TMPBaseObject.h"
#include "TMPLibrary/PoIPlacementMethods/ITPlacement/ITPlacementMethod.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"


class TMPStrategyMethod : public TMPBaseObject {
  public:

    typedef RoadmapGraph<CfgType, WeightType>         GraphType;
    typedef typename GraphType::vertex_descriptor     VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

    ///@name Constructon
    ///@{

    TMPStrategyMethod() = default;

    TMPStrategyMethod(XMLNode& _node);
 	
	TMPStrategyMethod(bool _useITs, bool _debug, std::string _dmLabel, double _connectionThreshold,
										Environment* _interactionEnvironment, 
										std::unordered_map<std::string, std::unique_ptr<ITPlacementMethod>>& _ITPlacementMethods);

    virtual ~TMPStrategyMethod();

		///@}
		///@name Interface
		///@{

		void operator()();

    ///@}
    ///@name Configure
    ///@{

		void Initialize() override;

    void Initialize(Robot* _superRobot);

    /// Removes all of the capability roadmaps currently stored if replanning is
    /// needed. 
    void ResetCapabilityRoadmaps();

		///@}
    ///@name Call Method
    ///@{

    /// Get plan for the input agents to perform the input tasks.
    /// _library needs to have the solution and problem set to the coordinator's
    /// values for these.
    virtual TaskPlan* PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
                                vector<std::shared_ptr<MPTask>> _tasks);

    ///@}
    ///@name Accessors
    ///@{

	Robot* GetRobot();

	HandoffAgent* GetCapabilityAgent(std::string _robotType);

    //Moved to TaskPlan
	//WholeTask* GetWholeTask(std::shared_ptr<MPTask> _subtask);

    ///@}

    //protected:

    ///@name Combined Roadmap Methods
    ///@{
	
    void CreateCombinedRoadmap();

    void GenerateITs();

    void FindITLocations(InteractionTemplate* _it);
    
		/// Transforms the ITs into the disovered locations
		void TransformITs();
    
		/// Initiallizes configurations for each capability at the start and end
    /// constriants of each whole task and adds them to the megaRoadmap
    void SetupWholeTasks();
		
    ///@}
    ///@name Task Assignment
    ///@{

	virtual TaskPlan* AssignTasks();

	virtual void DecomposeTasks();

		///@}
    ///@name Helper Methods
    ///@{

    /// Generates the dummy agents of each capabiltiy used for planning
    void GenerateDummyAgents();

		/// Generates the whole task object for each input task
		//Moved to TaskPlan
        //void CreateWholeTasks(std::vector<std::shared_ptr<MPTask>> _tasks);

		void AddPlacementMethod(std::unique_ptr<ITPlacementMethod> _pm);
	
		///@}
	

    ///@}
    ///@name Member Variables
    ///@{

		Robot* m_robot;

    MPLibrary* m_library{nullptr};   ///< The shared-roadmap planning library.

    std::unique_ptr<MPSolution> m_solution{nullptr}; ///< The shared-roadmap solution.

    std::unique_ptr<Environment> m_interactionEnvironment;    ///< The handoff template environment.

    /// Map from each capability to the roadmap for that capability.
    std::unordered_map<std::string, std::shared_ptr<GraphType>> m_capabilityRoadmaps;

    /// The combined roadmap of all heterogenous robots and handoffs.
    GraphType* m_combinedRoadmap{nullptr};

    /// The VIDs of all individual agent roadmaps in each transformed handoff template.
    std::vector<std::vector<size_t>> m_transformedRoadmaps;

    /// The VIDs of the start and end points of the whole tasks in the
    /// megaRoadmap
    std::vector<std::vector<size_t>> m_wholeTaskStartEndPoints;

    /// Maps agent capabilities to a dummy agent used for planning.
    std::unordered_map<std::string, HandoffAgent*> m_dummyMap;
    
		std::vector<HandoffAgent*> m_memberAgents;       ///< All robots in the group.

    /// The list of WholeTasks, which need to be divided into subtasks
    //Moved to TaskPlan
    //std::vector<WholeTask*> m_wholeTasks;
    
		/// Map of IT Placement Method Options
    //Moved to TMPLibrary
    //std::unordered_map<std::string, std::unique_ptr<PlacementMethod>> m_ITPlacementMethods;

		/// Map subtasks to the WholeTask that they are included in to access the
    /// next subtask.
    //Moved to TaskPlan
    //std::unordered_map<std::shared_ptr<MPTask>, WholeTask*> m_subtaskMap;

    //TODO:: Figure out if I need to keep track of robot and task start and end
    //points

    /// Determines if capability roadmaps include ITs
    bool m_useITs{true};

    bool m_debug{false};

		std::string m_dmLabel;
    
		double m_connectionThreshold{1.5};

	bool m_initialized{false};
    ///@}
};

#endif
