#ifndef TMP_STRATEGY_METHOD_H
#define TMP_STRATEGY_METHOD_H

#include "Behaviors/Agents/HandoffAgent.h"
#include "Behaviors/Agents/WholeTask.h"
#include "Behaviors/TMPStrategies/ITPlacement/PlacementMethod.h"
#include "Behaviors/TMPStrategies/TaskPlan.h"
#include "MPProblem/MPTask.h"

class TMPStrategyMethod {
  public:

    typedef RoadmapGraph<CfgType, WeightType>         GraphType;
    typedef typename GraphType::vertex_descriptor     VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

    ///@name Constructon
    ///@{

    TMPStrategyMethod() = default;

    TMPStrategyMethod(XMLNode& _node);

    virtual ~TMPStrategyMethod() = default;

    ///@}
    ///@name Call Method
    ///@{

    /// Get plan for the input agents to perform the input tasks.
    /// _library needs to have the solution and problem set to the coordinator's
    /// values for these.
    virtual TaskPlan PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
                               vector<std::shared_ptr<MPTask>> _tasks, Robot* _superRobot,
                               std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>*
                                                               _ITPlacementMethods = nullptr);

    /// Removes all of the capability roadmaps currently stored if replanning is
    /// needed. TODO::LazyQuery may remove the need for this.
    void ResetCapabilityRoadmaps();

    ///@}

  protected:

    ///@name
    ///@{



    /// Generate roadmaps for each capability present in the agent set.
    void GenerateCapabilityRoadmaps(MPLibrary* _library, vector<HandoffAgent*> _agents,
                                    vector<std::shared_ptr<MPTask>> _tasks, Robot* _superRobot,
                                    std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>*
                                                                    _ITPlacementMethods);

    /// Copy the generated capability roadmaps into each agent of the
    /// capability.
    void CopyCapabilityRoadmaps(std::vector<HandoffAgent*> _agents, Robot* _superRobot);

    void GenerateInteractionTemplates(MPLibrary* _library,
                                      vector<HandoffAgent*> _agents,
                                      Robot* _superRobot);


    void PlaceInteractionTemplates(MPLibrary* _library, Robot* _superRobot,
                                   std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>*
                                   _ITPlacementMethods);

    void FindITLocations(InteractionTemplate* _it, Robot* _superRobot);

    void TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg);

    void ConnectDistinctRoadmaps(vector<size_t> _roadmap1, vector<size_t> _roadmap2,
                                 HandoffAgent* _agent, Robot* _superRobot, MPLibrary* _library);

    /// Randomly transforms the handoff template and attempts to validate it in
    /// the environment
    void TranslateHandoffTemplates(Robot* _superRobot);

    /// Initiallizes configurations for each capability at the start and end
    /// constriants of each whole task and adds them to the megaRoadmap
    void SetupWholeTasks(Robot* _superRobot);

    void CreateCombinedRoadmap(Robot* _superRobot);

    void Initialize(Robot* _superRobot);

    /// Makes sure all of the agents are ready to execute the plans
    void InitializeAgents();

    /// Generates the dummy agents of each capabiltiy used for planning
    void GenerateDummyAgents();

    void GenerateHandoffTemplates(Robot* _superRobot);

    /// Calls it placement methods listed in XML
    void FindITLocations(InteractionTemplate* _it);


    ///@}
    ///@name Member Variables
    ///@{

    MPLibrary* m_library{nullptr};   ///< The shared-roadmap planning library.

    MPSolution* m_solution{nullptr}; ///< The shared-roadmap solution.

    std::unique_ptr<Environment> m_handoffEnvironment;    ///< The handoff template environment.

    /// Map from each capability to an agent of that capability.
    std::unordered_map<std::string, HandoffAgent*> m_dummyAgentMap;

    /// Map from each capability to the roadmap for that capability.
    std::unordered_map<std::string, GraphType*> m_capabilityRoadmaps;

    /// The combined roadmap of all heterogenous robots and handoffs.
    GraphType* m_megaRoadmap{nullptr};

    /// The VIDs of all individual agent roadmaps in each transformed handoff template.
    std::vector<std::vector<size_t>> m_transformedRoadmaps;

    /// The VIDs of the start and end points of the whole tasks in the
    /// megaRoadmap
    std::vector<std::vector<size_t>> m_wholeTaskStartEndPoints;

    /// Maps agent capabilities to a dummy agent used for planning.
    std::unordered_map<std::string, HandoffAgent*> m_dummyMap;
    
	std::vector<HandoffAgent*> m_memberAgents;       ///< All robots in the group.

    /// The list of WholeTasks, which need to be divided into subtasks
    std::vector<WholeTask*> m_wholeTasks;
    
	/// Map of IT Placement Method Options
    std::unordered_map<std::string, std::unique_ptr<PlacementMethod>> m_ITPlacementMethods;


    //TODO:: Figure out if I need to keep track of robot and task start and end
    //points

    /// Determines if capability roadmaps include ITs
    bool m_useITs{true};

    bool m_debug{false};

	std::string m_dmLabel;
    
	double m_connectionThreshold{1.5};
    ///@}
};

#endif
