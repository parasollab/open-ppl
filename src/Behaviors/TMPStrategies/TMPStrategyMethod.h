#ifndef TMP_STRATEGY_METHOD_H
#define TMP_STRATEGY_METHOD_H

#include "Behaviors/Agents/HandoffAgent.h"
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
    ///@name
    ///@{

    /// Get plan for the input agents to perform the input tasks.
    /// _library needs to have the solution and problem set to the coordinator's
    /// values for these.
    virtual TaskPlan PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
                               vector<std::shared_ptr<MPTask>> _tasks, Robot* _superRobot,
                               std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>*
                                                               _ITPlacementMethods = nullptr) override;

    /// Removes all of the capability roadmaps currently stored if replanning is
    /// needed. TODO::LazyQuery may remove the need for this.
    void ResetCapabilityRoadmaps();

    ///@}

  private:

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

    void FindITLocations(InteractionTemplate* _it, MPLibrary* _library, Robot* _superRobot,
                         std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>*
                                                         _ITPlacementMethods);

    void TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg);

    void GenerateDummyAgents(std::vector<HandoffAgent*> _agents);

    void ConnectDistinctRoadmaps(vector<size_t> _roadmap1, vector<size_t> _roadmap2,
                                 HandoffAgent* _agent, Robot* _superRobot, MPLibrary* _library);
    ///@}
    ///@name Member Variables
    ///@{


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

    //TODO:: Figure out if I need to keep track of robot and task start and end
    //points

    /// Determines if capability roadmaps include ITs
    bool m_useITs{true};

    bool m_debug{false};
    ///@}
};

#endif
