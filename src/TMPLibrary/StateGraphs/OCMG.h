#ifndef PPL_OCMG_H_
#define PPL_OCMG_H_

#include "TMPLibrary/StateGraphs/StateGraph.h"


#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "TMPLibrary/ActionSpace/Condition.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/Interaction.h"

class OCMG : public StateGraph {

  public:

    ///@name Local Types
    ///@{

    typedef GroupLocalPlan<Cfg>                                  GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType>            GroupRoadmapType;
    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>     MPSolution;

    typedef Condition::State State;

    typedef std::unordered_map<std::string,Robot*> RoleMap;

    ///@} 
    ///@name Construction
    ///@{

    OCMG();

    OCMG(XMLNode& _node);

    virtual ~OCMG() = default;

    ///@}
    ///@name Initialization
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{

    GroupRoadmapType* GetGroupRoadmap(RobotGroup* _group);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    void ConstructObjectRoadmap(Robot* _object);

    void ConstructRobotRoadmap(Robot* _robot);

    void CopyRoadmap(GroupRoadmapType* _rm, Robot* _passive, FormationCondition* _condition);

    void SampleInteractions();

    bool SampleInteraction(Interaction* _interaction, State _state);

    bool SampleInteractionWithPassive(Interaction* _interaction, State _state, 
                                      RobotGroup* _passive);

    bool RunInteractionStrategy(Interaction* _interaction, State _start);

    void BuildIndividualObjectModeGraph();

    ///@}
    ///@name Internal State
    ///@{

    std::string m_roadmapStrategy; ///< MPStrategy to build individual roadmaps with.

    std::string m_querySampler;

    size_t m_maxQueryAttempts;

    size_t m_interactionAttempts;

    size_t m_interactionSamples;

    std::string m_connector;

    std::unique_ptr<MPSolution> m_solution;

    std::set<RobotGroup*> m_groups;

    ///@}
};

#endif
