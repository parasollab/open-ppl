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

    typedef std::map<Interaction*,std::vector<std::pair<State,State>>> SavedInteractions;

    typedef std::map<const Terrain*,std::map<GroupRoadmapType*,std::set<size_t>>> TerrainVIDs;

    struct ModeInfo {
      Robot* robot;
      Formation* formation;
      const Terrain* terrain;

      ModeInfo(Robot* _robot = nullptr, Formation* _formation = nullptr, 
                      const Terrain* _terrain = nullptr) :
        robot(_robot), formation(_formation), terrain(_terrain) {}

      bool operator==(const ModeInfo& _other) const {
        return robot     == _other.robot 
           and ((!formation and !_other.formation) or formation == _other.formation or *formation == *_other.formation)
           and terrain   == _other.terrain;
      }
    };

    typedef GenericStateGraph<ModeInfo,double> SingleObjectModeGraph;

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

    SingleObjectModeGraph* GetSingleObjectModeGraph();

    const SavedInteractions& GetSavedInteractions();

    std::vector<RobotGroup*> GetRobotGroups();

    std::vector<Robot*> GetRobots();

    std::vector<Robot*> GetObjects();

    TerrainVIDs GetTerrainVIDs();

    std::pair<State,State> GetSingleObjectModeGraphEdgeTransitions(
                            size_t _source, size_t _target, Robot* _object);

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

    State CopyAndConnectState(State _state);

    void BuildIndividualObjectModeGraph();

    void MapTerrainVIDs();

    bool IsReachable(const Terrain* _terrain, Robot* _robot);

    bool IsReachable(Robot* _robot1, Robot* _robot2);

    void SaveEdgeTransitions(size_t _source, size_t _target);

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

    std::set<Robot*> m_robots;

    std::set<Robot*> m_objects;

    std::set<RobotGroup*> m_groups;

    SavedInteractions m_savedInteractions;

    TerrainVIDs m_terrainVIDs;

    std::unique_ptr<SingleObjectModeGraph> m_omg;

    std::map<Robot*,std::map<std::pair<size_t,size_t>,
            std::pair<State,State>>> m_omgEdgeTransitions;

    ///@}
};

std::ostream& operator<<(std::ostream& _os, const OCMG::ModeInfo);
std::istream& operator>>(std::istream& _is, const OCMG::ModeInfo);

#endif
