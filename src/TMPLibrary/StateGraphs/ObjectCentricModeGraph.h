#ifndef PPL_OBJECT_CENTRIC_MODE_GRAPH_H_
#define PPL_OBJECT_CENTRIC_MODE_GRAPH_H_

#include "StateGraph.h"

#include "ConfigurationSpace/Formation.h"
#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPProblem/Environment/Environment.h"

#include "TMPLibrary/ActionSpace/Interaction.h"

#include <map>
#include <unordered_map>

class ObjectCentricModeGraph : public StateGraph {

  public: 

    ///@name LocalTypes
    ///@{

    typedef Condition::State                          State;
    typedef GroupLocalPlan<Cfg>                       GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType> GroupRoadmapType;

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

    /// Key is the robot pointer for the object.
    /// Value is either the robot holding the object or the stable 
    /// placement region it is in.
    typedef std::map<Robot*,ModeInfo> ObjectMode;

    /// Key is the robot pointer for the robot or object involved.
    /// Value is the Interaction (and reverse statues) the key is involved in and the role (string)
    /// it plays in the interaction.
    //typedef std::map<Robot*,std::pair<std::pair<Interaction*,bool>,std::string>> ObjectModeSwitch;
    typedef std::unordered_map<std::string,Robot*> RoleMap;
    typedef std::unordered_map<Interaction*,std::vector<std::pair<bool,RoleMap>>> ObjectModeSwitch;

    typedef GenericStateGraph<ObjectMode,ObjectModeSwitch> GraphType;
    typedef GraphType::VID                                 VID;
  
    ///@}
    ///@name Construction
    ///@{

    ObjectCentricModeGraph();

    ObjectCentricModeGraph(XMLNode& _node);

    ~ObjectCentricModeGraph();

    ///@}
    ///@name Interface
    ///@{

    void Initialize() override;

    void GenerateRepresentation(const State& _start);

    ///@}
    ///@name Accessors
    ///@{

    GraphType* GetObjectModeGraph();

    MPSolution* GetMPSolution();

    GroupRoadmapType* GetGroupRoadmap(RobotGroup* _group);

    ///@}
    ///@name Debug
    ///@{

    void Print();

    ///@{

  private:

    ///@name Helper Functions
    ///@{

    void BuildRoadmaps();

    ObjectMode GenerateInitialMode(const State& _start);

    void BuildModeGraph(ObjectMode& _initialMode);

    std::vector<std::vector<std::pair<Robot*,std::string>>> GetAllApplications(
                Interaction* _interaction, VID _source, bool _reverse = false);

    void ExpandApplications(
                   const std::vector<std::vector<std::pair<Robot*,std::string>>>& _roleCombos, 
                   std::vector<std::pair<ObjectModeSwitch,std::set<Robot*>>>& _outgoing, 
                   Interaction* _interaction, bool _reverse = false);

    void ApplyEdge(ObjectModeSwitch _edge, VID _source, std::set<VID>& _newModes, 
                   const std::set<Robot*>& _used);


    ///@}
    ///@name Internal State
    ///@{

    GraphType m_graph;
 
    std::unique_ptr<MPSolution> m_solution;

    std::unordered_map<const Terrain*, size_t> m_capacities;

    std::vector<Robot*> m_robots;

    std::unordered_map<RobotGroup*,std::set<Formation*>> m_groups;

    std::string m_mpStrategy;
 
    ///@}

};

std::ostream& operator<<(std::ostream& _os, const ObjectCentricModeGraph::ObjectMode);
std::istream& operator>>(std::istream& _is, const ObjectCentricModeGraph::ObjectMode);

#endif
