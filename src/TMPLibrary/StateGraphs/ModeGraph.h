#ifndef PPL_MODE_GRAPH_H_
#define PPL_MODE_GRAPH_H_

#include "StateGraph.h"

#include "ConfigurationSpace/Formation.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "TMPLibrary/ActionSpace/Action.h"

#include "Traits/CfgTraits.h"

#include "Utilities/Hypergraph.h"

#include <memory>
#include <unordered_map>
#include <unordered_set>

class Interaction;

class ModeGraph : public StateGraph {

  public:
    
    ///@name Local Types 
    ///@{

    struct Mode {

      RobotGroup* robotGroup;
      std::unordered_set<Formation*> formations;
      std::vector<std::unique_ptr<Constraint>> constraints;

    };

    typedef std::pair<Action*,bool>                              ReversibleAction;
    typedef Hypergraph<Mode*,ReversibleAction>                   ModeHypergraph;
    typedef size_t                                               VID;
    typedef TMPBaseObject::GroupCfgType                          GroupCfgType;
    typedef TMPBaseObject::GroupLocalPlanType                    GroupLocalPlanType;
    typedef TMPBaseObject::GroupRoadmapType                      GroupRoadmapType;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>>          GroupPathType;
    typedef PathType<MPTraits<Cfg,DefaultWeight<Cfg>>>           Path;
    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>     MPSolution;
    typedef std::pair<GroupRoadmapType*,VID>                     GroundedVertex;
    typedef std::vector<std::vector<std::shared_ptr<GroupTask>>> TransitionTaskSet;

    struct Transition {

      //std::unordered_map<Robot*,Path*> explicitPaths;
      std::unordered_map<Robot*,std::vector<Cfg>> explicitPaths;
      std::unordered_map<Robot*,std::pair<double,std::pair<VID,VID>>> implicitPaths;
      TransitionTaskSet taskSet;
      std::unordered_map<GroupTask*,std::unordered_set<Formation*>> taskFormations;
      double cost;

      bool operator==(const Transition& _other) const {
        return explicitPaths  == _other.explicitPaths
           and implicitPaths  == _other.implicitPaths
           and taskSet        == _other.taskSet
           and taskFormations == _other.taskFormations
           and cost           == _other.cost;
      }

      bool operator!=(const Transition& _other) const {
        return !(*this == _other);
      }

    };

    typedef Hypergraph<GroundedVertex,Transition>           GroundedHypergraphLocal;
    typedef Action::State                                   State;


    ///@}
    ///@name Construction
    ///@{

    ModeGraph();

    ModeGraph(XMLNode& _node);

    virtual ~ModeGraph();

    ///@}
    ///@name Interface
    ///@{

    void Initialize() override;

    void GenerateRepresentation(const State& _start);

    ///@}
    ///@name Accessors
    ///@{

    ModeHypergraph& GetModeHypergraph();

    GroundedHypergraphLocal& GetGroundedHypergraphLocal();

    GroupRoadmapType* GetGroupRoadmap(RobotGroup* _group);

    //MPSolution* GetMPSolution();
  
    VID GetModeOfGroundedVID(const VID& _vid) const;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    std::vector<VID> AddStartState(const State& _start);

    void GenerateModeHypergraph(const std::vector<VID>& _initialModes);

    void SampleNonActuatedCfgs(const State& _start,std::set<VID>& _startVIDs,std::set<VID>& _goalVIDs);

    void ConfigureGoalSets(const size_t& _sink, std::set<VID>& _goalVIDs);

    std::vector<std::set<SemanticTask*>> BuildTaskSets(std::set<SemanticTask*> _taskSet, size_t _index, 
                                                 const std::vector<std::set<SemanticTask*>>& _buckets);

    void SampleTransitions();

    void GenerateRoadmaps(const State& _start,std::set<VID>& _startVIDs,std::set<VID>& _goalVIDs);

    void ConnectTransitions();

    void ApplyAction(Action* _action, std::set<std::vector<VID>>& _applied,
                     std::vector<VID>& _newModes, bool _forward=true);

    std::vector<std::vector<ModeGraph::VID>> CollectModeSets(
               const std::vector<std::vector<VID>>& _formationModes, 
               size_t _index, 
               const std::vector<VID>& _partialSet);


    std::vector<std::vector<Mode*>> CollectModeSetCombinations(
                        const std::vector<std::vector<std::vector<Robot*>>>& _possibleAssignments,
                        size_t _index, std::vector<Mode*> _partial, 
                        const std::set<Robot*>& _used);


    std::vector<Mode*> CollectModeCombinations(const std::vector<std::vector<Robot*>>& _possibleModeAssignments,
                        size_t _index, const std::vector<Robot*> _partial,
                        const std::set<Robot*>& _used);

    void SaveInteractionPaths(Interaction* _interaction, State& _start, State& _end,
                                 std::unordered_map<RobotGroup*,Mode*> _startModeMap,
                                 std::unordered_map<RobotGroup*,Mode*> _endModeMap);

    VID AddMode(Mode* _mode);

    std::set<VID> AddStateToGroundedHypergraphLocal(const State& _state, std::unordered_map<RobotGroup*,Mode*> _modeMap);

    bool CanReach(const State& _state);

    bool ContainsSolution(std::set<VID>& _startVIDs);

    ///@}
    ///@name Internal State
    ///@{

    // Main set of representations

    ModeHypergraph m_modeHypergraph;

    GroundedHypergraphLocal m_groundedHypergraph;

    //std::unique_ptr<MPSolution> m_solution;

    // Additional info

    std::vector<unique_ptr<Mode>> m_modes;

    std::unordered_map<VID,std::unordered_set<VID>> m_modeGroundedVertices;

    std::map<size_t,size_t> m_groundedInstanceTracker;

    std::set<size_t> m_unactuatedModes;

    // XML Parameters

    std::string m_unactuatedSM;

    std::string m_querySM;

    std::string m_queryStrategy;

    std::string m_queryStrategyStatic;

    std::string m_expansionStrategy;

    size_t m_numUnactuatedSamples;

    size_t m_numInteractionSamples;

    size_t m_maxAttempts;

    std::set<size_t> m_entryVertices;
    std::set<size_t> m_exitVertices;

    std::unordered_map<SemanticTask*,size_t> m_goalVertexTaskMap;

    bool m_writeHypergraphs{false};

    // TEMP TO GET SEPARATE GROUNDED HYPERGRAPH
    std::string m_GH;

    ///@}

};

std::ostream& operator<<(std::ostream& _os, const ModeGraph::Mode* _mode);
std::istream& operator>>(std::istream& _is, const ModeGraph::Mode* _mode);

std::ostream& operator<<(std::ostream& _os, const ModeGraph::ReversibleAction _ra);
std::istream& operator>>(std::istream& _is, const ModeGraph::ReversibleAction _ra);

//std::ostream& operator<<(std::ostream& _os, const ModeGraph::GroundedVertex _vertex);
//std::istream& operator>>(std::istream& _is, const ModeGraph::GroundedVertex _vertex);

std::ostream& operator<<(std::ostream& _os, const ModeGraph::Transition _t);
std::istream& operator>>(std::istream& _is, const ModeGraph::Transition _t);
#endif
