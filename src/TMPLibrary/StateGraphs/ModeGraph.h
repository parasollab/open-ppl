#ifndef PPL_MODE_GRAPH_H_
#define PPL_MODE_GRAPH_H_

#include "StateGraph.h"

#include "ConfigurationSpace/Formation.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

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
    typedef GroupLocalPlan<Cfg>                                  GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType>            GroupRoadmapType;
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

    };

    typedef Hypergraph<GroundedVertex,Transition>           GroundedHypergraph;
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

    GroundedHypergraph& GetGroundedHypergraph();

    GroupRoadmapType* GetGroupRoadmap(RobotGroup* _group);

    MPSolution* GetMPSolution();
  
    ///@}

  private:

    ///@name Helper Functions
    ///@{

    std::vector<VID> AddStartState(const State& _start);

    void GenerateModeHypergraph(const std::vector<VID>& _initialModes);

    void SampleNonActuatedCfgs(const State& _start,std::set<VID>& _startVIDs,std::set<VID>& _goalVIDs);

    void SampleTransitions();

    void GenerateRoadmaps(const State& _start,std::set<VID>& _startVIDs,std::set<VID>& _goalVIDs);

    void ConnectTransitions();

    void ApplyAction(Action* _action, std::set<std::vector<VID>>& _applied,
                     std::vector<VID>& _newModes);

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

    std::set<VID> AddStateToGroundedHypergraph(const State& _state, std::unordered_map<RobotGroup*,Mode*> _modeMap);

    ///@}
    ///@name Internal State
    ///@{

    // Main set of representations

    ModeHypergraph m_modeHypergraph;

    GroundedHypergraph m_groundedHypergraph;

    std::unique_ptr<MPSolution> m_solution;

    // Additional info

    std::vector<unique_ptr<Mode>> m_modes;

    std::unordered_map<VID,std::unordered_set<VID>> m_modeGroundedVertices;

    std::set<size_t> m_unactuatedModes;

    // XML Parameters

    std::string m_unactuatedSM;

    std::string m_querySM;

    std::string m_expansionStrategy;

    std::string m_queryStrategy;

    size_t m_numUnactuatedSamples;

    size_t m_numInteractionSamples;

    size_t m_maxAttempts;


    ///@}

};

#endif
