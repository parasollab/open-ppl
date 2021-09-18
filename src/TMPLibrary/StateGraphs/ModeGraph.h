#ifndef PPL_MODE_GRAPH_H_
#define PPL_MODE_GRAPH_H_

#include "StateGraph.h"

#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "TMPLibrary/ActionSpace/Action.h"

#include "Traits/CfgTraits.h"

#include "Utilities/Hypergraph.h"

#include <unordered_map>
#include <unordered_set>

class Interaction;

class ModeGraph : public StateGraph {

  public:
    
    ///@name Local Types 
    ///@{

    typedef RobotGroup                                       Mode;
    typedef Hypergraph<Mode*,Action*>                        ModeHypergraph;
    typedef size_t                                           VID;
    typedef GroupLocalPlan<Cfg>                              GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType>        GroupRoadmapType;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>>      GroupPathType;
    typedef PathType<MPTraits<Cfg,DefaultWeight<Cfg>>>       Path;
    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;
    typedef std::pair<GroupRoadmapType*,VID>                 GroundedVertex;

    struct Transition {
      std::unordered_map<Robot*,Path*> explicitPaths;
      std::unordered_map<Robot*,std::pair<VID,VID>> implicitPaths;
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

    void GenerateRepresentation(const State& _start);

    ///@}
    ///@name Accessors
    ///@{

    ModeHypergraph& GetModeHypergraph();

    GroundedHypergraph& GetGroundedHypergraph();

    GroupRoadmapType* GetGroupRoadmap(RobotGroup* _group);

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    void GenerateModeHypergraph(const State& _start);

    void SampleTransitions();

    void GenerateRoadmaps(const State& _start);

    void ConnectTransitions();

    std::vector<VID> ApplyAction(Action* _action, 
                    std::set<std::set<VID>>& _applied);

    std::vector<std::set<ModeGraph::VID>> CollectModeSets(
               const std::vector<std::vector<VID>>& _formationModes, 
               size_t _index, 
               const std::set<VID>& _partialSet);


    std::vector<std::vector<ModeGraph::Mode*>> CollectModeSetCombinations(
                        const std::vector<std::vector<std::vector<Robot*>>>& _possibleAssignments,
                        size_t _index, std::vector<Mode*> _partial, 
                        const std::set<Robot*>& _used);


    std::vector<ModeGraph::Mode*> CollectModeCombinations(const std::vector<std::vector<Robot*>>& _possibleModeAssignments,
                        size_t _index, const std::vector<Robot*> _partial,
                        const std::set<Robot*>& _used);

    void SaveInteractionPaths(Interaction* _interaction, State& _start, State& _end);

    std::set<VID> AddStateToGroundedHypergraph(const State& _state);

    ///@}
    ///@name Internal State
    ///@{

    // Main set of representations

    ModeHypergraph m_modeHypergraph;

    GroundedHypergraph m_groundedHypergraph;

    std::unique_ptr<MPSolution> m_solution;

    // Additional info

    std::unordered_map<VID,std::unordered_set<VID>> m_modeGroundedVertices;

    // XML Parameters

    std::string m_expansionStrategy;

    std::string m_queryStrategy;

    size_t m_numSamples;

    size_t m_maxAttempts;

    ///@}

};

#endif