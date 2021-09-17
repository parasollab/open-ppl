#include "ModeGraph.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"

#include <set>

/*------------------------------ Construction --------------------------------*/

ModeGraph::
ModeGraph() {
    this->SetName("ModeGraph");
}

ModeGraph::
ModeGraph(XMLNode& _node) : StateGraph(_node) {
    this->SetName("ModeGraph");

    m_expansionStrategy = _node.Read("expansionStrategy",true,"",
                        "MPStrategy label to build initial roadaps.");
    m_queryStrategy = _node.Read("queryStrategy",true,"",
                        "MPStrategy label to query roadaps.");
    m_numSamples = _node.Read("numSamples",false,1,1,1000,
                        "The number of samples to generate for each transtion.");
    m_maxAttempts = _node.Read("maxAttemptz",false,1,1,1000,
                        "The max number of attempts to generate a sample.");
}

ModeGraph::
~ModeGraph() {}

/*-------------------------------- Interface ---------------------------------*/

void
ModeGraph::
GenerateRepresentation(const State& _start) {

    GenerateModeHypergraph(_start);
    SampleTransitions();
    GenerateRoadmaps(_start);
    ConnectTransitions();

}

/*-------------------------------- Accessors ---------------------------------*/

ModeGraph::ModeHypergraph&
ModeGraph::
GetModeHypergraph() {
    return m_modeHypergraph;
}

ModeGraph::GroundedHypergraph&
ModeGraph::
GetGroundedHypergraph() {
    return m_groundedHypergraph;
}

ModeGraph::GroupRoadmapType*
ModeGraph::
GetGroupRoadmap(RobotGroup* _group) {
    return m_solution->GetGroupRoadmap(_group);
}

/*---------------------------- Helper Functions ------------------------------*/

void
ModeGraph::
GenerateModeHypergraph(const State& _start) {

    // Extract initial submodes
    std::set<Mode*> newModes;
    for(auto kv : _start) {
        auto mode = kv.first;
        newModes.insert(mode);
    }

    auto as = this->GetTMPLibrary()->GetActionSpace();

    // Keep track of already expanded mode/action combinations
    std::unordered_map<Action*,std::set<std::set<VID>>> appliedActions;

    do {
      
      // Add new modes to the hyerpgraph
      for(auto mode : newModes) {
        auto vid = m_modeHypergraph.AddVertex(mode);
        m_modeGroundedVertices[vid] = std::unordered_set<VID>();
      }

      // Apply actions to discovered modes to make new modes
      for(auto actionLabel : as->GetActions()) {
        auto action = actionLabel.second;
        newModes = ApplyAction(action,appliedActions[action]);
      }

    } while(!newModes.empty());
}

void
ModeGraph::
SampleTransitions() {

    // For each edge in the mode graph, generate n samples

    // Apply interaction strategy to interaction
}

void
ModeGraph::
GenerateRoadmaps(const State& _start) {

    // For each mode in the mode hypergraph, run the expansion strategy

    // If mode is initial mode, add starting vertex
}

void
ModeGraph::
ConnectTransitions() {

    // For each mode in the mode hypergraph, attempt to connect transition samples
}

std::set<ModeGraph::Mode*>
ModeGraph::
ApplyAction(Action* _action, std::set<std::set<VID>>& _applied) {

    // Extract the formation constraint

    // Look at possible combinations of modes in the mode hyerpgraph 
    // that satisfy the formation constraint

    // Apply the action and generate new modes for each valid combination
    // not already in _applied
    std::set<Mode*> newModes;

    // Return the set of new modes
    return newModes;
}

/*----------------------------------------------------------------------------*/