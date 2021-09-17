#include "ModeGraph.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"

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

    for(auto& kv : m_modeHypergraph.GetHyperarcMap()) {

        auto& hyperarc = kv.second;
        auto interaction = dynamic_cast<Interaction*>(hyperarc.property);

        State modeSet;
        for(auto vid : hyperarc.tail) {
            Mode* mode = m_modeHypergraph.GetVertex(vid).property;
            modeSet[mode] = std::make_pair(nullptr,MAX_INT);
        }

        auto label = interaction->GetInteractionStrategyLabel();
        auto is = this->GetInteractionStrategyMethod(label);

        for(size_t i = 0; i < m_numSamples; i++) {

            for(size_t j = 0; j < m_maxAttempts; j++) {

                if(!is->operator()(interaction,modeSet))
                    continue;

                // TODO::Save interaction paths
                break;
            }
        }
    }

}

void
ModeGraph::
GenerateRoadmaps(const State& _start) {


  // If mode is initial mode, add starting vertex
  for(const auto& kv : _start) {
      auto mode = kv.first;
      auto grm = kv.second.first;
      auto vid = kv.second.second;
      auto gcfg = grm->GetVertex(vid);

      auto newGrm = m_solution->GetGroupRoadmap(mode);
      auto newGcfg = gcfg.SetGroupRoadmap(newGrm);
      newGrm->AddVertex(newGcfg);
  }

  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
 
  // For each mode in the mode hypergraph, run the expansion strategy
  for(auto kv : m_modeHypergraph.GetVertexMap()) {
      auto vertex = kv.second;
      auto mode = vertex.property;

      // Initialize dummy task
      auto task = new GroupTask(mode);

      for(auto r : mode->GetRobots()) {
        auto t = MPTask(r);
        task->AddTask(t);
      } 
 
      // Call the MPLibrary solve function to expand the roadmap
      lib->SetPreserveHooks(true);
      lib->Solve(prob,task,m_solution.get(),m_expansionStrategy, LRand(), 
             "ExpandModeRoadmap");
      lib->SetPreserveHooks(false);

      delete task;
    }
}

void
ModeGraph::
ConnectTransitions() {

    // For each mode in the mode hypergraph, attempt to connect transition samples
    for(auto kv1 : m_groundedHypergraph.GetVertexMap()) {
        auto vid1 = kv1.first;
        auto vertex1 = kv1.second;
        auto mode = vertex1.property.first;

        // Create start constraint from vertex

        for(auto kv2 : m_groundedHypergraph.GetVertexMap()) {
            auto vid2 = kv2.first;

            // Make sure vertices are unique
            if(vid1 == vid2)
                continue;

            // Make sure vertices of the same mode
            auto vertex2 = kv2.second;
            if(mode != vertex2.property.first)
                continue;


            // Create task out of task vertices
            // GroupTask* task(mode);
        }
    }
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