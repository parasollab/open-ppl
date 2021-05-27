#include "CombinedRoadmap.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/ChildAgent.h"

#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/Solution/Plan.h"

#include "Utilities/MetricUtils.h"


/*------------------------------ Construction --------------------------------*/
CombinedRoadmap::
CombinedRoadmap(){
	this->SetName("CombinedRoadmap");
};

CombinedRoadmap::
CombinedRoadmap(XMLNode& _node) : StateGraph(_node) {
	this->SetName("CombinedRoadmap");
}

CombinedRoadmap::
~CombinedRoadmap() {
  for(auto sm : m_semanticRoadmaps) {
    delete sm;
  } 
}
/*------------------------------ Construction -------------------------------*/

void
CombinedRoadmap::
Initialize(){
  auto prob = this->GetMPProblem();

  // Intialize hypergraph with source and sink
  m_hypergraph = std::unique_ptr<TMPHypergraph>(new TMPHypergraph());
  TMPVertex source, sink;
  auto sourceVID = m_hypergraph->AddVertex(source);
  auto sinkVID = m_hypergraph->AddVertex(sink);

  std::set<size_t> sourceTailSet;

  if(m_debug) {
    std::cout << "Source VID: " << sourceVID << std::endl;
    std::cout << "Sink VID: " << sinkVID << std::endl;
  }

  // Initialize mpsolution from coordinator
  auto c = this->GetPlan()->GetCoordinator();
  m_mpSolution = std::unique_ptr<MPSolution>(new MPSolution(c->GetRobot()));

  for(auto& group : prob->GetRobotGroups()) { 
    // Add individial robots to mp solution
    for(auto& r : group->GetRobots()) {
      m_mpSolution->AddRobot(r);
    }

    // Create new group roadmaps in mp solution
    //auto grm = new GroupRoadmapType(group.get(), m_mpSolution.get());
    m_mpSolution->AddRobotGroup(group.get());
    auto grm = m_mpSolution->GetGroupRoadmap(group.get());

    // Create new semantic roadmap from group roadmap
    auto sm = new SemanticRoadmap(std::make_pair(grm,ActionUpdate()));
    m_semanticRoadmaps.insert(sm);

    // Create initial group vertex
    auto gcfg = GroupCfg(grm);

    // Add initial cfg to individual roadmaps
    for(auto& r : group->GetRobots()) {
      auto rm = m_mpSolution->GetRoadmap(r);
      auto cfg = prob->GetInitialCfg(r);
      auto vid = rm->AddVertex(cfg);
     
      // Update group vertex 
      gcfg.SetRobotCfg(r,vid);
    }

    // Add intial group vertex and connect to virtual source
    auto rvid = grm->AddVertex(gcfg);

    TMPVertex vertex(rvid,sm);

    auto hvid = m_hypergraph->AddVertex(vertex);
    if(m_debug) {
      std::cout << "Hypergraph vid: " << hvid << std::endl;
    }

    sourceTailSet.insert(hvid);
  }

  m_hypergraph->AddHyperarc({sourceVID},sourceTailSet,TMPHyperarc());
}

/*-------------------------------- Accessors --------------------------------*/

    
size_t 
CombinedRoadmap::
AddConfiguration(SemanticRoadmap* _sr, GroupCfg _cfg) {
  return 0;
}
    
std::set<size_t> 
CombinedRoadmap::
AddInteraction(CompositeSemanticRoadmap _csr, Interaction* _inter) {

  return {};
}

    
const Hypergraph<CombinedRoadmap::TMPVertex,CombinedRoadmap::TMPHyperarc>* 
CombinedRoadmap::
GetHypergraph() const {
  return m_hypergraph.get();
}

    
const std::set<CombinedRoadmap::SemanticRoadmap*>& 
CombinedRoadmap::
GetSemanticRoadmaps() const {
  return m_semanticRoadmaps;
}

    
MPSolution* 
CombinedRoadmap::
GetMPSolution() const {
  return m_mpSolution.get();
}
    
void
CombinedRoadmap::
AddRobotGroup(RobotGroup* _group) {
  m_mpSolution->AddRobotGroup(_group);
}

/*------------------------------ Helper Functions ----------------------------*/
/*----------------------------------------------------------------------------*/




