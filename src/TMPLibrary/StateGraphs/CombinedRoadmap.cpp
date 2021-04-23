#include "CombinedRoadmap.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/ChildAgent.h"


#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/TMPTools/InteractionTemplate.h"

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

/*------------------------------ Construction -------------------------------*/

void
CombinedRoadmap::
Initialize(){
}

/*-------------------------------- Accessors --------------------------------*/

    
size_t 
CombinedRoadmap::
AddConfiguration(SemanticRoadmap* _sr, GroupCfg _cfg) {
  return 0;
}
    
std::set<size_t> 
CombinedRoadmap::
AddInteraction(CompositeSemanticRoadmap _csr, InteractionTemplate* _it) {
  return {};
}

    
const Hypergraph<CombinedRoadmap::TMPVertex,CombinedRoadmap::TMPHyperarc>* 
CombinedRoadmap::
GetHypergraph() const {
  return m_hypergraph.get();
}

    
const std::set<CombinedRoadmap::SemanticRoadmap>& 
CombinedRoadmap::
GetSemanticRoadmaps() const {
  return m_semanticRoadmaps;
}

    
const MPSolution* 
CombinedRoadmap::
GetMPSolution() const {
  return m_mpSolution.get();
}
/*------------------------------ Helper Functions ----------------------------*/




