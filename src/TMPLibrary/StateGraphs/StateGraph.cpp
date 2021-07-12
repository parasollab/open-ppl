#include "StateGraph.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Simulator/Simulation.h"
#include "TMPLibrary/Solution/Plan.h"

/*------------------------------ Construction --------------------------------*/

StateGraph::
StateGraph() {}

StateGraph::
StateGraph(XMLNode& _node) : TMPBaseObject(_node) {
  m_pmLabel = _node.Read("pmLabel",false,"",
                "The placement method used for points of interests (i.e. ITs)");
}

/*------------------------------ Construction --------------------------------*/

void
StateGraph::
Initialize() {
	if(m_graph){
		delete m_graph;
	}
	m_graph = new GenericStateGraph<Cfg, DefaultWeight<Cfg>>(this->GetPlan()->GetCoordinator()->GetRobot());

  ConstructGraph();
}

/*------------------------------ Accessors --------------------------------*/

GenericStateGraph<Cfg, DefaultWeight<Cfg>>*
StateGraph::
GetGraph() {
  return m_graph;
}

void
StateGraph::
LoadStateGraph(){
	this->GetPlan()->GetCoordinator()->SetGenericStateGraph(m_graph);
}

/*------------------------------ Helpers --------------------------------*/

void
StateGraph::
ConstructGraph() {
}
