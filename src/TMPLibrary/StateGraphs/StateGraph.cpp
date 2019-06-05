#include "StateGraph.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/TaskPlan.h"

/*------------------------------ Construction --------------------------------*/

StateGraph::
StateGraph(XMLNode& _node) : TMPBaseObject(_node) {
}

/*------------------------------ Construction --------------------------------*/

void
StateGraph::
Initialize() {
	if(m_graph){
		delete m_graph;
	}
	m_graph = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(this->GetTaskPlan()->GetCoordinator()->GetRobot());

	ConstructGraph();
}

/*------------------------------ Accessors --------------------------------*/

RoadmapGraph<Cfg, DefaultWeight<Cfg>>*
StateGraph::
GetGraph(){
	return m_graph;
}

/*------------------------------ Helpers --------------------------------*/

void
StateGraph::
ConstructGraph(){

}
