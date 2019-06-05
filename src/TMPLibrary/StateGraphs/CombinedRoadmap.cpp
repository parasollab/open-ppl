#include "CombinedRoadmap.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/TaskPlan.h"

/*------------------------------ Construction --------------------------------*/

CombinedRoadmap::
CombinedRoadmap(XMLNode& _node) : StateGraph(_node) {
}

/*------------------------------ Construction --------------------------------*/

void
CombinedRoadmap::
Initialize() {

	StateGraph::Initialize();
}

/*------------------------------ Accessors --------------------------------*/


/*------------------------------ Helpers --------------------------------*/

void
CombinedRoadmap::
ConstructGraph(){
	std::cout << "Creating combined roadmap and robot-type roadmaps." << std::endl;
}
