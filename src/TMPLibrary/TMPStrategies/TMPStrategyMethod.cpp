#include "TMPStrategyMethod.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/PoIPlacementMethods/PoIPlacementMethod.h"
#include "TMPLibrary/StateGraphs/Helpers/ITConnector.h"
#include "TMPLibrary/StateGraphs/StateGraph.h"
#include "TMPLibrary/TaskPlan.h"

#include "Simulator/Simulation.h"


/*****************************************Constructor*****************************************************/
TMPStrategyMethod::
TMPStrategyMethod(XMLNode& _node){
  m_sgLabel = _node.Read("sgLabel", true, "",
												 "Label for the state graph used by the TMPStrategy");
}

TMPStrategyMethod::
~TMPStrategyMethod(){
}

/******************************************Configure*****************************************************/

void
TMPStrategyMethod::
Initialize(){
	this->GetTaskPlan()->Initialize();
	this->GetStateGraph(m_sgLabel)->Initialize();
}

/****************************************Call Method*****************************************************/
void
TMPStrategyMethod::
operator()(){
	Initialize();	
	PlanTasks();
	this->GetStateGraph(m_sgLabel)->LoadStateGraph();
	DecomposeTasks();
	AssignTasks();
}

/*****************************************Accessors******************************************************/

/**************************************Combined Roadmap**************************************************/

void
TMPStrategyMethod::
PlanTasks(){}

void
TMPStrategyMethod::
AssignTasks(){}

void
TMPStrategyMethod::
DecomposeTasks(){}
