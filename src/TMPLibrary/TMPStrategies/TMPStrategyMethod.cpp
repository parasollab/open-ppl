#include "TMPStrategyMethod.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/PoIPlacementMethods/PoIPlacementMethod.h"
#include "TMPLibrary/StateGraphs/Helpers/ITConnector.h"
#include "TMPLibrary/TaskPlan.h"

#include "Simulator/Simulation.h"


/*****************************************Constructor*****************************************************/
TMPStrategyMethod::
TMPStrategyMethod(XMLNode& _node){
  m_useITs = _node.Read("useITs", false, m_useITs,
                        "Indicate if the TMP Strategy should use ITs when planning.");
  m_debug = _node.Read("debug", false, m_debug,
                       "Indicate if the TMP Strategy should output debug information.");
  m_dmLabel = _node.Read("dmLabel", true, "", "Distance metric for checking "
      "nearest agents and charging locations.");
  m_connectionThreshold = _node.Read("connectionThreshold",true,1.2, 0., 1000.,
      "Acceptable variabliltiy in IT paths.");
	m_placementMethod = _node.Read("poiPlacementMethod", false, "", 
			"Methods for placing points of interest in the environment");

  for(auto& child : _node){
		// Load the environment file used to create ITs
		if(child.Name() == "InteractionEnvironment"){
			if(!m_interactionEnvironment){
				m_interactionEnvironment = std::unique_ptr<Environment>(new Environment(child));
			}
		}
		//TODO::Figure out if this is going to be extracted or how to find a problem pointer
		//TODO::Temp fix may be to figure out where the problem pointe is needed and delay that functionality
		//else if(child.Name() == "PlacementMethod"){
		//	AddPlacementMethod(PlacementMethod::Factory(m_problem(), child));
		//}
	}

}
/*
TMPStrategyMethod::
TMPStrategyMethod(bool _useITs, bool _debug, std::string _dmLabel, double _connectionThreshold,
									Environment* _interactionEnvironment, 
									std::unordered_map<std::string, std::unique_ptr<PlacementMethod>>& _ITPlacementMethods) :
								  m_useITs(_useITs), 
									m_debug(_debug),
									m_dmLabel(_dmLabel), 
									m_connectionThreshold(_connectionThreshold){

	m_interactionEnvironment = std::unique_ptr<Environment>(_interactionEnvironment);
  for(auto& pm : _ITPlacementMethods){
		m_ITPlacementMethods[pm.first] = pm.second->Clone();
	}

}*/

TMPStrategyMethod::
~TMPStrategyMethod(){
	delete m_combinedRoadmap;
}

/******************************************Configure*****************************************************/

void
TMPStrategyMethod::
Initialize(){

}

void 
TMPStrategyMethod::
Initialize(Robot* _robot){
  m_initialized = true;

  m_robot = _robot;

  this->GetTaskPlan()->GetTeam().clear();

	//TODO::load team into task plan

  this->GetTaskPlan()->GenerateDummyAgents(); 
  
  ResetCapabilityRoadmaps();

  //m_wholeTasks.clear();
  //m_wholeTaskStartEndPoints.clear();

  if(m_combinedRoadmap){
		delete m_combinedRoadmap;
  }
	m_combinedRoadmap = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(m_robot);

  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));
	m_solution->SetRoadmap(m_robot,m_combinedRoadmap);

}


/****************************************Call Method*****************************************************/

TaskPlan*
TMPStrategyMethod::
PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
          vector<std::shared_ptr<MPTask>> _tasks){
  if(!m_initialized){
	throw RunTimeException(WHERE, "TMPStrategyMethod not initialized.");
  }

	this->GetMPLibrary()->SetMPSolution(m_solution.get());

  this->GetTaskPlan()->GetTeam() = _agents;

  this->GetTaskPlan()->CreateWholeTasks(_tasks);

  return new TaskPlan();
}

void
TMPStrategyMethod::
operator()(){
}

/*****************************************Accessors******************************************************/

Robot* 
TMPStrategyMethod::
GetRobot(){
	return m_robot;
}

/**************************************Combined Roadmap**************************************************/

TaskPlan*
TMPStrategyMethod::
AssignTasks(){
	return new TaskPlan();
}

void
TMPStrategyMethod::
DecomposeTasks(){}
