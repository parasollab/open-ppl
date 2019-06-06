#include "MultiAgentMultiTaskPlanner.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"

/*****************************************Constructor****************************************************/
MultiAgentMultiTaskPlanner::
MultiAgentMultiTaskPlanner(){
	this->SetName("MultiAgentMultiTaskPlanner");
}

MultiAgentMultiTaskPlanner::
MultiAgentMultiTaskPlanner(XMLNode& _node) : TMPStrategyMethod(_node){
	this->SetName("MultiAgentMultiTaskPlanner");
}

/******************************************Configure*****************************************************/



/*****************************************Call Method****************************************************/

void
MultiAgentMultiTaskPlanner::
PlanTasks(){
  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();	
	this->GetMPLibrary()->GetTask()->SetRobot(robot);
	
	auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());

	for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
		sg->AddTaskToGraph(wholeTask);
		TaskPlan* taskPlan = new TaskPlan();

		//Find Task Plan

		m_taskPlans.push_back(taskPlan);
		sg->RemoveTaskFromGraph(wholeTask);
	}
	//TODO::Compress into a singular taskplan
}

/*****************************************TaskGraph Functions****************************************************/

TaskPlan*
MultiAgentMultiTaskPlanner::
MAMTDijkstra(WholeTask* _wholeTask){
	SSSPPathWeightFunction<TaskGraph> weight;
	weight = [this](typename TaskGraph::adj_edge_iterator& _ei,
                   const double _sourceDistance,
                   const double _targetDistance) {
            return this->MAMTPathWeight(_ei,_sourceDistance,_targetDistance);
        };

	//TODO::Actually call dijkstras
  return new TaskPlan();	
}

/*****************************************Helper Functions****************************************************/

double
MultiAgentMultiTaskPlanner::
MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
	const double _sourceDistance, const double _targetDistance) {
  
  const double edgeWeight  = _ei->property().GetWeight();
  double readyTime = 0;
  HandoffAgent* newAgent = nullptr;
  size_t source = _ei->source();
  size_t target = _ei->target();
  
  //Check if edge is virtual and if so find the next robot and keep track of current robot in a map in this class
  bool virt = (edgeWeight == -1);
  if(virt){
		auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
  	readyTime = sg->RobotSelection(target,&newAgent);
  }
  else{
  	newAgent = m_nodeAgentMap[source];
  }

  //TODO::Then do the rest of the regular dijkstra stuff
  // Compute the new 'distance', which is the number of timesteps at which
  // the robot would reach the target node.
  double newDistance;
  if(virt){
  	//Check if the robot will need extra time to reach the location
  	if(readyTime > _sourceDistance){
  		newDistance = readyTime;
  	}
  	//Or if it can be there when the previous robot reaches the interaction point
  	else{
  		newDistance = _sourceDistance;
  	}
  }
  else{
  	newDistance = _sourceDistance + edgeWeight;
  }

  // If this edge is better than the previous we update the robot at the node
  if(newDistance < _targetDistance) {
  	m_nodeAgentMap[target] = newAgent;
  }
  return newDistance;
}
