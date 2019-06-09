#include "MultiAgentMultiTaskPlanner.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/MPTask.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"

#include "Utilities/SSSP.h"

/*****************************************Constructor****************************************************/
MultiAgentMultiTaskPlanner::
MultiAgentMultiTaskPlanner(){
	this->SetName("MultiAgentMultiTaskPlanner");
}

MultiAgentMultiTaskPlanner::
MultiAgentMultiTaskPlanner(XMLNode& _node) : TMPStrategyMethod(_node){
	this->SetName("MultiAgentMultiTaskPlanner");
}

MultiAgentMultiTaskPlanner::
~MultiAgentMultiTaskPlanner(){}

void
MultiAgentMultiTaskPlanner::
Initialize(){
	TMPStrategyMethod::Initialize();
}

/******************************************Configure*****************************************************/



/*****************************************Call Method****************************************************/

void
MultiAgentMultiTaskPlanner::
PlanTasks(){
  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
	auto dummyTask = new MPTask(robot);	
	this->GetMPLibrary()->SetTask(dummyTask);
	
	auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());

	for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
		auto query = sg->AddTaskToGraph(wholeTask);

		TaskPlan* taskPlan = MAMTDijkstra(wholeTask,query.first,query.second);
		//*this->GetTaskPlan() = *taskPlan;
		m_taskPlans.push_back(taskPlan);
		sg->RemoveTaskFromGraph(wholeTask);
		m_nodeAgentMap.clear();
	}
	//TODO::Compress into a singular taskplan
}

/*****************************************TaskGraph Functions****************************************************/

TaskPlan*
MultiAgentMultiTaskPlanner::
MAMTDijkstra(WholeTask* _wholeTask, size_t _start, size_t _goal){
	m_nodeAgentMap[_start] = this->GetTaskPlan()->GetCoordinator();

	SSSPPathWeightFunction<TaskGraph> weight;
	weight = [this,_goal](typename TaskGraph::adj_edge_iterator& _ei,
                   const double _sourceDistance,
                   const double _targetDistance) {
            return this->MAMTPathWeight(_ei,_sourceDistance,_targetDistance,_goal);
        };

	//TODO::Actually call dijkstras
	
	SSSPTerminationCriterion<TaskGraph> termination(
		[_goal](typename TaskGraph::vertex_iterator& _vi,
						const SSSPOutput<TaskGraph>& _sssp) {
					return _goal == _vi->descriptor() ? SSSPTermination::EndSearch
																						: SSSPTermination::Continue;
			}
	);
	auto g = this->GetStateGraph(m_sgLabel)->GetGraph();
	const SSSPOutput<TaskGraph> sssp = DijkstraSSSP(g, {_start}, weight, termination);

	const size_t last = sssp.ordering.back();
	if(_goal != last){
		return new TaskPlan();	
	}	

	//Extract path
	std::vector<size_t> path;
	path.push_back(last);

	size_t current = last;
	do {
		current = sssp.parent.at(current);
		path.push_back(current);
	} while(current != _start);
	std::reverse(path.begin(), path.end());

	return ExtractTaskPlan(path,_wholeTask,sssp.distance);

}

/*****************************************Helper Functions****************************************************/

double
MultiAgentMultiTaskPlanner::
MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
	const double _sourceDistance, const double _targetDistance, size_t _goal) {
  
  const double edgeWeight  = _ei->property().GetWeight();
  double readyTime = 0;
  Agent* newAgent = nullptr;
  size_t source = _ei->source();
  size_t target = _ei->target();
  
  //Check if edge is virtual and if so find the next robot and keep track of current robot in a map in this class
  bool virt = (edgeWeight == -1);
  if(virt and target !=_goal){
		auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
  	readyTime = sg->RobotSelection(target,&newAgent,m_nodeRATCache[source]);
  }
  else{
  	newAgent = m_nodeAgentMap[source];
  }

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
		m_nodeRATCache[target] = m_nodeRATCache[source];
		Cfg location = this->GetStateGraph(m_sgLabel)->GetGraph()->GetVertex(source);
		location.SetRobot(m_nodeAgentMap[source]->GetRobot());
	  m_nodeRATCache[target][m_nodeAgentMap[source]] = std::make_pair(location,_sourceDistance);	
  }
  return newDistance;
}

TaskPlan*
MultiAgentMultiTaskPlanner::
ExtractTaskPlan(const std::vector<size_t>& _path, WholeTask* _wholeTask, 
								std::unordered_map<size_t,double> _distance){
	if(m_debug){
		std::cout << "Print path." << std::endl;
		for(auto vid : _path){
			std::string label;
			if(vid == _path.front() or vid == _path.back()){
				label = "Virtual";
			}
			else{
				label = m_nodeAgentMap.at(vid)->GetRobot()->GetLabel();
			}
			std::cout << vid << " : " << label << std::endl;
		}
	}
	auto g = this->GetStateGraph(m_sgLabel)->GetGraph();
	//TaskPlan* taskPlan = new TaskPlan();
	auto taskPlan = this->GetTaskPlan();
	//taskPlan->AddWholeTask(_wholeTask);
	Agent* currentAgent = nullptr;
	std::shared_ptr<MPTask> previousTask(nullptr);
	size_t first;
	size_t last;
	for(size_t i = 0; i < _path.size(); i++){
		auto vid = _path[i];
		if(vid == _path.front())
			continue;
		if(!currentAgent){ //should only be the case for the first node
			currentAgent = m_nodeAgentMap[vid];
			first = vid;
		}
		//need to keep all virtual nodes virtual or maybe not...
		// The virtual node is really just a copy of the real cfg
		else if(m_nodeAgentMap[vid] != currentAgent or vid == _path.back()){
			if(m_debug){
				std::cout << "New task: " << first << "->" << last << std::endl;
			}
			auto start = g->GetVertex(first);
			auto goal = g->GetVertex(last);
			auto task = CreateMPTask(currentAgent->GetRobot(), start, goal);
			taskPlan->AddSubtask(static_cast<HandoffAgent*>(currentAgent),task,_wholeTask);
			if(previousTask){
				taskPlan->AddDependency(task,previousTask);
			}
			previousTask = task;
			first = vid;
			if(vid == _path.back()){
				vid = _path[i-1];
			}
			goal.SetRobot(currentAgent->GetRobot());
			auto avail = std::pair<Cfg,double>(goal,_distance[vid]);//May need to set robot pointer for goal
			this->GetTaskPlan()->UpdateRAT(static_cast<HandoffAgent*>(currentAgent),avail);
			currentAgent = m_nodeAgentMap[vid];
		}
		else{
			last = vid;
		}
	}
	return new TaskPlan();;
}

std::shared_ptr<MPTask>
MultiAgentMultiTaskPlanner::
CreateMPTask(Robot* _robot, Cfg _start, Cfg _goal){
	std::shared_ptr<MPTask> task(new MPTask(_robot));
	_start.SetRobot(_robot);
	_goal.SetRobot(_robot);
	
	if(!_goal.GetRobot()->IsManipulator()){
    auto radius = (_robot->GetMultiBody()->GetBoundingSphereRadius());

    std::unique_ptr<CSpaceBoundingBox> boundingBox(
        new CSpaceBoundingBox({_start.GetPosition()[0],_start.GetPosition()[1],0}));

    boundingBox->SetRange(0,
                          (_start.GetPosition()[0]-radius),
                          (_start.GetPosition()[0]+radius));
    boundingBox->SetRange(1,
                          (_start.GetPosition()[1]-radius),
                          (_start.GetPosition()[1]+radius));
    boundingBox->SetRange(2,-1,1);

    auto startConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(_start.GetRobot(), std::move(boundingBox)));
    std::unique_ptr<CSpaceBoundingBox> boundingBox2(
        new CSpaceBoundingBox({_goal.GetPosition()[0],_goal.GetPosition()[1],0}));

    boundingBox2->SetRange(0,
                           (_goal.GetPosition()[0]-radius/2),
                           (_goal.GetPosition()[0]+radius/2));
    boundingBox2->SetRange(1,
                           (_goal.GetPosition()[1]-radius/2),
                           (_goal.GetPosition()[1]+radius/2));
    boundingBox2->SetRange(2,-1,1);

    auto goalConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(_goal.GetRobot(), std::move(boundingBox2)));

    task->SetStartConstraint(std::move(startConstraint));
    task->ClearGoalConstraints();
    task->AddGoalConstraint(std::move(goalConstraint));
	}
	else{
		throw RunTimeException(WHERE, "Manipulator code not yet written for MAMTP.");
	}
	return task;
}
