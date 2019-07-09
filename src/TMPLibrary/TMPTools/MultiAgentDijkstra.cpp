#include "MultiAgentDijkstra.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"

#include "Utilities/PMPLExceptions.h"

MultiAgentDijkstra::
MultiAgentDijkstra() : TMPBaseObject() {
	this->SetName("MultiAgentDijkstra");	
}

MultiAgentDijkstra::
MultiAgentDijkstra(XMLNode& _node) : TMPBaseObject (_node) {
	this->SetName("MultiAgentDijkstra");
	m_sgLabel = _node.Read("sgLabel", true, "",
												 "Label for the state graph used by MultiAgentDijkstra.");	
}

MultiAgentDijkstra::
~MultiAgentDijkstra(){};

bool
MultiAgentDijkstra::
Run(WholeTask* _wholeTask, TaskPlan* _plan){
	if(!_wholeTask){
		throw RunTimeException(WHERE, "MultiAgentDijkstra needs a wholeTask to solve.");
	}
	TaskPlan* savedPlan = nullptr;
	if(_plan){
		savedPlan = this->GetTaskPlan();
		this->GetTMPLibrary()->SetTaskPlan(_plan);
	}

	auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
	auto query = sg->AddTaskToGraph(_wholeTask);

	auto start = query.first;
	auto goal = query.second;

	m_nodeAgentMap[start] = this->GetTaskPlan()->GetCoordinator();

	//Search backwards from goal to start
	SSSPPathWeightFunction<TaskGraph> weight;
	weight = [this,start](typename TaskGraph::adj_edge_iterator& _ei,
                   const double _sourceDistance,
                   const double _targetDistance) {
            return this->MAMTPathWeight(_ei,_sourceDistance,_targetDistance,start);
        };

	//TODO::Actually call dijkstras
	
	SSSPTerminationCriterion<TaskGraph> termination(
		[start](typename TaskGraph::vertex_iterator& _vi,
						const SSSPOutput<TaskGraph>& _sssp) {
					return start == _vi->descriptor() ? SSSPTermination::EndSearch
																						: SSSPTermination::Continue;
			}
	);
	auto g = this->GetStateGraph(m_sgLabel)->GetGraph();
	const SSSPOutput<TaskGraph> sssp = DijkstraSSSP(g, {goal}, weight, termination);

	const size_t last = sssp.ordering.back();
	if(start != last){
		return new TaskPlan();	
	}	

	//Extract path
	std::vector<size_t> path;
	path.push_back(last);

	size_t current = last;
	do {
		current = sssp.parent.at(current);
		path.push_back(current);
	} while(current != goal);
	//std::reverse(path.begin(), path.end());

	ExtractTaskPlan(path,_wholeTask,sssp.distance);
	sg->RemoveTaskFromGraph(_wholeTask);
	m_nodeAgentMap.clear();
	m_nodeRATCache.clear();
	m_parentMap.clear();
	m_incomingEdgeMap.clear();
	if(savedPlan)
		this->GetTMPLibrary()->SetTaskPlan(savedPlan);
	return true;
}

void
MultiAgentDijkstra::
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
			//TODO::Update to occupied interval logic
			//auto avail = std::pair<Cfg,double>(goal,_distance[vid]);//May need to set robot pointer for goal
			//this->GetTaskPlan()->UpdateRAT(static_cast<HandoffAgent*>(currentAgent),avail);
			currentAgent = m_nodeAgentMap[vid];
		}
		else{
			last = vid;
		}
	}
	//Update RAT
	for(auto avail : m_nodeRATCache[_path.back()]){
		for(auto interval : avail.second){
			this->GetTaskPlan()->UpdateRAT(static_cast<HandoffAgent*>(avail.first),interval);
		}
	}
}

std::shared_ptr<MPTask>
MultiAgentDijkstra::
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

double
MultiAgentDijkstra::
MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
	const double _sourceDistance, const double _targetDistance, size_t _goal) {
  
  const double edgeWeight  = _ei->property().GetWeight();
  double readyTime = 0;
  Agent* newAgent = nullptr;
  size_t source = _ei->source();
  size_t target = _ei->target();
  
  //Check if edge is virtual and if so find the next robot and keep track of current robot in a map in this class
  bool virt = (edgeWeight == -1);
	size_t parent = -1;
	double incomingEdge = -1;
	if(virt){
		if(m_parentMap.empty())//virtual goal node in backwards search
			return 0;
		// TODO::figure out if these checks are actually needed-may be the case that these always exist given 
		// its a virtual node
		//if(std::find(m_parentMap.begin(),m_parentMap.end(),source) != m_parentMap.end()) {
			parent = m_parentMap[source];
			incomingEdge = m_incomingEdgeMap[source];
		//}
	}
  if(virt and target !=_goal){
		auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
		readyTime = sg->RobotSelection(source,target,&newAgent,m_nodeRATCache[source],
																	 parent, incomingEdge);
  }
  else{
  	newAgent = m_nodeAgentMap[source];
  }

  // Compute the new 'distance', which is the number of timesteps at which
  // the robot would reach the target node.
  double newDistance;
  if(virt){
  	/*//This is the old forward implementation
		//Check if the robot will need extra time to reach the location
  	if(readyTime > _sourceDistance){
  		newDistance = readyTime;
  	}
  	//Or if it can be there when the previous robot reaches the interaction point
  	else{
  		newDistance = _sourceDistance;
  	}
		*/
		// receiving < delivering
		// The agent on the delivering end of the previously evaluated interaction can arrive before the 
		// just discovered agent can deliver the task to the IT.
		// We over compensated the readyTime of the previous agent and need to subtract it. 
		// This does not ruin the dijkstra search because only robots of the specified type can use the edge,
		// and this is still the first robot that can get there.  
		if(_sourceDistance < readyTime) {
			double oldReadyTime = 0;
			// If the subtask is dirent from start to goal, there is no oldReadyTime to account for.
			auto it = m_incomingEdgeMap.find(parent);
			if(it	!= m_incomingEdgeMap.end())	{
				oldReadyTime = m_incomingEdgeMap[parent];
			}	
			newDistance = _sourceDistance - oldReadyTime + readyTime;
		}
		// receiving > delivering
		// The agent on the receiving end of the previously evaluated interaction arrives after the 
		// just discovered agent delivers the task.
		// We have already accounted for the readyTime of the just discovered agent in the readyTime of the
		// previous agent.
		else {
			newDistance = _sourceDistance - readyTime;
		}
  }
  else{
  	newDistance = _sourceDistance + edgeWeight;
  }

  // If this edge is better than the previous we update the robot at the node
  if(newDistance < _targetDistance) {
  	m_nodeAgentMap[target] = newAgent;
		if(virt){
			m_nodeAgentMap[source] = newAgent;
			if(!m_nodeAgentMap[parent])//checks the initial motion
				m_nodeAgentMap[parent] = newAgent;
			m_nodeRATCache[target] = m_nodeRATCache[source];
			Cfg endLocation = this->GetStateGraph(m_sgLabel)->GetGraph()->GetVertex(parent);
			endLocation.SetRobot(newAgent->GetRobot());
			double endTime = readyTime + incomingEdge;
			Cfg startLocation = this->GetStateGraph(m_sgLabel)->GetGraph()->GetVertex(source);
			startLocation.SetRobot(newAgent->GetRobot());

			auto interval = new OccupiedInterval(newAgent->GetRobot(), startLocation, endLocation,
																					 readyTime, endTime);
	
	  	m_nodeRATCache[target][newAgent].push_back(interval);	
  	}
		m_parentMap[target] = source;
		m_incomingEdgeMap[target] = newDistance - _sourceDistance;

	}
	if(!m_nodeAgentMap[target])
		std::cout << "problem here" << std::endl;
  return newDistance;
}

