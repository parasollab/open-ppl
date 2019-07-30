#include "TaskCBSPlanner.h"

#include "Simulator/Simulation.h"

#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/TMPTools/MultiAgentDijkstra.h"
#include "TMPLibrary/TMPTools/TMPTools.h"
#include "TMPLibrary/WholeTask.h"
#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"

#include "Utilities/CBS/NewCBSTree.h"

TaskCBSPlanner::
TaskCBSPlanner() {
  this->SetName("TaskCBSPlanner");
}

TaskCBSPlanner::
TaskCBSPlanner(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("TaskCBSPlanner");
  m_madLabel = _node.Read("madLabel", true, "", "MultiAgentDijkstra search to use.");
  m_sgLabel = _node.Read("sgLabel",true,"","State graph to use.");
	m_bypass1 = _node.Read("bypass1",false,false,"Flag to use bypass type 1 or not.");
	m_makespan = _node.Read("makespan",false,false,"Flag to use makespan vs sum of cost.");
}

TaskCBSPlanner::
~TaskCBSPlanner(){}

bool
TaskCBSPlanner::
Run(std::vector<WholeTask*> _wholeTasks, std::shared_ptr<TaskPlan> _plan) {
  if(_wholeTasks.empty()){
    _wholeTasks = this->GetTaskPlan()->GetWholeTasks();
  }

	size_t totalNodes = 1;
	size_t maxDepth = 0;
	size_t minNodeDepth = 0;
	size_t nodesExplored = 1;

  std::shared_ptr<TaskPlan> savedPlan = nullptr;
  if(_plan){
		savedPlan = this->GetTaskPlan();
    this->GetTMPLibrary()->SetTaskPlan(_plan);
  }

  std::shared_ptr<TaskPlan> initialPlan = std::shared_ptr<TaskPlan>(new TaskPlan());
  *(initialPlan.get()) = *(this->GetTaskPlan().get());

  for (auto& wholeTask : _wholeTasks){
    this->GetTMPTools()->GetMultiAgentDijkstra(m_madLabel)->Run(wholeTask, initialPlan);
    initialPlan->InitializeRAT();
  }
	if(m_debug) {
		std::cout << "Initial Plan" << std::endl;
  	for(auto& tv: initialPlan->GetTIM()){
    	std::cout << std::endl << "whole task pointer: " << tv.first << std::endl;
    	for(auto interval : tv.second){
      		std::cout << interval.Print() << std::endl;
    	}
  	}
	}

  //auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
  // create initial node to grow tree from.
  std::shared_ptr<TaskCBSNode<WholeTask, OccupiedInterval>> initialNode = 
		std::shared_ptr<TaskCBSNode<WholeTask,OccupiedInterval>>(new TaskCBSNode<WholeTask,
    OccupiedInterval>(initialPlan, this->GetTaskPlan()));//,sg->GetAvailableIntervalGraph().get());

  auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
  std::unordered_map<WholeTask*,std::set<size_t>>& validVIDs = initialNode->GetValidVIDs();

  for(auto kv : sg->GetTaskAigVIDs()) {
    for(auto vid : kv.second) {
      validVIDs[kv.first].insert(vid);
    }
    for(auto vit = sg->GetAvailableIntervalGraph()->begin();
        vit != sg->GetAvailableIntervalGraph()->end(); vit++) {
      if(!sg->GetAvailableIntervalGraph()->IsVertexInvalidated(vit->descriptor()))
        validVIDs[kv.first].insert(vit->descriptor());
    }
  }
  // initialize tree
  NewCBSTree<WholeTask, OccupiedInterval> tree = NewCBSTree<WholeTask, OccupiedInterval>();

  // if the initial plan has a conflict, update conflict map and grow tree.
  auto conflictMap = FindConflict(initialNode.get());
	size_t numConflicts = 0;
	for(auto kv : conflictMap) {
		numConflicts += kv.second.size();
	}
	numConflicts = numConflicts/2;
  if (!conflictMap.empty()){
		auto newNodes = GrowTree(initialNode,conflictMap);
		if(m_bypass1)
			totalNodes += Bypass1(initialNode,newNodes,tree,numConflicts);
		else {
			totalNodes += newNodes.size();
			for(auto newNode : newNodes) {
				tree.Insert(newNode);
			}
		}
		//delete initialNode;
    //for(auto node : GrowTree(initialNode, tree, conflictMap)){
    //  auto toReplan = node->GetToReplan();
    //  InsertConflict((*(node->GetConflicts()))[toReplan],conflictMap[toReplan]);
    //}
  }
	
	//GrowTree(initialNode,conflictMap);

  double minPlanCost = std::numeric_limits<double>::max();
  std::shared_ptr<TaskPlan> minPlan = initialPlan;

  while(!tree.Empty()){
    // grab minimum cost node, cast NewCBSNode as a TaskCBSNode.
    std::shared_ptr<TaskCBSNode<WholeTask, OccupiedInterval>> node =
      std::shared_ptr<TaskCBSNode<WholeTask, OccupiedInterval>>(
			static_cast<TaskCBSNode<WholeTask, OccupiedInterval>*>(tree.GetMinNode()));

    if(m_debug){
      std::cout << "Tree size: " << tree.Length() << std::endl;
      std::cout << "Node depth: " << node->GetDepth() << std::endl;
			std::cout << "Conflict size: " << std::endl;
			for(auto& conflictMap : *(node->GetConflicts())) {
				std::cout << conflictMap.first << " : " << conflictMap.second.size() << std::endl;
			}
    }
    // if the node's cost is greater than the current minimum plan's cost,
    // do not search further down that node.
    if(node->GetCost(m_makespan) >= minPlanCost and minPlan!=initialPlan)
      //continue;
      break;//Should only reach this point once all nodes are >= current plan cost bc of pq

		if(node->GetDepth() > maxDepth)
			maxDepth = node->GetDepth();
		nodesExplored += 1;

    // set the current plan to the current node's task plan.
    //TaskPlan* currentPlan = node->GetTaskPlan();


		std::set<size_t> validVIDs = node->GetValidVIDs()[node->GetToReplan()];
    // replan based on the new RAT.
    if(!node->HasReplanned()) { 
    	// create a RAT from the current node's plan and conflict map.
    	// CreateRAT will update the task plan's RAT.
    	node->CreateRAT();
    	if(m_debug){
      	std::cout << "Node depth: " << node->GetDepth() << std::endl;
      	std::cout << "Robot Availability Table:" << std::endl;
      	for(auto& ra : node->GetTaskPlan()->GetRAT()){
        	std::cout << ra.first << std::endl;
        	for(auto inter : ra.second) {
          	std::cout << inter.Print() << std::endl;
        	}
      	}
    	}

			auto validSolution = this->GetTMPTools()->GetMultiAgentDijkstra(m_madLabel)->Run(node->GetToReplan(),
        										 node->GetTaskPlan(),validVIDs);
			node->SetReplanned(true);
			if(!validSolution)
				continue;
		}
		else {
			if(m_debug) {
  			for (auto& tv: node->GetTaskPlan()->GetTIM()){
    			std::cout << std::endl << "whole task pointer: " << tv.first << std::endl;
    			for (auto interval : tv.second){
      			std::cout << interval.Print() << std::endl;
    			}
  			}
			}
		}

		conflictMap = FindConflict(node.get());
			
    if(!m_makespan and m_debug) {//m_debug){
      std::cout << "Found conflicts: " << std::endl;
      for(auto kv : conflictMap){
				std::cout << "WholeTask: " << kv.first << std::endl;
				for(auto conflict : kv.second) {
        	std::cout << conflict << std::endl
          					<< conflict->GetConstraint().Print() << std::endl;
				}
      }
    }
    /*if (!conflictMap.empty()){
      GrowTree(node, tree);
      continue;
      }*/
    if(m_debug){
      std::cout << "Min Plan Cost: " << minPlanCost << std::endl;
      std::cout << "Current Plan Cost: " << node->GetCost(m_makespan) << std::endl;

			if(minPlan != initialPlan) {
				std::cout << "Min Plan Lengths" << std::endl;
				for(auto ti : minPlan->GetTIM()) {
					std::cout << ti.first << " : " << ti.second.size() << std::endl;
				}
			}
			std::cout << "Current Plan Lengths" << std::endl;
			for(auto ti : node->GetTaskPlan()->GetTIM()) {
				std::cout << ti.first << " : " 
									<< ti.second.size() << " : " 
									<< node->GetTaskPlan()->GetPlanCost(ti.first) << std::endl;
				//for(auto i : ti.second){
				//	std::cout << i.Print() << std::endl;
				//}	
				//std::cout << std::endl;
			}
    }
		bool noConflicts = true;
		for(auto kv : conflictMap) {
			if(!kv.second.empty())
				noConflicts = false;
		}
    if (!noConflicts and (node->GetCost(m_makespan) < minPlanCost or minPlan==initialPlan)){
      
			auto newNodes = GrowTree(node,conflictMap);
			if(m_bypass1) {
				size_t numConflicts = 0;
				for(auto kv : conflictMap) {
					numConflicts += kv.second.size();
				}
				numConflicts = numConflicts/2;
				totalNodes += Bypass1(node,newNodes,tree,numConflicts);
			}
			else {
				totalNodes += newNodes.size();
				for(auto newNode : newNodes) {
					tree.Insert(newNode);
				}
			}
			//delete node->GetTaskPlan();
      //for(auto node : GrowTree(initialNode, tree,conflictMap)){
      //  auto toReplan = node->GetToReplan();
      //InsertConflict((*(node->GetConflicts()))[toReplan],conflictMap[toReplan]);
      //}
			//delete node;
      continue;
    }
    if(node->GetCost(m_makespan) < minPlanCost){
      minPlanCost = node->GetCost(m_makespan);
			//delete minPlan;
      minPlan = node->GetTaskPlan();
			minNodeDepth = node->GetDepth();
    }
		//delete node;
  }
/*
  for (auto& tv: minPlan->GetTIM()){
    std::cout << std::endl << "whole task pointer: " << tv.first << std::endl;
    for (auto interval : tv.second){
      std::cout << interval.Print() << std::endl;
    }
  }
*/

  FinalizeTaskPlan(minPlan);
  *(this->GetTaskPlan()) = *(minPlan.get());

	//Simulation::GetStatClass()->SetStat("TotalNodes", totalNodes);
	Simulation::GetStatClass()->SetStat("TotalNodes", nodesExplored+tree.Length());
	Simulation::GetStatClass()->SetStat("MaxDepth", maxDepth);
	Simulation::GetStatClass()->SetStat("MinNodeDepth", minNodeDepth);
	Simulation::GetStatClass()->SetStat("NodesExplored", nodesExplored);
	Simulation::GetStatClass()->SetStat("PlanCost", minPlanCost);

  //Restore initial library task plan pointer if a different plan was passed in.
  if(savedPlan)
    this->GetTMPLibrary()->SetTaskPlan(savedPlan);
  return true;
}


std::vector<TaskCBSNode<WholeTask, OccupiedInterval>*>
TaskCBSPlanner::
GrowTree(std::shared_ptr<TaskCBSNode<WholeTask, OccupiedInterval>> _parentNode,
    //NewCBSTree<WholeTask, OccupiedInterval>& _tree,
    std::unordered_map<WholeTask*,std::vector<NewConflict<OccupiedInterval>*>> _conflictMap){

	std::set<WholeTask*> conflictedTasks;

  std::vector<TaskCBSNode<WholeTask, OccupiedInterval>*> newNodes;
	auto count = _conflictMap.size();
	auto index = 0;
  for (std::pair<WholeTask*, std::vector<NewConflict<OccupiedInterval>*>> kv :
    // *(_parentNode->GetConflicts())) {
  	_conflictMap) {
		for(auto conflict : kv.second){
			index++;
		
			if(m_debug)
				std::cout << "HERE 1" << std::endl;
			
			WholeTask* task = kv.first;

			if(conflictedTasks.count(task))
				continue;
			conflictedTasks.insert(task);

			TaskCBSNode<WholeTask, OccupiedInterval>* newNode = new TaskCBSNode<WholeTask,
							OccupiedInterval>(_parentNode.get(), this->GetTaskPlan(),task);

   	/*for (NewConflict<OccupiedInterval>* conflict :
    	((*(_parentNode->GetConflicts()))[task])){
	
    	newNode->AddConflict(task, conflict);
    	}*/
    	InsertConflict((*(newNode->GetConflicts()))[task],conflict);

    	auto interval = conflict->GetConstraint();
    	auto agent = interval.GetAgent();
    	std::unordered_map<Agent*,std::vector<OccupiedInterval>> updates;

    	updates[agent].push_back(interval);

    	auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
    	auto vidSets = sg->UpdateAvailableIntervalGraph(updates,newNode->GetToReplan(),
																										newNode->GetValidVIDs()[newNode->GetToReplan()]);

			if(m_debug) {
				std::cout << "Updating vids for task " << index << " out of " << count << ": " << task << std::endl;
			}
			newNode->UpdateValidVIDs(vidSets.first,vidSets.second,newNode->GetToReplan());
			if(m_debug) {
				std::cout << "Updated vids for task " << index << " out of " << count << ": " << task << std::endl;
			}
    	newNodes.push_back(newNode);
		}
  }
  return newNodes;
}


std::unordered_map<WholeTask*,std::vector<NewConflict<OccupiedInterval>*>>
TaskCBSPlanner::
FindConflict(TaskCBSNode<WholeTask, OccupiedInterval>* _node){
  std::unordered_map<WholeTask*,std::vector<NewConflict<OccupiedInterval>*>> conflictMap;
  int size = _node->GetTaskPlan()->GetWholeTasks().size();
  for (int i = 0; i < size; i++){
    WholeTask* task = (_node->GetTaskPlan()->GetWholeTasks())[i];
		conflictMap[task] = {};
	}
  for (int i = 0; i < size - 1; i++){
    WholeTask* firstTask = (_node->GetTaskPlan()->GetWholeTasks())[i];
    for (int j = i + 1; j < size; j ++){
      WholeTask* secondTask = (_node->GetTaskPlan()->GetWholeTasks())[j];

      OccupiedInterval firstInterval;// = new OccupiedInterval();
      OccupiedInterval secondInterval;// = new OccupiedInterval();

      // check if intervals contain a conflict
      if(CompareIntervals(firstTask, secondTask, _node->GetTaskPlan()->GetTIM(),
            firstInterval, secondInterval)){
        // create a new conflict that holds the overlapping intervals.
        TaskConflict<OccupiedInterval>* firstConflict =
          new TaskConflict<OccupiedInterval>(secondInterval, secondInterval.GetAgent(),
              firstTask);

        TaskConflict<OccupiedInterval>* secondConflict =
          new TaskConflict<OccupiedInterval>(firstInterval, firstInterval.GetAgent(),
              secondTask);

        // insert conflicts into the conflict map.
        //InsertConflict((*(_node->GetConflicts()))[firstTask], firstConflict);
        //InsertConflict((*(_node->GetConflicts()))[secondTask], secondConflict);
        conflictMap[firstTask].push_back(firstConflict);
        conflictMap[secondTask].push_back(secondConflict);
        //if(!m_bypass1)
					return conflictMap;
      }

    }
  }
  return conflictMap;
}


bool
TaskCBSPlanner::
CompareIntervals(WholeTask* _task1, WholeTask* _task2, std::unordered_map<WholeTask*,
    std::list<OccupiedInterval>> _TIM, OccupiedInterval& _interval1,
    OccupiedInterval& _interval2){
  for (OccupiedInterval it1 : _TIM[_task1]){
    for (OccupiedInterval it2 : _TIM[_task2]){
      // if intervals do not have the same agent, go to next iteration.
      if (it1.GetAgent() != it2.GetAgent())
        continue;

      // if intervals do not overlap and are reachable, go to next iteration
      if (!(it1.CheckTimeOverlap(it2))){
        if(it1.GetStartTime() < it2.GetStartTime()){
          if(CheckReachable(it1, it2))
            continue;
        }
        else{
          if(CheckReachable(it2, it1))
            continue;
        }
      }

      // otherwise, return true and save the occupied intervals.
      _interval1 = it1;
      _interval2 = it2;
      return true;
    }
  }
  return false;
}


bool
TaskCBSPlanner::
CheckReachable(OccupiedInterval& _interval1, OccupiedInterval& _interval2){
  auto sg = this->GetStateGraph(m_sgLabel);
  auto mtg = static_cast<MultiTaskGraph*>(sg.get());
  auto travel = mtg->LowLevelGraphPathWeight(_interval1.GetEndLocation(),
      _interval2.GetStartLocation());
  return ((_interval1.GetEndTime() + travel) <= _interval2.GetStartTime());
}

void
TaskCBSPlanner::
InsertConflict(std::list<NewConflict<OccupiedInterval>*>& _conflicts,
    NewConflict<OccupiedInterval>* _newConflict){
  // if the conflict map is empty, insert the new conflict and exit function.
  //if(_conflicts.empty()){
    _conflicts.push_back(_newConflict);
    return;
  //}

  OccupiedInterval& newInterval = _newConflict->GetConstraintRef();

  // update the iterator until it is a conflict with the same agent as the new
  // conflict.
  std::list<NewConflict<OccupiedInterval>*>::iterator it = _conflicts.begin();
  while(it != _conflicts.end() and
      (*it)->GetConstraint().GetAgent() != newInterval.GetAgent()){
    it++;

	}

  while(it != _conflicts.end()){
    OccupiedInterval& currentInterval = (*it)->GetConstraintRef();

    // nextIterator is the first iterator after the current iterator that is
    auto nextIterator = std::next(it);
    while(nextIterator != _conflicts.end() &&
        (*nextIterator)->GetConstraint().GetAgent() != newInterval.GetAgent()){
      nextIterator = std::next(nextIterator);
    }

    // if the next iterator is the end of the conflicts list, then add the
    // conflict to the end of the list.
    if (nextIterator == _conflicts.end() &&
        newInterval.GetStartTime() > currentInterval.GetEndTime()){
      _conflicts.push_back(_newConflict);
      return;
    }
    else if (nextIterator == _conflicts.end()){
      double newStart = std::min(currentInterval.GetStartTime(),
          newInterval.GetEndTime());
      double newEnd = std::max(currentInterval.GetEndTime(),
          newInterval.GetEndTime());

      currentInterval.SetStartTime(newStart);
      currentInterval.SetEndTime(newEnd);
      return;
    }

    OccupiedInterval& nextInterval = (*nextIterator)->GetConstraintRef();
    // if the new interval has a time overlap with the current interval...
    if(newInterval.CheckTimeOverlap(currentInterval)){
      // check if the new interval does not have a time overlap with the
      // following interval -> update the current interval.
      if (!newInterval.CheckTimeOverlap(nextInterval)){
        double newStart = std::min(currentInterval.GetStartTime(),
            newInterval.GetEndTime());
        double newEnd = std::max(currentInterval.GetEndTime(),
            newInterval.GetEndTime());

        currentInterval.SetStartTime(newStart);
        currentInterval.SetEndTime(newEnd);
        return;
      }

      vector<std::list<NewConflict<OccupiedInterval>*>::iterator> removeConflicts;
      removeConflicts.push_back(it);
      auto lastIterator = nextIterator;

      // while the new interval overlaps with the next same agent interval...
      while (newInterval.CheckTimeOverlap((*nextIterator)->GetConstraint())){
        // add the current valid nextIterator to be removed
        removeConflicts.push_back(nextIterator);

        // set last iterator, which will be later modified, to next iterator
        lastIterator = nextIterator;

        // find next iterator
        nextIterator = std::next(nextIterator);
        // while the next iterator's agent is not equivalent to the new
        // interval's agent, continue incrementing next agent.
        while(nextIterator != _conflicts.end() &&
            (*nextIterator)->GetConstraint().GetAgent() != newInterval.GetAgent()){
          nextIterator == std::next(nextIterator);
				}

        // if the next iterator reaches the end of the list, continue to
        // removing and modifying conflicts from the list.
        if (nextIterator == _conflicts.end())
          break;
      }

      // remove last iterator in removeConflicts. this will be the lastIterator
      // value that we will want to modify, not remove.
      removeConflicts.pop_back();

      // update the last iterator with the min start time and max end time.
      double newStart = std::min(currentInterval.GetStartTime(),
          newInterval.GetStartTime());
      double newEnd = std::max((*lastIterator)->GetConstraint().GetEndTime(),
          newInterval.GetEndTime());

			OccupiedInterval& lastInterval = (*lastIterator)->GetConstraintRef();
      lastInterval.SetStartTime(newStart);
      lastInterval.SetEndTime(newEnd);

      // remove conflicts that have been merged into the last iterator.
      for (auto c : removeConflicts)
        _conflicts.erase(c);
      return;
    }
    else if (newInterval.GetStartTime() > currentInterval.GetEndTime() &&
        newInterval.GetEndTime() < nextInterval.GetStartTime()){
      _conflicts.insert(nextIterator, _newConflict);
      return;
    }

    it ++;
    // while the current iterator does not ahve the same agent as the new
    // conflict, continue iterating.
    while(it != _conflicts.end() &&
        (*it)->GetConstraint().GetAgent() != newInterval.GetAgent()){
      it++;
		}
  }
  _conflicts.push_back(_newConflict);
  return;
}

std::shared_ptr<MPTask>
TaskCBSPlanner::
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


void
TaskCBSPlanner::
FinalizeTaskPlan(std::shared_ptr<TaskPlan> _plan){
  // TODO: Setup task dependency map.
  std::unordered_map<HandoffAgent*, std::list<std::pair<double,
    std::shared_ptr<MPTask>>>> trashMap;
  _plan->RemoveAllDependencies();
  for (auto& ti : _plan->GetTIM()){
    auto intervals = ti.second;
    ti.first->m_subtasks.clear();
    ti.first->m_agentAssignment.clear();

    std::cout << std::endl << "WholeTask Pointer: " << ti.first << std::endl;
    std::shared_ptr<MPTask> previous = nullptr;
    for (auto inter : intervals){
      auto subtask = (inter.GetTask())
        ? inter.GetTask()
        : CreateMPTask(inter.GetAgent()->GetRobot(),
            inter.GetStartLocation(), inter.GetEndLocation());
      if (previous)
        _plan->AddDependency(previous, subtask);
      trashMap[inter.GetAgent()].push_back(std::make_pair(inter.GetStartTime(),
            subtask));
      previous = subtask;
      ti.first->m_subtasks.push_back(subtask);
      ti.first->m_agentAssignment.push_back(inter.GetAgent());
      if(m_debug){
        std::cout << inter.Print() << std::endl;
      }
    }
  }
  _plan->InitializeAgentTaskMap();
  for (auto& t : trashMap){
    t.second.sort();
    for(auto kv : t.second){
      _plan->AddSubtask(t.first, kv.second);
    }
  }
}

size_t
TaskCBSPlanner::
Bypass1(std::shared_ptr<TaskCBSNode<WholeTask, OccupiedInterval>> _node,
				std::vector<TaskCBSNode<WholeTask, OccupiedInterval>*> _children,
				NewCBSTree<WholeTask, OccupiedInterval>& _tree,
				size_t _numConflicts) {

	std::set<TaskCBSNode<WholeTask,OccupiedInterval>*> badNodes;

	for(auto child : _children) {
		
		std::set<size_t> validVIDs = child->GetValidVIDs()[child->GetToReplan()];
		auto validSolution = this->GetTMPTools()->GetMultiAgentDijkstra(m_madLabel)->Run(child->GetToReplan(),
        									child->GetTaskPlan(),validVIDs);
		child->SetReplanned(true);

		if(!validSolution)	
			badNodes.insert(child);
		
		if(child->GetCost(m_makespan) > _node->GetCost(m_makespan))
			continue;

		auto conflictMap = FindConflict(child);
		size_t childConflicts = 0;
		for(auto kv : conflictMap)
			childConflicts += kv.second.size();
		childConflicts = childConflicts/2;

		if(childConflicts < _numConflicts) {
			auto copy = new TaskCBSNode<WholeTask,OccupiedInterval>();
			*copy = *(_node.get());

			copy->SetReplanned(true);
			copy->SetTaskPlan(child->GetTaskPlan());
			copy->SetDepth(_node->GetDepth());
			copy->SetToReplan(_node->GetToReplan());
			


			_tree.Insert(copy);

			std::cout << "BYPASSING CONFLICTS" << std::endl;
			std::cout << "BYPASSING CONFLICTS" << std::endl;
			std::cout << "BYPASSING CONFLICTS" << std::endl;
			std::cout << "BYPASSING CONFLICTS" << std::endl;
			std::cout << "BYPASSING CONFLICTS" << std::endl;
			std::cout << "BYPASSING CONFLICTS" << std::endl;
			std::cout << "BYPASSING CONFLICTS" << std::endl;

			return 0;
		}
	}  		

	for(auto child : _children) {
		if(badNodes.count(child)) {
			delete child;
			continue;
		}
		_tree.Insert(child);
	}

	return _children.size() - badNodes.size();
}
