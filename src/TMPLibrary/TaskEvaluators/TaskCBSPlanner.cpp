#include "TaskCBSPlanner.h"

#include"TMPLibrary/TaskPlan.h"
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
}

TaskCBSPlanner::
~TaskCBSPlanner(){}

bool
TaskCBSPlanner::
Run(std::vector<WholeTask*> _wholeTasks, TaskPlan* _plan) {
  if(_wholeTasks.empty()){
    _wholeTasks = this->GetTaskPlan()->GetWholeTasks();
  }

  TaskPlan* savedPlan = nullptr;
  if(_plan){
    savedPlan = this->GetTaskPlan();
    this->GetTMPLibrary()->SetTaskPlan(_plan);
  }

  TaskPlan* initialPlan = new TaskPlan();
  *initialPlan = *(this->GetTaskPlan());

  for (auto& wholeTask : _wholeTasks){
    this->GetTMPTools()->GetMultiAgentDijkstra(m_madLabel)->Run(wholeTask, initialPlan);
    initialPlan->InitializeRAT();
  }

  // create initial node to grow tree from.
  TaskCBSNode<WholeTask, OccupiedInterval>* initialNode = new TaskCBSNode<WholeTask,
    OccupiedInterval>(initialPlan, this->GetTaskPlan());

  // initialize tree
  NewCBSTree<WholeTask, OccupiedInterval> tree = NewCBSTree<WholeTask, OccupiedInterval>();

  // if the initial plan has a conflict, update conflict map and grow tree.
  auto conflictMap = FindConflict(initialNode);
  if (!conflictMap.empty()){
    GrowTree(initialNode, tree, conflictMap);
		//for(auto node : GrowTree(initialNode, tree, conflictMap)){
    //  auto toReplan = node->GetToReplan();
    //  InsertConflict((*(node->GetConflicts()))[toReplan],conflictMap[toReplan]);
    //}
  }

  double minPlanCost = std::numeric_limits<double>::max();
  TaskPlan* minPlan = initialPlan;

  while(!tree.Empty()){
    // grab minimum cost node, cast NewCBSNode as a TaskCBSNode.
    TaskCBSNode<WholeTask, OccupiedInterval>* node =
      static_cast<TaskCBSNode<WholeTask, OccupiedInterval>*>(tree.GetMinNode());

    // if the node's cost is greater than the current minimum plan's cost,
    // do not search further down that node.
    if(node->GetCost() > minPlanCost)
      continue;

    // set the current plan to the current node's task plan.
    //TaskPlan* currentPlan = node->GetTaskPlan();

    // create a RAT from the current node's plan and conflict map.
    // CreateRAT will update the task plan's RAT.
    node->CreateRAT();

		if(m_debug){
  		std::cout << "Robot Availability Table:" << std::endl;
  		for(auto& ra : node->GetTaskPlan()->GetRAT()){
    		std::cout << ra.first << std::endl;
    		for(auto inter : ra.second) {
      		std::cout << inter.Print() << std::endl;
    		}
  		}
		}

    // replan based on the new RAT.
    this->GetTMPTools()->GetMultiAgentDijkstra(m_madLabel)->Run(node->GetToReplan(),
        node->GetTaskPlan());

    conflictMap = FindConflict(node);

		if(m_debug){
			std::cout << "Found conflicts: " << std::endl;
			for(auto conflict : conflictMap){
				std::cout << conflict.first << std::endl
									<< conflict.second->GetConstraint().Print() << std::endl;
			}
		}
    /*if (!conflictMap.empty()){
      GrowTree(node, tree);
      continue;
    }*/
    if (!conflictMap.empty() and node->GetCost() < minPlanCost){
			GrowTree(node, tree,conflictMap);
      //for(auto node : GrowTree(initialNode, tree,conflictMap)){
      //  auto toReplan = node->GetToReplan();
      //InsertConflict((*(node->GetConflicts()))[toReplan],conflictMap[toReplan]);
      //}
      continue;
    }
		if(node->GetCost() < minPlanCost){
    	minPlanCost = node->GetCost();
    	minPlan = node->GetTaskPlan();
		}
  }

  FinalizeTaskPlan(minPlan);
  *(this->GetTaskPlan()) = *minPlan;

  //Restore initial library task plan pointer if a different plan was passed in.
  if(savedPlan)
    this->GetTMPLibrary()->SetTaskPlan(savedPlan);
  return true;
}


std::vector<TaskCBSNode<WholeTask, OccupiedInterval>*>
TaskCBSPlanner::
GrowTree(TaskCBSNode<WholeTask, OccupiedInterval>* _parentNode,
    NewCBSTree<WholeTask, OccupiedInterval>& _tree,
		std::unordered_map<WholeTask*,NewConflict<OccupiedInterval>*> _conflictMap){

  std::vector<TaskCBSNode<WholeTask, OccupiedInterval>*> newNodes;
  for (std::pair<WholeTask*, NewConflict<OccupiedInterval>*> kv :
                           // *(_parentNode->GetConflicts())) {
                           _conflictMap) {

    WholeTask* task = kv.first;
    TaskCBSNode<WholeTask, OccupiedInterval>* newNode = new TaskCBSNode<WholeTask,
                            OccupiedInterval>(_parentNode, this->GetTaskPlan(),task);

    /*for (NewConflict<OccupiedInterval>* conflict :
                            ((*(_parentNode->GetConflicts()))[task])){

      newNode->AddConflict(task, conflict);
    }*/
     InsertConflict((*(newNode->GetConflicts()))[task],kv.second);
    _tree.Insert(newNode);
    newNodes.push_back(newNode);
  }
  return newNodes;
}


std::unordered_map<WholeTask*,NewConflict<OccupiedInterval>*>
TaskCBSPlanner::
FindConflict(TaskCBSNode<WholeTask, OccupiedInterval>* _node){
  std::unordered_map<WholeTask*,NewConflict<OccupiedInterval>*> conflictMap;
  int size = _node->GetTaskPlan()->GetWholeTasks().size();
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
        conflictMap[firstTask] = firstConflict;
        conflictMap[secondTask] = secondConflict;
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
  if(_conflicts.empty()){
    _conflicts.push_back(_newConflict);
    return;
  }

  OccupiedInterval newInterval = _newConflict->GetConstraint();

  // update the iterator until it is a conflict with the same agent as the new
  // conflict.
  std::list<NewConflict<OccupiedInterval>*>::iterator it = _conflicts.begin();
  while(it != _conflicts.end() and
        (*it)->GetConstraint().GetAgent() != newInterval.GetAgent())
    it++;

  while(it != _conflicts.end()){
    OccupiedInterval currentInterval = (*it)->GetConstraint();

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

    OccupiedInterval nextInterval = (*nextIterator)->GetConstraint();
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
            (*nextIterator)->GetConstraint().GetAgent() != newInterval.GetAgent())
          nextIterator == std::next(nextIterator);

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

      (*lastIterator)->GetConstraint().SetStartTime(newStart);
      (*lastIterator)->GetConstraint().SetEndTime(newEnd);

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
        (*it)->GetConstraint().GetAgent() != newInterval.GetAgent())
      it++;
  }
  _conflicts.push_back(_newConflict);
  return;
}
/*
void
TaskCBSPlanner::
InsertConflict(std::list<NewConflict<OccupiedInterval>*>& _conflicts,
    NewConflict<OccupiedInterval>* _newConflict){
  std::list<NewConflict<OccupiedInterval>*>::iterator it = _conflicts.begin();

  OccupiedInterval newInterval = _newConflict->GetConstraint();
  std::cout << "Insert Conflict function ----------------------------" << std::endl;
  if(_conflicts.empty()){
    _conflicts.push_back(_newConflict);
    return;
  }

  while (it != _conflicts.end()){
    std::cout << "Inside While Loop" << std::endl;
    OccupiedInterval currentInterval = (*it)->GetConstraint();

    auto nextIterator = std::next(it);
    if(nextIterator == _conflicts.end() &&
        (newInterval.GetStartTime() > currentInterval.GetEndTime() ||
        newInterval.GetAgent() != currentInterval.GetAgent())){
      _conflicts.push_back(_newConflict);
      return;
    }
    else if (nextIterator == _conflicts.end()){
      double newStart = std::min(currentInterval.GetStartTime(),
          newInterval.GetEndTime());
      double newEnd = std::max(currentInterval.GetEndTime(),
          newInterval.GetEndTime());

      // update the current interval with the minimum start time and the
      // maximum end time.
      currentInterval.SetStartTime(newStart);
      currentInterval.SetEndTime(newEnd);
      return;
    }

    OccupiedInterval nextInterval = (*nextIterator)->GetConstraint();

    if (newInterval.CheckTimeOverlap(currentInterval)){

      if (!newInterval.CheckTimeOverlap(nextInterval)){
        // if the new conflict overlaps with the current conflict, but not the
        // next interval, update the current conflict.
        double newStart = std::min(currentInterval.GetStartTime(),
            newInterval.GetStartTime());
        double newEnd = std::max(currentInterval.GetEndTime(),
            newInterval.GetEndTime());

        // update the current interval with the minimum start time and the
        // maximum end time.
        currentInterval.SetStartTime(newStart);
        currentInterval.SetEndTime(newEnd);
        return;
        // TODO: delete _newConflict pointer?
      }

      // if the new interval overlaps with the next interval, loop through until
      // there is no overlap while keeping track of the end iterator.
      vector<std::list<NewConflict<OccupiedInterval>*>::iterator> removeConflicts;

      // add the current interval that overlapped with the
      removeConflicts.push_back(it);
      auto lastIterator = it;

      while(newInterval.CheckTimeOverlap(nextInterval)){
        // while the next iterator's agent is not equivalent to the new
        // interval's agent, continue incrementing next agent.
        while((*nextIterator)->GetConstraint().GetAgent() != newInterval.GetAgent() &&
            nextIterator != _conflicts.end())
          nextIterator == std::next(nextIterator);

        // add iterator to the removeConflicts vector to remove in the future.
        removeConflicts.push_back(nextIterator);

        // keep track of last iterator so that we can update it with the new
        // conflict's information.
        lastIterator = nextIterator;

        nextIterator = std::next(nextIterator);
        if(nextIterator == _conflicts.end())
          break;
      }
      // remove last iterator in removeConflicts b/c this is the lastIterator
      // value that we want to modify, not remove.
      removeConflicts.pop_back();

      // update the last iterator with the minimum start time and the maximum
      // end time.
      double newStart = std::min(currentInterval.GetStartTime(),
          newInterval.GetStartTime());
      double newEnd = std::max((*lastIterator)->GetConstraint().GetEndTime(),
          newInterval.GetEndTime());

      (*lastIterator)->GetConstraint().SetStartTime(newStart);
      (*lastIterator)->GetConstraint().SetEndTime(newEnd);

      // erase the start iterator up to the last iterator if the agents are
      // all equivalent.
      for (auto c : removeConflicts)
        _conflicts.erase(c);

      return;
    }
    else if (newInterval.GetStartTime() > currentInterval.GetEndTime() &&
        newInterval.GetEndTime() < nextInterval.GetStartTime()){
      _conflicts.insert(nextIterator, _newConflict);
      return;
    }
    it++;
    while((*it)->GetConstraint().GetAgent() != newInterval.GetAgent() and
        it != _conflicts.end())
      it++;
  }
}
*/

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
FinalizeTaskPlan(TaskPlan* _plan){
  // TODO: Setup task dependency map.
  std::unordered_map<HandoffAgent*, std::list<std::pair<double,
    std::shared_ptr<MPTask>>>> trashMap;
  _plan->RemoveAllDependencies();
  for (auto& ti : _plan->GetTIM()){
    auto intervals = ti.second;
    ti.first->m_subtasks.clear();

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
      if(m_debug){
        std::cout << inter.Print() << std::endl;
      }
    }
  }
  _plan->InitializeAgentTaskMap();
  for (auto& t : trashMap){
    t.second.sort();
    for(auto kv : t.second){
      _plan -> AddSubtask(t.first, kv.second);
    }
  }
}
