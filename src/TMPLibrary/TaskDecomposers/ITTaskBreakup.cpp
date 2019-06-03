#include "ITTaskBreakup.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"

#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Constraints/BoundaryConstraint.h"

#include "Simulator/Simulation.h"
#include "Simulator/BulletModel.h"

#include "Traits/CfgTraits.h"

ITTaskBreakup::
ITTaskBreakup(XMLNode& _node) : TaskDecomposerMethod(_node){}

ITTaskBreakup::
ITTaskBreakup(Robot* _robot) : m_robot(_robot) {}

ITTaskBreakup::
~ITTaskBreakup() = default;

void
ITTaskBreakup::
BreakupTask(WholeTask* _wholeTask){
  // Break the wholeTasks into subtasks based on when the robot pointer changes
  // in the wholeTask path. This change indicates that the robot capability
  // changed along the path in the megaRoadmap
  if(m_debug){
    std::cout << "Splitting up next whole task: " << _wholeTask << std::endl;
    std::cout << "Printing out path of whole task" << std::endl;
    for(auto cfg : _wholeTask->m_wholePath){
      std::cout << cfg.PrettyPrint()
                << " : : "
                << cfg.GetRobot()->GetCapability()
                << " : : "
                << cfg.GetRobot() << std::endl;
    }
  }
  Simulation::GetStatClass()->StartClock("IT Task Decomposition");

  Robot* currentRobot = nullptr;
  Cfg start;
  Cfg goal;
  for(auto cfg : _wholeTask->m_wholePath){
    if(cfg.GetRobot() == m_robot){
      continue;
    }
    else if(!currentRobot){
      start = cfg;
      currentRobot = cfg.GetRobot();
    }
    else if(currentRobot == cfg.GetRobot()){
      goal = cfg;
    }
    else if(currentRobot != cfg.GetRobot() and goal.GetRobot()){
      auto subtask = MakeSubtask(currentRobot,start,goal,_wholeTask);
      _wholeTask->m_subtasks.push_back(subtask);
      if(currentRobot->GetAgent()->GetCapability() != cfg.GetRobot()->GetAgent()->GetCapability()){
        currentRobot = cfg.GetRobot();
      }
      start = cfg;
    }
  }
  Cfg blank;
  if(m_robot and start.GetRobot() and goal.GetRobot()){
    auto subtask = MakeSubtask(currentRobot,start,goal,_wholeTask);
    _wholeTask->m_subtasks.push_back(subtask);
  }
  Simulation::GetStatClass()->StopClock("IT Task Decomposition");
}

std::shared_ptr<MPTask>
ITTaskBreakup::
MakeSubtask(Robot* _robot, Cfg _start, Cfg _goal, WholeTask* _wholeTask){

  auto& map = _wholeTask->m_interactionPoints;
  std::vector<Cfg> startPath = {};
  std::vector<Cfg> goalPath = {};
  for(auto& interactionPoint : map){
    if(*interactionPoint.first == _start){
      startPath = *interactionPoint.second;
    }
    else if(*interactionPoint.first == _goal){
      goalPath = *interactionPoint.second;
    }
  }

  std::shared_ptr<MPTask> subtask = std::shared_ptr<MPTask>(new MPTask(_robot));
  // Make start and goal constrants from start and end cfgs

  if(m_debug and _start == _goal){
    std::cout << "start and end are same" << std::endl;
  }

  // Create new subtask for a non-manipulator robot
  if(!_goal.GetRobot()->IsManipulator()){
    auto radius = (_robot->GetMultiBody()->GetBoundingSphereRadius());
/*
    std::unique_ptr<CSpaceBoundingSphere> boundingSphere(
        new CSpaceBoundingSphere(_start.GetPosition(), 1.2*radius));
    auto startConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(_robot, std::move(boundingSphere)));


    std::unique_ptr<CSpaceBoundingSphere> boundingSphere2(
        new CSpaceBoundingSphere(_goal.GetPosition(), radius));
    auto goalConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(_robot, std::move(boundingSphere2)));

    subtask->SetStartConstraint(std::move(startConstraint));
    subtask->AddGoalConstraint(std::move(goalConstraint));*/

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

    /* std::unique_ptr<CSpaceBoundingSphere> boundingSphere2(
       new CSpaceBoundingSphere(goal->GetCenter(), radius));
       auto goalConstraint = std::unique_ptr<BoundaryConstraint>
       (new BoundaryConstraint(robots[0], std::move(boundingSphere2)));
       */
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

    subtask->SetStartConstraint(std::move(startConstraint));
    subtask->ClearGoalConstraints();
    subtask->AddGoalConstraint(std::move(goalConstraint));



  }
  // Create new subtask for a manipulator robot
  else {
    auto startBox = unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(8));
    startBox->ShrinkToPoint(_start);
    auto ranges = startBox->GetRanges();
    for(size_t i = 3; i < ranges.size(); i++){
      auto range = ranges[i];
      auto center = range.Center();
      startBox->SetRange(i, center-.05, center+.05);
    }

    std::cout << "Ranges for start constraint" << std::endl;
    for(auto r : startBox->GetRanges()){
      std::cout << r << std::endl;
    }

    auto startConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(_robot, std::move(startBox)));


    auto goalBox = unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(8));
    goalBox->ShrinkToPoint(_goal);
    ranges = goalBox->GetRanges();
    for(size_t i = 3; i < ranges.size(); i++){
      auto range = ranges[i];
      auto center = range.Center();
      goalBox->SetRange(i, center-.05, center+.05);

    }
    std::cout << "Ranges for goal constraint" << std::endl;
    for(auto r : goalBox->GetRanges()){
      std::cout << r << std::endl;
    }

    auto goalConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(_robot, std::move(goalBox)));


    subtask->SetStartConstraint(std::move(startConstraint));
    subtask->AddGoalConstraint(std::move(goalConstraint));
  }

  subtask->SetCapability(_robot->GetAgent()->GetCapability());

  if(m_debug){
    std::cout << "Start of subtask: " << _start.PrettyPrint() << std::endl;
    std::cout << "End of subtask: " << _goal.PrettyPrint() << std::endl;
  }
  _wholeTask->m_interactionPathsDelivering[subtask] = goalPath;
  _wholeTask->m_interactionPathsReceiving[subtask] = startPath;
  return subtask;
}
