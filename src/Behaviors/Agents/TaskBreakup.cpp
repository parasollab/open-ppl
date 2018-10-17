#include "TaskBreakup.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "MPProblem/MPHandoffTemplate.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "Simulator/Simulation.h"
#include "Simulator/BulletModel.h"
#include "Traits/CfgTraits.h"

TaskBreakup::
TaskBreakup(Robot* _robot) : m_robot(_robot) {}

TaskBreakup::
~TaskBreakup() = default;

void
TaskBreakup::
BreakupTask(WholeTask* _wholeTask){
  // Break the wholeTasks into subtasks based on when the robot pointer changes
  // in the wholeTask path. This change indicates that the robot capability
  // changed along the path in the megaRoadmap
  Simulation::GetStatClass()->StartClock("IT Task Decomposition");
  if(m_debug){
    std::cout << "Splitting up next whole task: " << _wholeTask << std::endl;
    std::cout << "Printing out path of whole task" << std::endl;
    for(auto cfg : _wholeTask->m_wholePath){
      std::cout << cfg.PrettyPrint() << " : : " << cfg.GetRobot()->GetCapability() << std::endl;
    }
  }
  // Loop through path to find cfgs where robot pointer changes
  // When the robot pointer changes, create a new subtask
  // TODO: Maybe check based on capability rather than robot pointer
  Robot* checkRobot = nullptr;
  Cfg subtaskStart;
  Cfg subtaskEnd;
  // First cfg in the path is the coordinator configurations at the
  // start of the whole task
  //TODO: Need to figure out how to have node that is differernt for
  //coordinator when its still an "icreate" and not include it in the agent
  //paths
  size_t startIndex = 0;
  for(size_t i = startIndex; i < _wholeTask->m_wholePath.size(); i++){
    auto cfg = _wholeTask->m_wholePath[i];
    if(i == _wholeTask->m_wholePath.size()-1){
      cfg.SetRobot(_wholeTask->m_wholePath[i-1].GetRobot());
      subtaskEnd = cfg;
    }
    else if(cfg.GetRobot() == m_robot){
      if(i == startIndex){
        //TODO: This is hacky and is a result of the above stated TODO issue
        //because the initial cfg in the path is with the coordinator
        cfg.SetRobot(_wholeTask->m_wholePath[i+1].GetRobot());
      }
      else{
        cfg.SetRobot(checkRobot);
      }
    }
    if(m_debug){
      std::cout << "Robot pointer in path: " << cfg.GetRobot() << std::endl;
      std::cout << cfg.PrettyPrint() << std::endl;
    }
    // If it has not reached the handoff or end of path, keep moving end point
    // back
    if(cfg.GetRobot()==checkRobot and i != (_wholeTask->m_wholePath.size()-1)){
      if(m_debug) {
        std::cout << "Same robot ptr" << std::endl;
      }
      subtaskEnd = cfg;
    }
    else{
      if(m_debug){
        std::cout << "Different robot ptr or end of path" << std::endl;
      }
      // If it's the start of a new subtask, set new start point
      if(!checkRobot){
        subtaskEnd = cfg;
        checkRobot = cfg.GetRobot();
        if(i == startIndex){
          subtaskStart = cfg;
        }
      }
      // If we have reached a handoff or end of the path, create a subtask of
      // the wholetask
      else {
        if(m_debug){
          std::cout << "End of subtask" << std::endl;
        }
        std::shared_ptr<MPTask> subtask = std::shared_ptr<MPTask>(new MPTask(m_robot));
        // Make start and goal constrants from start and end cfgs

        if(m_debug and subtaskStart == subtaskEnd){
          std::cout << "start and end are same" << std::endl;
        }

        // Create new subtask for a non-manipulator robot
        if(!m_robot->IsManipulator()){
          auto radius = 1.2 * (m_robot->GetMultiBody()->GetBoundingSphereRadius());

          std::unique_ptr<CSpaceBoundingSphere> boundingSphere(
              new CSpaceBoundingSphere(subtaskStart.GetPosition(), radius));  
          auto startConstraint = std::unique_ptr<BoundaryConstraint>
            (new BoundaryConstraint(checkRobot, std::move(boundingSphere)));


          std::unique_ptr<CSpaceBoundingSphere> boundingSphere2(
              new CSpaceBoundingSphere(subtaskEnd.GetPosition(), radius));  
          auto goalConstraint = std::unique_ptr<BoundaryConstraint>
            (new BoundaryConstraint(checkRobot, std::move(boundingSphere2)));

          subtask->SetStartConstraint(std::move(startConstraint));
          subtask->AddGoalConstraint(std::move(goalConstraint));
        }
        // Create new subtask for a manipulator robot
        else {
          auto startBox = unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(8));
          startBox->ShrinkToPoint(subtaskStart);
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
            (new BoundaryConstraint(checkRobot, std::move(startBox)));


          auto goalBox = unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(8));
          goalBox->ShrinkToPoint(subtaskEnd);
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
            (new BoundaryConstraint(checkRobot, std::move(goalBox)));


          subtask->SetStartConstraint(std::move(startConstraint));
          subtask->AddGoalConstraint(std::move(goalConstraint));
        }

        subtask->SetCapability(subtaskStart.GetRobot()->GetAgent()->GetCapability()); 

        if(m_debug){
          std::cout << "Start of subtask: " << subtaskStart.PrettyPrint() << std::endl;
          std::cout << "End of subtask: " << subtaskEnd.PrettyPrint() << std::endl;
        }

        // Push task back into whole task object
        _wholeTask->m_subtasks.push_back(subtask);
        checkRobot = nullptr;
        if(m_debug){
          std::cout << "Start of new subtask" << std::endl;
        }
        subtaskStart = cfg;
      }
    } 
  }
  Simulation::GetStatClass()->StopClock("IT Task Decomposition");
}
