#include "TaskSolution.h"

#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
/*----------------------------------- Construction -----------------------------*/

TaskSolution::
TaskSolution(SemanticTask* _task) : m_task(_task) {}
		
TaskSolution::
~TaskSolution() {}

SemanticTask* 
TaskSolution::
GetTask() {
	return m_task;
}

/*------------------------------------ Accessors ------------------------------*/

void 
TaskSolution::
SetRobot(Robot* _robot) {
	m_robot = _robot;
}

Robot* 
TaskSolution::
GetRobot() {
	return m_robot;
}

void 
TaskSolution::
SetRobotGroup(RobotGroup* _group) {
	m_robotGroup = _group;
}

RobotGroup* 
TaskSolution::
GetRobotGroup() {
	return m_robotGroup;
}

void 
TaskSolution::
SetMotionSolution(MPSolution* _solution) {
	m_motionSolution = _solution;
}

TaskSolution::MPSolution* 
TaskSolution::
GetMotionSolution() {
	return m_motionSolution;
}
		
void 
TaskSolution::
SetStartTime(double _startTime) {
	m_startTime = _startTime;
}

double 
TaskSolution::
GetStartTime() {
	return m_startTime;
}
	
/*--------------------------------------- Print ----------------------------*/	
void 
TaskSolution::
Print() {
	std::cout << "Solution for " << m_task->GetLabel() << std::endl;
	std::cout << "Allocated to ";
	if(m_robot) {
		std::cout << m_robot->GetLabel();
	}
	else if(m_robotGroup) {
		std::cout << m_robotGroup->GetLabel() << " (";
		for(auto r : m_robotGroup->GetRobots())
			std::cout << r->GetLabel() << ", ";
	}

	std::cout << std::endl << "Start time: " << m_startTime << std::endl;

	std::cout << "Path: ";
	if(m_robot) {
		for(auto vid : m_motionSolution->GetPath(m_robot)->VIDs()) {
			std::cout << vid << ", ";
		}
	}
	else if(m_robotGroup) {
    std::cout << "Composite path" << std::endl;
    std::unordered_map<std::string,std::vector<Cfg>> individualPaths;
		for(auto r : m_robotGroup->GetRobots()) {
      std::vector<Cfg> path;
      individualPaths[r->GetLabel()] = path;
    }
    for(auto g : m_motionSolution->GetGroupPath(m_robotGroup)->Cfgs()) {
      for(auto r : m_robotGroup->GetRobots()) {
        individualPaths[r->GetLabel()].push_back(g.GetRobotCfg(r));
      }
    }
    for(auto kv : individualPaths) {
      std::cout << std::endl << kv.first << std::endl;
      for(auto c : kv.second) {
        std::cout << "[" << c.PrettyPrint() << "],    ";
      }
    }
    std::cout << "Decoupled path" << std::endl;
    for(auto r : m_robotGroup->GetRobots()) {
      std::cout << std::endl << r->GetLabel() << std::endl;
      for(auto vid : m_motionSolution->GetPath(r)->VIDs()) {
        std::cout << vid << ", ";
      }
      std::cout << std::endl;
      for(auto cfg : m_motionSolution->GetPath(r)->Cfgs()) {
        std::cout << cfg.PrettyPrint() << std::endl;
      }
    }
	}
	std::cout << std::endl << std::endl;	
}
