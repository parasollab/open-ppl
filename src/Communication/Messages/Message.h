#ifndef _PMPL_MESSAGE_H_
#define _PMPL_MESSAGE_H_

#include <string>
#include <vector>


#include "MPLibrary/MPSolution.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/Control.h"
#include "MPProblem/Robot/Robot.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"

/* Contains functions for converting data into messages and messages into data */

//Helper
//Returns the top level in the first element and all immediate child nodes in the following elements
std::vector<std::string>
ParseSubMessage(std::string _msg);

/*--------------------------------------- Query Info --------------------------------*/

//Robot Team
std::string
RobotTeamToMessage(std::vector<Robot*> _robots, Robot* _coordinator = nullptr); 

std::pair<std::vector<Robot*>,std::string>
MessageToRobotTeam(std::string _msg, MPProblem* _problem);

//Decomposition
std::string
DecompositionToMessage(Decomposition* _decomp);

Decomposition*
MessageToDecomposition(std::string _msg, MPProblem* _problem);

//Semantic Task
std::string SemanticTaskToMessage(SemanticTask* _task);

SemanticTask* MessageToSemanticTask(std::string _msg, Decomposition* _decomp, MPProblem* _problem);

//Motion Planning Task
std::string
MPTaskToMessage(MPTask* _mpTask);

MPTask*
MessageToMPTask(std::string _msg, MPProblem* _problem);

std::string
ConstraintToMessage(const Constraint* _constraint);

std::unique_ptr<Constraint>
MessageToConstraint(std::string _msg, Robot* _robot);

/*---------------------------------- Plan Info ---------------------------------------*/

//Plan
std::string
PlanToMessage(Plan* _plan);

Plan*
MessageToPlan(std::string _msg, Decomposition* _decomp, MPProblem* _problem);

//Task Solution
std::string
TaskSolutionToMessage(TaskSolution* _solution);

TaskSolution*
MessageToTaskSolution(std::string _msg, Plan* _plan);

//Motion Solution
std::vector<std::string>
MotionSolutionToMessage(MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>* _solution, std::string _label);

MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>*
MessageToMotionSolution(std::string _msg);

//Roadmap
std::vector<std::string> RoadmapToMessage(RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _roadmap, Robot* _robot);

RoadmapGraph<Cfg,DefaultWeight<Cfg>>* MessageToRoadmap(std::string _msg, Robot* _robot);

/*------------------------------- Control Info  -------------------------------------*/

//Control 
std::string ControlSetToMessage(ControlSet& _c, size_t _steps);

std::pair<size_t,ControlSet> MessageToControlSet(std::string _msg, MPProblem* _problem);

#endif
