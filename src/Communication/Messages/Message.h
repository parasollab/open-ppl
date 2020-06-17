#ifndef _PMPL_MESSAGE_H_
#define _PMPL_MESSAGE_H_

#include <string>
#include <vector>

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/Robot.h"

/* Contains functions for converting data into messages and messages into data */

//Helper
//Returns the top level in the first element and all immediate child nodes in the following elements
std::vector<std::string>
ParseSubMessage(std::string _msg);

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
#endif
