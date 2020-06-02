#include "PlanningServer.h"

/*--------------------------- Construction ---------------------------*/

PlanningServer::
PlanningServer(TMPLibrary* _library) : m_library(_library) {}

/*--------------------------- Interface ------------------------------*/

std::string
PlanningServer::
Solve(std::string _msg) const {
	return "A motion plan";
}
