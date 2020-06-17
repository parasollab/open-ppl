#ifndef _PMPL_PLANNING_SERVER_H_
#define _PMPL_PLANNING_SERVER_H_

#include <sstream>
#include <string>

#include "TMPLibrary/TMPLibrary.h"

class PlanningServer {
	public:
		///@name LocalTypes
		///@{

		struct PlanningProblem {
			Coordinator* 				 coordinator;
			std::vector<Robot*>  team;
			Decomposition*       decomposition;
		};

		///@}
		///@name Construction
		///@{

		PlanningServer(TMPLibrary* m_library);

		~PlanningServer() = default;

		///@}		
		///@name Interface
		///@{

		std::string Solve(std::string _msg) const;

		///@}

	private:
		///@name Helper Functions
		///@{

		PlanningProblem ParseMessage(std::string _msg) const;

		///@}
		///@name Internal State
		///@{

		TMPLibrary* m_library;

		///@}
};

#endif
