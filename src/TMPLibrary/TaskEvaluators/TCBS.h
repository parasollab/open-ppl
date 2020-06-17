#ifndef TCBS_H_
#define TCBS_H_

#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

#include "Utilities/CBS/DiscreteMotionConflict.h"
#include "Utilities/CBS/NewCBSTree.h"
#include "Utilities/CBS/TCBSNode.h"

/************************************************************************************/
//
// This is an implementation of the TCBS algorithm presented in An Optimal Algorithm
// to Solve the Combined Task Allocation and Path Finding Problem.
//
/***********************************************************************************/
class TCBS : public TaskEvaluatorMethod {

	public:

		///@name Local Types 
		///@{

		typedef TCBSNode<HandoffAgent, std::pair<size_t,std::pair<size_t,size_t>>> Node;
		typedef DiscreteMotionConflict<std::pair<size_t,std::pair<size_t,size_t>>> MotionConflict;

		///@}
		///@name Construction
		///@{

		TCBS();

		TCBS(XMLNode& _node);
	
		virtual ~TCBS();

		///@
		virtual bool Run(std::vector<WholeTask*> _wholeTasks = {}, std::shared_ptr<TaskPlan> _plan = nullptr) override;

	private:
		
		///@name Grow Tree Functions
		///@{
		std::vector<Node*> Expand(std::shared_ptr<Node> _node, std::vector<WholeTask*> _tasks);

		std::vector<Node*> SplitVertexConflict(std::shared_ptr<Node> _node, size_t _conflict,
										HandoffAgent* _firstAgent, HandoffAgent* _secondAgent);

		std::vector<Node*> SplitEdgeConflict(std::shared_ptr<Node> _node, size_t _conflict,
										HandoffAgent* _firstAgent, HandoffAgent* _secondAgent);

		///@}
		///@name Helper Functions
		///@{

		/// Looks at paths contained in the node and checks for interagent conflicts
		/// @param _node Node containing potential solution
		std::vector<Node*> FindConflict(std::shared_ptr<Node> _node);

		std::vector<size_t> DiscreteSearch(Node* _node, size_t _start, size_t _goal, 
																			 size_t _startTime, size_t _minEndTime = 0, bool _setup=false);

		bool UpdatePlan(Node* _node);

		void FinalizePlan(Node* _node);

		///@}
		bool m_makespan{false}; ///<Flag indicating evaluation metric to use	

		std::string m_vcLabel;	

};

#endif
