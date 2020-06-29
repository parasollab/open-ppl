#ifndef TMPCBS_H_
#define TMPCBS_H_

#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"

#include "Utilities/CBS/DiscreteAgentAllocation.h"
#include "Utilities/CBS/DiscreteMotionConflict.h"
#include "Utilities/CBS/NewCBSTree.h"
#include "Utilities/CBS/TMPCBSNode.h"

#include "Utilities/GeneralCBS/GeneralCBS.h"

/************************************************************************************/
//
//
/***********************************************************************************/
class TMPCBS : public TaskEvaluatorMethod {

	public:

		///@name Local Types 
		///@{

		typedef TMPCBSNode<WholeTask, std::pair<size_t,std::pair<size_t,size_t>>> Node;
		typedef Node::Motion 																												 Motion;
		typedef Node::MotionConflict 																								 MotionConflict;
		typedef Node::TaskConflict																									 TaskConflict;

		///@}
		///@name Construction
		///@{

		TMPCBS();

		TMPCBS(XMLNode& _node);
	
		virtual ~TMPCBS();

		///@
		virtual bool Run(std::vector<WholeTask*> _wholeTasks = {}, std::shared_ptr<TaskPlan> _plan = nullptr) override;

	private:
		
		///@name Grow Tree Functions
		///@{
		std::vector<Node*> SplitVertexConflict(std::shared_ptr<Node> _node, WholeTask* _task1, WholeTask* _task2,
							HandoffAgent* _agent1, HandoffAgent* _agent2, size_t _time, size_t _vid1, size_t vid2);

		std::vector<Node*> SplitEdgeConflict(std::shared_ptr<Node> _node, WholeTask* _task1, WholeTask* _task2, 
																				 HandoffAgent* _agent1, HandoffAgent* _agent2,
																				 size_t _time, size_t _vid1, size_t _vid1b, size_t _vid2, size_t vid2b);

		///@}
		///@name Helper Functions
		///@{

		std::vector<size_t> TaskSearch(Node* _node);
		
		/// Looks at paths contained in the node and checks for interagent conflicts
		/// @param _node Node containing potential solution
		std::vector<Node*> FindConflict(std::shared_ptr<Node> _node);

		std::vector<Node*> FindTaskConflicts(std::shared_ptr<Node> _node);


		std::vector<Node*> CreateTaskConflictNodes(std::shared_ptr<Node> _node, 
												WholeTask* _task1, WholeTask* _task2,
												SubtaskPlan _subtask1, SubtaskPlan _subtask2, bool _setup = false);

		std::vector<Node*> FindMotionConflicts(std::shared_ptr<Node> _node);

		std::vector<size_t> DiscreteSearch(Node* _node, size_t _start, size_t _goal, 
																			 size_t _startTime, size_t _minEndTime = 0);

		bool UpdatePlan(Node* _node);

		void FinalizePlan(Node* _node);

		bool CheckSetupPath(std::shared_ptr<Node> _node, WholeTask* _task, 
							 size_t _startTime, size_t _endTime, size_t _startVID, size_t _endVID, HandoffAgent* _agent);

		void PatchSetupPaths(Node* _node);

		Decomposition* CreateDecomposition(std::vector<WholeTask*> _wholeTasks);

		void ConvertCBSSolutionToTaskPlan(const CBSSolution& _solution);

		std::shared_ptr<TaskSolution> ConvertAssignmentToTaskSolution(Assignment& _assign);

		///@}
		///@name Internal State
		///@{
		bool m_makespan{false}; ///<Flag indicating evaluation metric to use	

		std::string m_vcLabel;	
		std::string m_dmadLabel;	

		///@}
};

#endif
