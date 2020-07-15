#ifndef TCBS_ALLOCATOR_H_
#define TCBS_ALLOCATOR_H_

///////////////////////////////////////////////////////////////////////////////
///
/// This is an implementation of the TCBS combined task allocation and path 
/// finding method presented in 'An Optimal Algorithm to Solve the Combined 
/// Task Allocation and Path Finding Problem.'
///
///////////////////////////////////////////////////////////////////////////////


#include "TaskAllocatorMethod.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "TMPLibrary/WholeTask.h"

class TCBSAllocator : public TaskAllocatorMethod {

	public:
  	///@name Construction
    ///@{

  	TCBSAllocator();

		TCBSAllocator(XMLNode& _node);

		virtual ~TCBSAllocator();  	

    ///@}
    ///@name Call Method
    ///@{

		virtual void AllocateTasks();

		///@}

	private: 
	
		///@name Helper Functions
		///@{

		Decomposition* CreateDecomposition(std::vector<WholeTask*> _wholeTask);

		///@}
		///@name Internal State
		///@{

		std::string m_sgLabel;

		std::string m_vcLabel;

		bool m_lowLevelDebug;

		///< Indicates if the allocation and motion planning are coupled or decoupled.
		bool m_decoupled{false};

		///@}

};

#endif
