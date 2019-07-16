#ifndef PMPL_AUCTION_METHOD_H_
#define PMPL_AUCTION_METHOD_H_

#include "TMPLibrary/TaskAllocators/TaskAllocatorMethod.h"

class AuctionMethod : public TaskAllocatorMethod {
  public:

  	///@name Construction
    ///@{

  	AuctionMethod();

	AuctionMethod(XMLNode& _node);

	virtual ~AuctionMethod() = default;  	

    ///@}
    ///@name Call Method
    ///@{

	virtual void AllocateTasks() override;

	///@}
  private:
  	///@name Helpers
  	///@{

  	void AuctionSubtasks(WholeTask* _wholeTask);

  	///@}
  	///@name Internal State
  	///@{

  	std::vector<std::shared_ptr<MPTask>> m_unassignedTasks;

  	///@}
  }
};

/*----------------------------------------------------------------------------*/

#endif
