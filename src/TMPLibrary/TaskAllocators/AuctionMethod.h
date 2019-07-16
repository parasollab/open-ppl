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
};

/*----------------------------------------------------------------------------*/

#endif
