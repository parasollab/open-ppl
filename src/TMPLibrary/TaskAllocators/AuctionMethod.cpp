#include "AuctionMethod.h"

/*------------------------------ Construction --------------------------------*/

AuctionMethod::
AuctionMethod() {
	this->SetName("AuctionMethod");
}

AuctionMethod::
AuctionMethod(XMLNode& _node) : TaskAllocatorMethod(_node){
	this->SetName("AuctionMethod");
}
