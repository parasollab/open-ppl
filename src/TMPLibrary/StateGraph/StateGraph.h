#ifndef PMPL_STATE_GRAPH_H_
#define PMPL_STATE_GRAPH_H_

#include "TMPLibrary/TMPBaseObject.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

#include <iostream>

class StateGraph : public TMPBaseObject {
  public:

  	///@name Construction
    ///@{

  	StateGraph() = default;

	StateGraph(XMLNode& _node);

	virtual ~StateGraph() = default;  	

    ///@}
}

/*----------------------------------------------------------------------------*/

#endif