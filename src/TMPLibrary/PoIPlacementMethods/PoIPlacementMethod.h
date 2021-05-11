#ifndef PMPL_POI_PLACEMENT_METHOD_H_
#define PMPL_POI_PLACEMENT_METHOD_H_

#include "TMPLibrary/TMPBaseObject.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

#include <iostream>

class PoIPlacementMethod : public TMPBaseObject {
  public:

  	///@name Construction
    ///@{

  	PoIPlacementMethod() = default;

		PoIPlacementMethod(XMLNode& _node);

		virtual ~PoIPlacementMethod() = default;  	

    ///@}
};

/*----------------------------------------------------------------------------*/

#endif
