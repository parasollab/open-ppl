#ifndef PMPL_TMP_TOOLS_H_
#define PMPL_TMP_TOOLS_H_

#include "TMPLibrary/TMPBaseObject.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

#include <iostream>

class TMPTools : public TMPBaseObject {
  public:

  	///@name Construction
    ///@{

  	TMPTools() = default;

	TMPTools(XMLNode& _node);

	virtual ~TMPTools() = default;  	

    ///@}
}

/*----------------------------------------------------------------------------*/

#endif

TMPTools::
TMPTools(XMLNode _node) : TMPBaseObject(_node) {}