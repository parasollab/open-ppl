#ifndef PMPL_TASK_DECOMPOSER_METHOD_H_
#define PMPL_TASK_DECOMPOSER_METHOD_H_

#include "TMPLibrary/TMPBaseObject.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

#include <iostream>

class TaskDecomposerMethod : public TMPBaseObject {
  public:

  	///@name Construction
    ///@{

  	TaskDecomposerMethod() = default;

	TaskDecomposerMethod(XMLNode& _node);

	virtual ~TaskDecomposerMethod() = default;  	

    ///@}
}

/*------------------------------ Construction --------------------------------*/

TaskDecomposerMethod::
TaskDecomposerMethod(XMLNode& _node) : TMPBaseObject(_node) {
}

/*----------------------------------------------------------------------------*/

#endif