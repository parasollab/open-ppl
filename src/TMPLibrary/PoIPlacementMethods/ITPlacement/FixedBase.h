#ifndef FIXED_BASE_H_
#define FIXED_BASE_H_

#include "ITPlacementMethod.h"

#include "TMPLibrary/TMPTools/InteractionTemplate.h"

#include "Utilities/XMLNode.h"

class FixedBase : public ITPlacementMethod {

  public:

    ///@name Construction
    ///@{

    FixedBase(MPProblem* _problem);

    FixedBase(MPProblem* _problem, XMLNode& _node);

    ~FixedBase() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library,
                         TMPStrategyMethod* _tmpMethod) override;

    ///@}


  private:


};

#endif

