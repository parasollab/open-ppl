#ifndef FIXED_BASE_H_
#define FIXED_BASE_H_

#include "TMPLibrary/PoIPlacementMethods/ITPlacement/ITPlacementMethod.h"

#include "Utilities/XMLNode.h"

class FixedBase : public ITPlacementMethod {

  public:

    ///@name Construction
    ///@{

    FixedBase(MPProblem* _problem);

    FixedBase(XMLNode& _node);

    ~FixedBase() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution) override;

    ///@}


  private:


};

#endif

