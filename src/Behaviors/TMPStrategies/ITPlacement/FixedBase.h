#ifndef FIXED_BASE_H_
#define FIXED_BASE_H_

#include "PlacementMethod.h"

#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class FixedBase : public PlacementMethod {

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
                         Coordinator* _coordinator) override;

    ///@}


  private:


};

#endif

