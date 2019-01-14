#ifndef GRID_FILLING_H_
#define GRID_FILLING_H_

#include "PlacementMethod.h"

#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class GridFilling : public PlacementMethod {

  public:

    ///@name Construction
    ///@{

    GridFilling(MPProblem* _problem);

    GridFilling(MPProblem* _problem, XMLNode& _node);

    ~GridFilling() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library);

    ///@}


  private:


};

#endif

