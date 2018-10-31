#ifndef BALL_FILLING_H_
#define BALL_FILLING_H_

#include "PlacementMethod.h"

#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class BallFilling : public PlacementMethod {

  public:

    ///@name Construction
    ///@{

    BallFilling(MPProblem* _problem);

    BallFilling(MPProblem* _problem, XMLNode& _node);

    ~BallFilling() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library);

    ///@}


  private:


};

#endif

