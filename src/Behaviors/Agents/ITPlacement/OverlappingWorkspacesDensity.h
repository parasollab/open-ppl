#ifndef OVERLAPPING_WORKSPACES_DENSITY_H_
#define OVERLAPPING_WORKSPACES_DENSITY_H_

#include "PlacementMethod.h"

#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class OverlappingWorkspacesDensity : public PlacementMethod {

  public:

    ///@name Construction
    ///@{

    OverlappingWorkspacesDensity(MPProblem* _problem);

    OverlappingWorkspacesDensity(MPProblem* _problem, XMLNode& _node);

    ~OverlappingWorkspacesDensity() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library);

    ///@}


  private:


};

#endif

