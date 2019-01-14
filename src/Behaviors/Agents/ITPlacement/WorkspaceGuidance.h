#ifndef WORKSAPCE_GUIDANCE_H_
#define WORKSPACE_GUIDANCE_H_

#include "PlacementMethod.h"

#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class WorkspaceGuidance : public PlacementMethod {

  public:

    ///@name Construction
    ///@{

    WorkspaceGuidance(MPProblem* _problem);

    WorkspaceGuidance(MPProblem* _problem, XMLNode& _node);

    ~WorkspaceGuidance() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library);

    ///@}


  private:


};

#endif
