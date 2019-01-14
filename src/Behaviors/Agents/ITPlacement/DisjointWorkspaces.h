#ifndef DISJOINT_WORKSPACES_H_
#define DISJOINT_WORKSPACES_H_

#include "PlacementMethod.h"

#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class DisjointWorkspaces : public PlacementMethod {

  public:

    ///@name Construction
    ///@{

    DisjointWorkspaces(MPProblem* _problem);

    DisjointWorkspaces(MPProblem* _problem, XMLNode& _node);

    ~DisjointWorkspaces() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library);

    ///@}


  private:


};

#endif

