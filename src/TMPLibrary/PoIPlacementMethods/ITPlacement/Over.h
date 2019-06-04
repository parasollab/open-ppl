#ifndef OVERLAPPING_WORKSPACES_DENSITY_H_
#define OVERLAPPING_WORKSPACES_DENSITY_H_

#include "TMPLibrary/PoIPlacementMethods/ITPlacement/ITPlacementMethod.h"

class OverlappingWorkspacesDensity : public ITPlacementMethod {

  public:

    ///@name Construction
    ///@{

    OverlappingWorkspacesDensity(MPProblem* _problem);

    OverlappingWorkspacesDensity(XMLNode& _node);

    ~OverlappingWorkspacesDensity() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution);

    ///@}


  private:
    std::string m_dmLabel;
    double m_density{5};
    double m_proximity{1};


};

#endif

