#ifndef OVERLAPPING_WORKSPACES_DENSITY_H_
#define OVERLAPPING_WORKSPACES_DENSITY_H_

#include "ITPlacementMethod.h"

#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class OverlappingWorkspacesDensity : public ITPlacementMethod {

  public:

    ///@name Construction
    ///@{

    OverlappingWorkspacesDensity(MPProblem* _problem);

    OverlappingWorkspacesDensity(MPProblem* _problem, XMLNode& _node);

    ~OverlappingWorkspacesDensity() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library,
                          TMPStrategyMethod* _tmpMethod) override;

    ///@}


  private:
    std::string m_dmLabel;
    double m_density{5};
    double m_proximity{1};
    double m_numNodes{100};


};

#endif

