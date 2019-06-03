#ifndef WORKSAPCE_GUIDANCE_H_
#define WORKSPACE_GUIDANCE_H_

#include "ITPlacementMethod.h"

#include "Behaviors/Agents/Coordinator.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "ConfigurationSpace/RoadmapGraph.h"

#include "TMPLibrary/TMPTools/InteractionTemplate.h"

#include "Utilities/XMLNode.h"

#include "Workspace/WorkspaceSkeleton.h"

class WorkspaceGuidance : public ITPlacementMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    //typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> RoadmapGraph;
    //typedef typename RoadmapGraph::VID                     VID;

    ///@}
    ///@name Construction
    ///@{

    WorkspaceGuidance(MPProblem* _problem);

    WorkspaceGuidance(MPProblem* _problem, XMLNode& _node);

    ~WorkspaceGuidance() = default;

		std::unique_ptr<PlacementMethod> Clone() override;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library,
                         TMPStrategyMethod* _tmpMethod) override;

    ///@}


  private:

    void BuildSkeleton(MPLibrary* _library, TMPStrategyMethod* _tmpMethod);


    std::string m_dmLabel;
    double m_distanceThreshold;
    WorkspaceSkeleton m_skeleton;
    bool m_initialized{false};

};

#endif
