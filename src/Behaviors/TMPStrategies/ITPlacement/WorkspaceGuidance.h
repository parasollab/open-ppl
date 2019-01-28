#ifndef WORKSAPCE_GUIDANCE_H_
#define WORKSPACE_GUIDANCE_H_

#include "PlacementMethod.h"

#include "Behaviors/Agents/Coordinator.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"
#include "Workspace/WorkspaceSkeleton.h"

class WorkspaceGuidance : public PlacementMethod {

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

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library,
                         Coordinator* _coordinator) override;

    ///@}


  private:

    void BuildSkeleton(MPLibrary* _library, Coordinator* _coordinator);


    std::string m_dmLabel;
    double m_distanceThreshold;
    WorkspaceSkeleton m_skeleton;
    bool m_initialized{false};

};

#endif
