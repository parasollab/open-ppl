#ifndef WORKSAPCE_GUIDANCE_H_
#define WORKSPACE_GUIDANCE_H_

#include "TMPLibrary/PoIPlacementMethods/ITPlacement/ITPlacementMethod.h"

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

		WorkspaceGuidance() = default;

    WorkspaceGuidance(XMLNode& _node);

    ~WorkspaceGuidance() = default;

		std::unique_ptr<ITPlacementMethod> Clone() override;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution) override;

    ///@}


  private:

    void BuildSkeleton();


    std::string m_dmLabel;
    double m_distanceThreshold;
    WorkspaceSkeleton m_skeleton;
    bool m_initialized{false};

};

#endif
