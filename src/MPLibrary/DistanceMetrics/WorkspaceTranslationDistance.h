#ifndef PMPL_WORKSPACE_TRANSLATION_DISTANCE_H_
#define PMPL_WORKSPACE_TRANSLATION_DISTANCE_H_

#include "DistanceMetricMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Compute the L2 norm of the workspace translation between each body.
///
/// This is a reasonable approximation of swept volume for either a composite
/// c-space or a manipulator with very constrained joint motion. Large joint
/// motions break down the assumptions as the links may have to sweep large
/// distances in workspace to complete the transition - in that case it is a
/// poor approximation. It is best avoided for manipulators which make large
/// changes in joint space.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
class WorkspaceTranslationDistance : virtual public DistanceMetricMethod {

  public:

    ///@name Local Types
    ///@{

    

    ///@}
    ///@name Construction
    ///@{

    WorkspaceTranslationDistance();

    WorkspaceTranslationDistance(XMLNode& _node);

    virtual ~WorkspaceTranslationDistance() = default;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const Cfg& _c1, const Cfg& _c2) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    std::vector<mathtool::Vector3d> GetBodyCoordinates(const Cfg& _c) const
        noexcept;

    ///@}

};

#endif
