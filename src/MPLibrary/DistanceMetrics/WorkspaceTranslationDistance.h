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
template <typename MPTraits>
class WorkspaceTranslationDistance : virtual public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    WorkspaceTranslationDistance();

    WorkspaceTranslationDistance(XMLNode& _node);

    virtual ~WorkspaceTranslationDistance() = default;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    std::vector<mathtool::Vector3d> GetBodyCoordinates(const CfgType& _c) const
        noexcept;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
WorkspaceTranslationDistance<MPTraits>::
WorkspaceTranslationDistance() : DistanceMetricMethod<MPTraits>() {
  this->SetName("WorkspaceTranslation");
}


template <typename MPTraits>
WorkspaceTranslationDistance<MPTraits>::
WorkspaceTranslationDistance(XMLNode& _node)
    : DistanceMetricMethod<MPTraits>(_node) {
  this->SetName("WorkspaceTranslation");
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
WorkspaceTranslationDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  auto x = GetBodyCoordinates(_c1),
       y = GetBodyCoordinates(_c2);

  double sum = 0;
  for(size_t i = 0; i < x.size(); ++i)
    sum += (x[i] - y[i]).normsqr();
  return std::sqrt(sum);
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
std::vector<Vector3d>
WorkspaceTranslationDistance<MPTraits>::
GetBodyCoordinates(const CfgType& _c) const noexcept {
  _c.ConfigureRobot();
  const auto& bodies = _c.GetMultiBody()->GetBodies();

  std::vector<Vector3d> coordinates;
  for(const auto& body : bodies)
    coordinates.push_back(body.GetWorldTransformation().translation());

  return coordinates;
}

/*----------------------------------------------------------------------------*/

#endif
