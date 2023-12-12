#include "WorkspaceTranslationDistance.h"

/*------------------------------- Construction -------------------------------*/

WorkspaceTranslationDistance::
WorkspaceTranslationDistance() : DistanceMetricMethod() {
  this->SetName("WorkspaceTranslation");
}


WorkspaceTranslationDistance::
WorkspaceTranslationDistance(XMLNode& _node)
    : DistanceMetricMethod(_node) {
  this->SetName("WorkspaceTranslation");
}

/*----------------------------- Distance Interface ---------------------------*/

double
WorkspaceTranslationDistance::
Distance(const Cfg& _c1, const Cfg& _c2) {
  auto x = GetBodyCoordinates(_c1),
       y = GetBodyCoordinates(_c2);

  double sum = 0;
  for(size_t i = 0; i < x.size(); ++i)
    sum += (x[i] - y[i]).normsqr();
  return std::sqrt(sum);
}

/*---------------------------------- Helpers ---------------------------------*/

std::vector<Vector3d>
WorkspaceTranslationDistance::
GetBodyCoordinates(const Cfg& _c) const noexcept {
  _c.ConfigureRobot();
  const auto& bodies = _c.GetMultiBody()->GetBodies();

  std::vector<Vector3d> coordinates;
  for(const auto& body : bodies)
    coordinates.push_back(body.GetWorldTransformation().translation());

  return coordinates;
}
