#include "TopologicalDistance.h"

#include "MPLibrary/MPLibrary.h"

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>

/*------------------------------- Construction -------------------------------*/

TopologicalDistance::
TopologicalDistance() : DistanceMetricMethod() {
  this->SetName("TopologicalDistance");
}


TopologicalDistance::
TopologicalDistance(XMLNode& _node) :
    DistanceMetricMethod(_node) {
  this->SetName("TopologicalDistance");

  m_tmLabel = _node.Read("tmLabel", true, "", "The topological map to use.");

  // Parse the operator string.
  std::string op = _node.Read("operator", false, "max",
      "The operator to use for multibodies {sum, max, min}");
  std::transform(op.begin(), op.end(), op.begin(), ::tolower);

  if(op == "sum")
    m_operator = [](double& _output, const double _d) {
      _output += _d;
    };
  else if(op == "max")
    m_operator = [](double& _output, const double _d) {
      _output = std::max(_output, _d);
    };
  else if(op == "min")
    m_operator = [](double& _output, const double _d) {
      _output = std::min(_output, _d);
    };
  else
    throw ParseException(_node.Where()) << "Unrecognized operator '" << op
                                        << "'.";
}

/*---------------------- DistanceMetricMethod Overrides ----------------------*/

double
TopologicalDistance::
Distance(const Cfg& _c1, const Cfg& _c2) {
  // Locate the neighborhoods where the cfgs are located.
  auto tm = this->GetMPLibrary()->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto neighborhood1 = tm->LocateNeighborhood(_c1);
  auto neighborhood2 = tm->LocateNeighborhood(_c2);

  double output = 0;

  // Compute the topological distance between each region in the neighborhood
  // keys and combine with the operator function.
  for(size_t i = 0; i < neighborhood1.size(); ++i) {
    const WorkspaceRegion* const cell1 = neighborhood1[i],
                         * const cell2 = neighborhood2[i];

    // Ensure that the map has computed a inner-distances to the maximum extent
    // possible (this is a no-op if we've already computed it).
    tm->ComputeApproximateMinimumInnerDistances(cell1,
        std::numeric_limits<double>::infinity());

    // Get the inner distance according to the map.
    const double distance = tm->ApproximateMinimumInnerDistance(cell1, cell2);

    // If it is infinite, these regions aren't connectable.
    if(std::isinf(distance))
      return distance;

    // Otherwise, apply the operator function.
    m_operator(output, distance);
  }

  return output;
}
