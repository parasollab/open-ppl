#ifndef PMPL_TOPOLOGICAL_DISTANCE_H_
#define PMPL_TOPOLOGICAL_DISTANCE_H_

#include "DistanceMetricMethod.h"

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>



////////////////////////////////////////////////////////////////////////////////
/// For two free configurations, determine the euclidean shortest-path distance
/// (or inner-distance) between their containing cells in a workspace
/// decomposition using a topological map. Returns infinity if either
/// configuration is in obstacle space or if the configurations are in
/// disconnected regions of workspace according to the topological map's grid
/// approximation.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TopologicalDistance : virtual public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    TopologicalDistance();

    TopologicalDistance(XMLNode& _node);

    virtual ~TopologicalDistance() = default;

    ///@}
    ///@name Distance Metric Overrides
    ///@{

    /// Estimate the inner-distance through connected workspace between the
    /// decomposition cells holding each configuration. If the robot is a
    /// multibody, the distances between individual bodies are combined with the
    /// operator function listed in the XML node.
    /// @param _c1 The first configuration.
    /// @param _c2 The second configuration.
    /// @return The inner-distance between the base at _c1 and _c2.
    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_tmLabel; ///< The topological map to use.

    /// How to combine distances for multiple bodies.
    std::function<void(double&, const double)> m_operator;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
TopologicalDistance<MPTraits>::
TopologicalDistance() : DistanceMetricMethod<MPTraits>() {
  this->SetName("TopologicalDistance");
}


template <typename MPTraits>
TopologicalDistance<MPTraits>::
TopologicalDistance(XMLNode& _node) :
    DistanceMetricMethod<MPTraits>(_node) {
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

template <typename MPTraits>
double
TopologicalDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  // Locate the neighborhoods where the cfgs are located.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
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

/*----------------------------------------------------------------------------*/

#endif
