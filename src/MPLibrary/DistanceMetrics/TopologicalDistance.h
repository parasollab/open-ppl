#ifndef PMPL_TOPOLOGICAL_DISTANCE_H_
#define PMPL_TOPOLOGICAL_DISTANCE_H_

#include "DistanceMetricMethod.h"

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
class TopologicalDistance : virtual public DistanceMetricMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    

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
    virtual double Distance(const Cfg& _c1, const Cfg& _c2) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_tmLabel; ///< The topological map to use.

    /// How to combine distances for multiple bodies.
    std::function<void(double&, const double)> m_operator;

    ///@}
};

#endif
