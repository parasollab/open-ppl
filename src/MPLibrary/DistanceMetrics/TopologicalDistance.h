#ifndef PMPL_TOPOLOGICAL_DISTANCE_H
#define PMPL_TOPOLOGICAL_DISTANCE_H

#include "DistanceMetricMethod.h"

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>


namespace std {

  //////////////////////////////////////////////////////////////////////////////
  /// Define a hasher for a pair of region pointers for use with unordered maps.
  //////////////////////////////////////////////////////////////////////////////
  template <>
  struct hash<std::pair<const WorkspaceRegion*, const WorkspaceRegion*>> {

    typedef std::pair<const WorkspaceRegion*, const WorkspaceRegion*> KeyPair;

    /// @TODO This is a guess at a good hashing function. We need to validate
    ///       that it is a reasonable assumption.
    static constexpr size_t magicOffset = std::numeric_limits<size_t>::max() / 7;

    size_t operator()(const KeyPair& _key) const {
      static constexpr std::hash<const WorkspaceRegion*> hasher;
      return (hasher(_key.first) + magicOffset) ^ hasher(_key.second);
    }

  };

}


////////////////////////////////////////////////////////////////////////////////
/// For two free configurations, determine the distance between their containing
/// cells in a workspace decomposition using a topological map.
///
/// This metric is symmetric. It will produce an infinite distance if either
/// configuration cannot be located in the topological map.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TopologicalDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Local Types
    ///@{

    /// A pair of regions, which act as a hash key when caching the inter-region
    /// distances. Since this distance metric is symmetric, we will always
    /// store the lower address first to keep the key space compact.
    typedef std::pair<const WorkspaceRegion*, const WorkspaceRegion*> KeyPair;

    /// The different ways that we may combine the topological distance for each
    /// body in a multibody.
    enum class MultiBodyOperator {Sum, Max, Min};

    ///@}
    ///@name Construction
    ///@{

    TopologicalDistance();

    TopologicalDistance(XMLNode& _node);

    virtual ~TopologicalDistance() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Distance Metric Overrides
    ///@{

    /// First check the connected workspace distance between the decomposition
    /// cells holding each configuration's base. If that is equal, fall back to
    /// underlying DM.
    /// @param _c1 The first configuration.
    /// @param _c2 The second configuration.
    /// @return The connected workspace distance between the base at _c1 and _c2.
    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Update the distance map to include a new key pair. Since we have to
    /// compute an entire SSSP run to do this, we go ahead and save the mapping
    /// from the first cell to each other.
    /// @param _key The key pair.
    void UpdateMap(const KeyPair& _key);

    /// Make a map key from two region pointers.
    /// @param _r1 The first region.
    /// @param _r2 The second region.
    /// @return A key pair for {_r1,  _r2} which is identical to the key pair
    ///         for {_r2, _r1}.
    KeyPair MakeKey(const WorkspaceRegion* const _r1,
        const WorkspaceRegion* const _r2) const noexcept;

    /// Compute the topological distance between two workspace cells.
    /// @param _r1 The source workspace cell.
    /// @param _r2 The target workspace cell.
    /// @return The distance from _r1 to _r2 through free workspace as
    ///         determined by the topological map.
    double CellDistance(const WorkspaceRegion* const _r1,
        const WorkspaceRegion* const _r2);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_tmLabel; ///< The topological map to use.

    /// How to combine distances for multiple bodies.
    MultiBodyOperator m_operator;

    /// Track the distance between each cell.
    std::unordered_map<KeyPair, double> m_distanceMap;

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
  std::string op = _node.Read("operator", true, "",
      "The operator to use for multibodies {sum, max, min}");
  std::transform(op.begin(), op.end(), op.begin(), ::tolower);

  if(op == "sum")
    m_operator = MultiBodyOperator::Sum;
  else if(op == "max")
    m_operator = MultiBodyOperator::Max;
  else if(op == "min")
    m_operator = MultiBodyOperator::Min;
  else
    throw ParseException(_node.Where()) << "Unrecognized operator '" << op
                                        << "'.";
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
TopologicalDistance<MPTraits>::
Initialize() {
  m_distanceMap.clear();
}

/*---------------------- DistanceMetricMethod Overrides ----------------------*/

template <typename MPTraits>
double
TopologicalDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "TopologicalDistance::Distance");

  // Locate the neighborhoods where the cfgs are located.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto neighborhood1 = tm->LocateNeighborhood(_c1);
  auto neighborhood2 = tm->LocateNeighborhood(_c2);

  auto mb = _c1.GetMultiBody();
  double output = 0;

  // Compute the distance based on the designated operator.
  using F = std::function<void(const double)>;
  F f;
  switch(m_operator) {
    case MultiBodyOperator::Sum:
      f = F([&output](const double _d) {output += _d;});
      break;
    case MultiBodyOperator::Max:
      f = F([&output](const double _d) {output = std::max(output, _d);});
      break;
    case MultiBodyOperator::Min:
      f = F([&output](const double _d) {output = std::min(output, _d);});
      break;
    default:
      throw RunTimeException(WHERE) << "Illegal operator.";
  }

  // Compute the topological distance between each region in the neighborhood
  // keys.
  for(size_t i = 0; i < neighborhood1.size(); ++i) {
    // Get the topological distance between the decomposition cells for body i.
    const WorkspaceRegion* const cell1 = neighborhood1[i],
                         * const cell2 = neighborhood2[i];
    const double distance = CellDistance(cell1, cell2);

    // If it is infinite, the answer is known.
    if(std::isinf(distance))
      return distance;

    // If it is zero, the bodies are in the same cell. We will use the euclidean
    // distance between bodies in that case.
    if(distance == 0.) {
      _c1.ConfigureRobot();
      const Vector3d p1 = mb->GetBody(i)->GetWorldTransformation().translation();
      _c2.ConfigureRobot();
      const Vector3d p2 = mb->GetBody(i)->GetWorldTransformation().translation();

      const double interBodyDistance = (p1 - p2).norm();
      f(interBodyDistance);
    }
    // Otherwise, apply the operator function.
    else
      f(distance);
  }

  return output;
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
void
TopologicalDistance<MPTraits>::
UpdateMap(const KeyPair& _key) {
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto decomposition = tm->GetDecomposition();

  // Compute the frontier from this cell and save results to the distance map.
  const auto sssp = tm->ComputeFrontier(_key.first);

  for(const auto& item : sssp.distance) {
    const auto& region = decomposition->GetRegion(item.first);
    const KeyPair key = MakeKey(_key.first, &region);

    m_distanceMap[key] = item.second;
  }
}


template <typename MPTraits>
typename TopologicalDistance<MPTraits>::KeyPair
TopologicalDistance<MPTraits>::
MakeKey(const WorkspaceRegion* const _r1, const WorkspaceRegion* const _r2)
    const noexcept {
  return {std::min(_r1, _r2),
          std::max(_r1, _r2)};
}


template <typename MPTraits>
double
TopologicalDistance<MPTraits>::
CellDistance(const WorkspaceRegion* const _r1,
    const WorkspaceRegion* const _r2) {
  // If either cell is null, then one of the configurations is in obstacle
  // space. Their topological distance is then infinite.
  if(!_r1 or !_r2)
    return std::numeric_limits<double>::infinity();

  // If they are the same, their topological distance is 0.
  if(_r1 == _r2)
    return 0;

  // Look for the distance between _r1 and _r2 in the distance map.
  const KeyPair key = MakeKey(_r1, _r2);
  auto iter = m_distanceMap.find(key);

  // If we found it, we are done.
  if(iter != m_distanceMap.end())
    return iter->second;

  // Otherwise, update the distance map to include this key before returning.
  UpdateMap(key);
  return m_distanceMap[key];
}

/*----------------------------------------------------------------------------*/

#endif
