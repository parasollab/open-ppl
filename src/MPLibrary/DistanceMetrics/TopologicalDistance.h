#ifndef TOPOLOGICAL_DISTANCE_H
#define TOPOLOGICAL_DISTANCE_H

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
/// Wrapper for another distance metric (termed the underlying DM). The
/// connected workspace distance is evaluated first - if that is equal, then the
/// underlying DM will be used.
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

    typedef std::pair<const WorkspaceRegion*, const WorkspaceRegion*> KeyPair;

    ///@}
    ///@name Construction
    ///@{

    TopologicalDistance();

    /// Construct a distance wrapper for a TopologicalFilter.
    /// @param _tmLabel The topological map label.
    /// @param _dmLabel The underlying DM label.
    TopologicalDistance(const std::string& _tmLabel,
        const std::string& _dmLabel);

    TopologicalDistance(XMLNode& _node);

    virtual ~TopologicalDistance() = default;

    ///@}
    ///@name Distance Metric Overrides
    ///@{

    /// First check the connected workspace distance between the decomposition
    /// cells holding each configuration. If that is equal, fall back to
    /// underlying DM.
    /// @param _c1 The first configuration.
    /// @param _c2 The second configuration.
    /// @return The distance between _c1 and _c2.
    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Update the distance map to include a new key pair. Since we have to
    /// compute an entire SSSP run to do this, we go ahead and save the mapping
    /// from the first cell to each other.
    void UpdateMap(const KeyPair& _key);

    /// Make a map key from two region pointers. Since this distance metric is
    /// symmetric, we will always store the lower address first to keep the
    /// key space compact.
    /// @param _r1 The first region.
    /// @param _r2 The second region.
    /// @return A key pair for {_r1,  _r2} which is identical to the key pair
    ///         for {_r2, _r1}.
    KeyPair MakeKey(const WorkspaceRegion* _r1, const WorkspaceRegion* _r2)
        const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_tmLabel; ///< The topological map to use.
    std::string m_dmLabel; ///< Label for the underlying distance metric.

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
TopologicalDistance(const std::string& _tmLabel,
    const std::string& _dmLabel) :
    DistanceMetricMethod<MPTraits>(), m_tmLabel(_tmLabel), m_dmLabel(_dmLabel) {
  this->SetName("TopologicalDistance");
}


template <typename MPTraits>
TopologicalDistance<MPTraits>::
TopologicalDistance(XMLNode& _node) :
    DistanceMetricMethod<MPTraits>(_node) {
  this->SetName("TopologicalDistance");

  m_tmLabel = _node.Read("tmLabel", true, "", "The topological map to use.");

  m_dmLabel = _node.Read("dmLabel", true, "", "The underlying distance metric.");
}

/*---------------------- DistanceMetricMethod Overrides ----------------------*/

template <typename MPTraits>
double
TopologicalDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "TopologicalDistance::Distance");

  // Locate the cells where the cfgs are located.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto cell1 = tm->LocateRegion(_c1);
  auto cell2 = tm->LocateRegion(_c2);

  // If they are the same, their topological distance is 0. Fall back to the
  // underlying NF.
  if(cell1 == cell2) {
    stats->IncStat("TopologicalDistance::Fallback");
    return this->GetDistanceMetric(m_dmLabel)->Distance(_c1, _c2);
  }
  stats->IncStat("TopologicalDistance::Used");

  // Topological distance is symmetric, so we can save on the distance map
  // storage by always using key pairs that are sorted.
  const KeyPair key = MakeKey(cell1, cell2);

  // If the distance has already been calculated, return it now.
  auto iter = m_distanceMap.find(key);
  if(iter != m_distanceMap.end())
    return iter->second;

  // Otherwise, update the distance map to include this key before returning.
  UpdateMap(key);
  return m_distanceMap[key];
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
void
TopologicalDistance<MPTraits>::
UpdateMap(const KeyPair& _key) {
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto decomposition = tm->GetDecomposition();

  // Compute the SSSP from this cell and save all results to the distance map.
  const auto sssp = tm->ComputeSSSP(_key.first);

  for(const auto& item : sssp.distance) {
    const auto& region = decomposition->GetRegion(item.first);
    const KeyPair key = MakeKey(_key.first, &region);

    m_distanceMap[key] = item.second;
  }
}


template <typename MPTraits>
typename TopologicalDistance<MPTraits>::KeyPair
TopologicalDistance<MPTraits>::
MakeKey(const WorkspaceRegion* _r1, const WorkspaceRegion* _r2) const noexcept {
  return {std::min(_r1, _r2),
          std::max(_r1, _r2)};
}

/*----------------------------------------------------------------------------*/

#endif
