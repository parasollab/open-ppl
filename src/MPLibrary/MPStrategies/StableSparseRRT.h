#ifndef STABLE_SPARSE_RRT_H_
#define STABLE_SPARSE_RRT_H_

#include <vector>
#include <algorithm>
#include <unordered_map>

#include "BasicRRTStrategy.h"


////////////////////////////////////////////////////////////////////////////////
/// StableSparseRRT is an RRT variant which aims to maintain a sparse 'active
/// set' of vertices.
///
/// The active set is used for neighborhood finding instead of the entire
/// roadmap. The density of the active set is controlled by the 'witness
/// configurations', which enforce that there will be at most one active vertex
/// within the 'witness radius' (these are different from the witness
/// configurations used elsewhere in PMPL). The active vertex within a witness's
/// ball region is always the one with the best total cost from the start
/// configuration. This is distance in holonomic problems and time steps in
/// nonholonomic problems.
///
/// Paper reference:
/// Sparse Methods for Efficient Asymptotically Optimal Kinodynamic Planning.
///   Yanbo Li, Zakary Littlefield, and Kostas Bekris. Algorithmic Foundations
///   of Robotics XI (WAFR 2014). 2015. 263-282.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class StableSparseRRT : public BasicRRTStrategy<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    typedef typename BasicRRTStrategy<MPTraits>::TreeType TreeType;

    ///@}
    ///@name Construction
    ///@{

    StableSparseRRT(string _dm = "euclidean", string _nf = "Nearest",
        string _vc = "rapid", string _nc = "kClosest", string _ex = "BERO",
        vector<string> _evaluators = vector<string>(),
        string _gt = "UNDIRECTED_TREE",  bool _growGoals = false,
        double _growthFocus = .05, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3,
        double _witnessRadius = 0.5, bool _prune = false);

    StableSparseRRT(XMLNode& _node);

    virtual ~StableSparseRRT() = default;

    ///@}
    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;

    ///@}

  private:

    ///@name BasicRRTStrategy Overrides
    ///@{

    /// SST's version of this function requires a radius NF and only searches
    /// the active set.
    virtual VID FindNearestNeighbor(const CfgType& _cfg, const TreeType& _tree)
        override;

    /// SST's version of this function calls the base class version and then
    /// updates metadata specific to this method.
    virtual VID Extend(const VID _nearest, const CfgType& _target,
        LPOutput<MPTraits>& _lp) override;

    ///@}
    ///@name SST Helpers
    ///@{

    /// Update the active set, witnesses, and representatives after a successful
    /// extension.
    /// @param _nearestVID The starting VID for the extension.
    /// @param _newVID The new VID created by the extension.
    /// @param _lp The local plan info.
    void UpdateSSTStructures(const VID _nearestVID, const VID _newVID,
        const LPOutput<MPTraits>& _lp);

    // Find nearest witness to a vertex.
    pair<VID, double> FindNearestWitness(const CfgType& _newCfg);

    /// Recursively prune the set of inactive leaves until none remain.
    void PruneInactiveLeaves();

    ///@}
    ///@name Internal State
    ///@{

    set<VID> m_active; ///< The active set represents the 'sparse' tree.
    vector<VID> m_witnesses; ///< Set of witnesses for enforcing sparseness.

    vector<VID> m_inactiveLeaves; ///< The inactive nodes with no children.
    unordered_map<VID, VID> m_parent; ///< The parent map.

    unordered_map<VID, VID> m_representatives; ///< Maps a representative node
                                               ///< to a witness.

    double m_witnessRadius{.5}; ///< Distance between witness nodes.

    bool m_prune{false};

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
StableSparseRRT<MPTraits>::
StableSparseRRT(string _dm, string _nf, string _vc, string _nc, string _ex,
    vector<string> _evaluators, string _gt, bool _growGoals, double _growthFocus,
    size_t _numRoots, size_t _numDirections, size_t _maxTrial,
    double _witnessRadius, bool _prune) :
    BasicRRTStrategy<MPTraits>(_dm, _nf, _vc, _nc, _ex, _evaluators, _gt,
        _growGoals, _growthFocus, _numRoots, _numDirections, _maxTrial),
    m_witnessRadius(_witnessRadius), m_prune(_prune) {
  this->SetName("StableSparseRRT");
}


template <typename MPTraits>
StableSparseRRT<MPTraits>::
StableSparseRRT(XMLNode& _node) : BasicRRTStrategy<MPTraits>(_node) {
  this->SetName("StableSparseRRT");

  m_witnessRadius = _node.Read("witnessRadius", true, 0.5, .01,
      std::numeric_limits<double>::max(), "Distance between witnesses.");
  m_prune = _node.Read("prune", false, false,
      "Enable/disable pruning inactive leaves");
}

/*-------------------------- MPStrategy Overrides ----------------------------*/

template <typename MPTraits>
void
StableSparseRRT<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();

  if(this->m_debug)
    std::cout << "Initializing StableSparseRRT" << std::endl;

  const auto graph = this->GetRoadmap()->GetGraph();
  const VID start = graph->GetVID(this->m_query->GetQuery()[0]);

  m_active.insert(start);
  m_witnesses.push_back(start);
  m_representatives.emplace(make_pair(start, start));
}

/*----------------------- BasicRRTStrategy Overrides -------------------------*/

template <typename MPTraits>
typename StableSparseRRT<MPTraits>::VID
StableSparseRRT<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const TreeType& _tree) {
  MethodTimer mt(this->GetStatClass(), "SST::FindNearestNeighbor");
  auto g = this->GetRoadmap()->GetGraph();

  std::vector<std::pair<VID, double>> neighbors;

  // The candidate neighbors are those in both the current tree and the active
  // set.
  std::vector<VID> candidates;
  std::set_intersection(_tree.begin(), _tree.end(),
      m_active.begin(), m_active.end(), std::back_inserter(candidates));

  if(candidates.empty())
    throw RunTimeException(WHERE, "SST can't find any candidates.");

  // Search only the candidates using a radius NF.
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  nf->FindNeighbors(this->GetRoadmap(), candidates.begin(), candidates.end(),
      candidates.size() == g->get_num_vertices(),
      _cfg, back_inserter(neighbors));

  // Of the nodes in the radius, select the one with the best path cost.
  VID bestVID = INVALID_VID;
  double bestCost = std::numeric_limits<double>::max();

  for(const auto& n : neighbors) {
    // Check for invalid neighbors.
    if(n.first == INVALID_VID)
      throw RunTimeException(WHERE, "NF should not return bogus nodes.");

    // Check if this neighbor has the best cost so far.
    const double cost = g->GetVertex(n.first).
        GetStat("cost");
    if(cost < bestCost) {
      bestCost = cost;
      bestVID = n.first;
    }
  }

  if(this->m_debug and !neighbors.empty()) {
    const VID nearest = neighbors[0].first;
    std::cout << "Found best active neighbor " << bestVID
              << " with path cost "
              << std::setprecision(4) << bestCost << ".\n"
              << "\tNearest was " << nearest << " with path cost "
              << std::setprecision(4) << g->GetVertex(nearest).GetStat("cost")
              << "."
              << std::endl;
  }

  return bestVID;
}


template <typename MPTraits>
typename StableSparseRRT<MPTraits>::VID
StableSparseRRT<MPTraits>::
Extend(const VID _nearest, const CfgType& _target, LPOutput<MPTraits>& _lp) {
  // Extend using the basic RRT method.
  const VID newVID = BasicRRTStrategy<MPTraits>::Extend(_nearest, _target, _lp);

  // If we succeeded, update the SST structures before we return.
  const bool success = newVID != INVALID_VID;
  if(success)
    UpdateSSTStructures(_nearest, newVID, _lp);

  return newVID;
}

/*----------------------------- SST Functions --------------------------------*/

template <typename MPTraits>
void
StableSparseRRT<MPTraits>::
UpdateSSTStructures(const VID _nearestVID, const VID _newVID,
    const LPOutput<MPTraits>& _lp) {
  MethodTimer mt(this->GetStatClass(), "SST::UpdateSSTStructures");

  auto g = this->GetRoadmap()->GetGraph();

  // The nearest node is now the parent of the new node.
  m_parent[_newVID] = _nearestVID;

  // If the parent node was an inactive leaf, it isn't any longer.
  {
    auto iter = std::find(m_inactiveLeaves.begin(), m_inactiveLeaves.end(),
        _nearestVID);
    if(iter != m_inactiveLeaves.end())
      m_inactiveLeaves.erase(iter);
  }

  // Set the cost of the new node based on the previous node plus lp info.
  CfgType& nearest = g->GetVertex(_nearestVID);
  CfgType& newNode = g->GetVertex(_newVID);

  // Cost of new node is the cost from root to parent plus cost from parent to
  // new node.
  double cost = nearest.GetStat("cost");
  if(this->GetTask()->GetRobot()->IsNonholonomic())
    cost += _lp.m_edge.second.GetTimeSteps();
  else
    cost += _lp.m_edge.second.GetWeight();
  newNode.SetStat("cost", cost);

  // Find the nearest witness to newNode and the separation distance.
  const auto witnessPair = FindNearestWitness(newNode);

  const VID witness = witnessPair.first;
  const double distance = witnessPair.second;

  // If nearest witness is invalid or more than m_witnessRadius away from
  // newNode, then newNode is a new witness and its own representative.
  if(witness == INVALID_VID or distance > m_witnessRadius) {
    m_witnesses.push_back(_newVID);
    m_active.insert(_newVID);
    m_representatives[_newVID] = _newVID;

    if(this->m_debug)
      std::cout << "\tAdding new witness node " << _newVID
                << " with cost " << cost << "."
                << std::endl;
    return;
  }

  // Otherwise we need to see if this node is a better representative for the
  // nearby witness.
  const VID representative = m_representatives[witness];
  const double representativeCost = g->GetVertex(representative).GetStat("cost");

  // If the new node isn't better, it's an inactive leaf.
  if(cost >= representativeCost) {
    if(this->m_debug)
      std::cout << "\tNew node " << _newVID << " with cost "
                << cost << " >= " << representativeCost
                << " not better than previous representative " << representative
                << "." << std::endl;
    m_inactiveLeaves.push_back(_newVID);
    return;
  }

  // The new node is a better representative.
  auto cit = find(m_active.begin(), m_active.end(), representative);
  if(cit == m_active.end())
    throw RunTimeException(WHERE, "Could not find representative '"
        + std::to_string(representative) + "' in the active set.");

  m_active.erase(cit);
  m_active.insert(_newVID);
  m_representatives[witness] = _newVID;

  // Prune leaf nodes that are inactive.
  if(m_prune)
    PruneInactiveLeaves();
}



template <typename MPTraits>
pair<typename StableSparseRRT<MPTraits>::VID, double>
StableSparseRRT<MPTraits>::
FindNearestWitness(const CfgType& _cfg) {
  MethodTimer mt(this->GetStatClass(), "SST::FindNearestWitness");

  /// @TODO This should use the 'nearest' nf.
  const auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);

  vector<pair<VID, double>> witnessNeighbors;

  nf->FindNeighbors(this->GetRoadmap(), m_witnesses.begin(), m_witnesses.end(),
      false, _cfg, back_inserter(witnessNeighbors));

  if(this->m_debug)
    std::cout << "Found nearest witness " << witnessNeighbors.begin()->first
              << " at a distance of "
              << std::setprecision(4) << witnessNeighbors.begin()->second
              << " / " << m_witnessRadius
              << " units."
              << endl;

  return *(witnessNeighbors.begin());
}


template <typename MPTraits>
void
StableSparseRRT<MPTraits>::
PruneInactiveLeaves() {
  MethodTimer mt(this->GetStatClass(), "SST::PruneInactiveLeaves");
  auto g = this->GetRoadmap()->GetGraph();

  // Prune the inactive leaves until there are none.
  while(!m_inactiveLeaves.empty()) {
    // Get the next leaf.
    auto leaf = m_inactiveLeaves.end() - 1;

    // Find parent.
    const VID parent = m_parent[*leaf];

    // Remove leaf.
    g->DeleteVertex(*leaf);
    m_inactiveLeaves.erase(leaf);

    // If parent is now an inactive leaf, add it to m_inactiveLeaves.
    if(g->get_out_degree(parent) == 0) {
      auto iter = std::find(m_active.begin(), m_active.end(), parent);
      const bool inactive = iter == m_active.end();
      if(inactive)
        m_inactiveLeaves.push_back(parent);
    }
  }
}

/*----------------------------------------------------------------------------*/

#endif
