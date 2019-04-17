#ifndef PMPL_STABLE_SPARSE_RRT_H_
#define PMPL_STABLE_SPARSE_RRT_H_

#include "BasicRRTStrategy.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>


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
/// Reference:
///   Yanbo Li, Zakary Littlefield, and Kostas Bekris. "Sparse Methods for
///   Efficient Asymptotically Optimal Kinodynamic Planning". Algorithmic
///   Foundations of Robotics XI (WAFR 2014). 2015. 263-282.
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

    using typename BasicRRTStrategy<MPTraits>::VertexSet;

    ///@}
    ///@name Construction
    ///@{

    StableSparseRRT();

    StableSparseRRT(XMLNode& _node);

    virtual ~StableSparseRRT() = default;

    ///@}

  private:

    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name BasicRRTStrategy Overrides
    ///@{

    /// SST's version of this function requires a radius NF and only searches
    /// the active set.
    virtual VID FindNearestNeighbor(const CfgType& _cfg,
        const VertexSet* const _tree = nullptr) override;

    /// SST's version of this function calls the base class version and then
    /// updates metadata specific to this method.
    virtual VID Extend(const VID _nearest, const CfgType& _target,
        LPOutput<MPTraits>& _lp, const bool _requireNew = true) override;

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
    Neighbor FindNearestWitness(const CfgType& _newCfg);

    /// Recursively prune the set of inactive leaves until none remain.
    void PruneInactiveLeaves();

    ///@}
    ///@name Internal State
    ///@{

    /// The active set represents the 'sparse' tree.
    VertexSet m_active;
    /// Set of witnesses for enforcing sparseness.
    std::vector<VID> m_witnesses;
    /// The inactive nodes with no children.
    VertexSet m_inactiveLeaves;
    /// A map from child VID to parent VID for each roadmap vertex.
    std::unordered_map<VID, VID> m_parent;
    /// Maps a representative node to a witness.
    std::unordered_map<VID, VID> m_representatives;

    std::string m_witnessNfLabel; ///< NF to use for finding witness nodes.

    double m_witnessRadius{.5}; ///< Distance between witness nodes.
    bool m_prune{false};

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
StableSparseRRT<MPTraits>::
StableSparseRRT() {
  this->SetName("StableSparseRRT");
}


template <typename MPTraits>
StableSparseRRT<MPTraits>::
StableSparseRRT(XMLNode& _node) : BasicRRTStrategy<MPTraits>(_node) {
  this->SetName("StableSparseRRT");

  m_witnessNfLabel = _node.Read("witnessNfLabel", false, this->m_nfLabel,
      "Optional different NF to use for witnesses.");
  m_witnessRadius = _node.Read("witnessRadius", true, m_witnessRadius, .01,
      std::numeric_limits<double>::max(), "Distance between witnesses.");
  m_prune = _node.Read("prune", false, m_prune,
      "Enable/disable pruning inactive leaves");
}

/*-------------------------- MPStrategy Overrides ----------------------------*/

template <typename MPTraits>
void
StableSparseRRT<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();

  // Clear SST structures.
  m_active.clear();
  m_witnesses.clear();
  m_inactiveLeaves.clear();
  m_parent.clear();
  m_representatives.clear();

  if(this->m_debug)
    std::cout << "Initializing StableSparseRRT" << std::endl;

  // Find the start node.
  auto goalTracker = this->GetGoalTracker();
  const auto& startVIDs = goalTracker->GetStartVIDs();
  if(startVIDs.empty())
    throw RunTimeException(WHERE) << "A start VID is required for this method.";
  const VID start = *startVIDs.begin();

  m_active.insert(start);
  m_witnesses.push_back(start);
  m_representatives.emplace(start, start);
}

/*----------------------- BasicRRTStrategy Overrides -------------------------*/

template <typename MPTraits>
typename StableSparseRRT<MPTraits>::VID
StableSparseRRT<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const VertexSet* const _tree) {
  /// @note This needs to use a radius-NF type to make the proofs work.
  MethodTimer mt(this->GetStatClass(), "SST::FindNearestNeighbor");
  auto g = this->GetRoadmap();
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);

  if(this->m_debug)
    std::cout << "Searching for nearest neighbors to " << _cfg.PrettyPrint()
              << " with '" << this->m_nfLabel << "' from "
              << (_tree ? "a tree of size " + std::to_string(_tree->size())
                        : "the full roadmap")
              << "."
              << std::endl;

  std::vector<Neighbor> neighbors;

  // If we are not interested in one particular tree, search the entire active
  // set for neighbors.
  if(!_tree) {
    nf->FindNeighbors(g, m_active.begin(), m_active.end(),
        m_active.size() == g->get_num_vertices(),
        _cfg, std::back_inserter(neighbors));
  }
  // Otherwise we need to select only nodes that are both active and in the
  // tree.
  else {
    // Find the intersection of the active set with the current tree. Usually
    // the three should be larger.
    std::vector<VID> candidates;
    candidates.reserve(std::min(_tree->size(), m_active.size()));
    std::copy_if(m_active.begin(), m_active.end(),
        std::back_inserter(candidates),
        [_tree](const VID _vid){return _tree->count(_vid);}
    );

    // Ensure we found some candidates.
    if(candidates.empty())
      throw RunTimeException(WHERE) << "SST can't find any candidates.";

    nf->FindNeighbors(g, candidates.begin(), candidates.end(),
        candidates.size() == g->get_num_vertices(),
        _cfg, std::back_inserter(neighbors));
  }

  // Search only the candidates using a radius NF.

  // Of the nodes in the radius, select the one with the best path cost.
  VID bestVID = INVALID_VID;
  double bestCost = std::numeric_limits<double>::max();
  for(const auto& n : neighbors) {
    // Check for invalid neighbors.
    if(n.target == INVALID_VID)
      throw RunTimeException(WHERE) << "NF should not return bogus nodes.";

    // Check if this neighbor has the best cost so far.
    const double cost = g->GetVertex(n.target).GetStat("cost");
    if(cost < bestCost) {
      bestCost = cost;
      bestVID = n.target;
    }
  }

  // Debug.
  if(this->m_debug and !neighbors.empty()) {
    const VID nearest = neighbors[0].target;
    std::cout << "\tFound best active neighbor " << bestVID
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
Extend(const VID _nearest, const CfgType& _target, LPOutput<MPTraits>& _lp,
    const bool _requireNew) {
  // Extend using the basic RRT method.
  const VID newVID = BasicRRTStrategy<MPTraits>::Extend(_nearest, _target, _lp,
      _requireNew);

  // If we succeeded, update the SST structures before we return.
  const bool success = newVID != INVALID_VID;
  if(success) {
    // Skip this if we are connecting trees and the node wasn't new.
    auto g = this->GetRoadmap();
    const bool connecting = !_requireNew,
               newNode    = newVID == g->GetLastVID(),
               update     = newNode or !connecting;
    if(update);
      UpdateSSTStructures(_nearest, newVID, _lp);
  }

  return newVID;
}

/*----------------------------- SST Functions --------------------------------*/

template <typename MPTraits>
void
StableSparseRRT<MPTraits>::
UpdateSSTStructures(const VID _nearestVID, const VID _newVID,
    const LPOutput<MPTraits>& _lp) {
  MethodTimer mt(this->GetStatClass(), "SST::UpdateSSTStructures");

  auto g = this->GetRoadmap();

  // The nearest node is now the parent of the new node.
  m_parent[_newVID] = _nearestVID;

  // If the parent node was an inactive leaf, it isn't any longer.
  m_inactiveLeaves.erase(_nearestVID);

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

  const VID witness = witnessPair.target;
  const double distance = witnessPair.distance;

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
    m_inactiveLeaves.insert(_newVID);
    return;
  }

  // The new node is a better representative.
  auto cit = m_active.find(representative);
  if(cit == m_active.end())
    throw RunTimeException(WHERE) << "Could not find representative '"
                                  << representative << "' in the active set.";

  m_active.erase(cit);
  m_active.insert(_newVID);
  m_representatives[witness] = _newVID;

  // Prune leaf nodes that are inactive.
  if(m_prune)
    PruneInactiveLeaves();
}



template <typename MPTraits>
Neighbor
StableSparseRRT<MPTraits>::
FindNearestWitness(const CfgType& _cfg) {
  MethodTimer mt(this->GetStatClass(), "SST::FindNearestWitness");

  std::vector<Neighbor> witnessNeighbors;

  // Search with the witness NF.
  const auto nf = this->GetNeighborhoodFinder(this->m_witnessNfLabel);
  nf->FindNeighbors(this->GetRoadmap(), m_witnesses.begin(), m_witnesses.end(),
      false, _cfg, std::back_inserter(witnessNeighbors));

  // If that failed, return a null result.
  if(witnessNeighbors.empty())
    return Neighbor();

  if(this->m_debug)
    std::cout << "Found nearest witness " << witnessNeighbors.begin()->target
              << " at a distance of "
              << std::setprecision(4) << witnessNeighbors.begin()->distance
              << " / " << m_witnessRadius
              << " units."
              << std::endl;

  return *(witnessNeighbors.begin());
}


template <typename MPTraits>
void
StableSparseRRT<MPTraits>::
PruneInactiveLeaves() {
  MethodTimer mt(this->GetStatClass(), "SST::PruneInactiveLeaves");
  auto g = this->GetRoadmap();

  // Prune the inactive leaves until there are none.
  while(!m_inactiveLeaves.empty()) {
    // Get the next leaf.
    auto leaf = m_inactiveLeaves.begin();

    // Find parent.
    const VID parent = m_parent[*leaf];

    // Remove leaf.
    g->DeleteVertex(*leaf);
    this->RemoveNodeFromTrees(*leaf);
    m_parent.erase(*leaf);
    m_inactiveLeaves.erase(leaf);

    // If parent is now an inactive leaf, add it to m_inactiveLeaves.
    if(g->get_out_degree(parent) == 0) {
      const bool active = m_active.count(parent);
      if(!active)
        m_inactiveLeaves.insert(parent);
    }
  }
}

/*----------------------------------------------------------------------------*/

#endif
