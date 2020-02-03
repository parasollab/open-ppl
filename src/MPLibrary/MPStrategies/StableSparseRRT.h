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
/// A set of 'witness' configurations creates the sparseness criteria. Each
/// witness defines a ball region in state space, and the tree should grow
/// from only the lowest-cost vertex in each witness region. These vertices
/// form the 'active set', while the other vertices are 'inactive'. The latter
/// aren't used in neighborhood finding or extension, and should be pruned
/// away.
///
/// @note The 'witnesss configurations' here are are different from the witness
///       configurations used elsewhere in PMPL.
///
/// @note This method currently measures cost with distance in holonomic
///       problems and time steps in nonholonomic problems.
///
/// @note If we re-create an existing node, the original algorithm would keep
///       both copies and mark the more expensive one as inactive. This would
///       also cause the subtree rooted at the more expensive node to be
///       sub-optimal w.r.t. the best known cost to this state. Our roadmap
///       graph however does not permit duplicate states, so in our case the
///       new path will be discarded. It would be nice to save it if cheaper and
///       rewire its parent, but that wouldn't be exactly SST, and would also
///       require propogating cost changes to the children (which could also
///       require updates to the representatives for their witnesses). However
///       it is very rare to recreate the same exact state via a different local
///       plan, so the main way this matters in our usage is for the goal
///       states: if the goal is a single point in state space, our goal-biasing
///       heuristics will frequently recreate it from other parents, sometimes
///       with a cheaper plan. If we don't do something to take the cheaper
///       plan, we'll never see the cost decrease when running past the first
///       feasible solution. The easy way to avoid the problem is to use
///       non-point goals for this method so that the generated goal states are
///       slightly different (i.e. use a small sphere instead of a point).
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
    typedef typename RoadmapType::VertexSet VertexSet;

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
        const VertexSet* const _candidates = nullptr) override;

    /// SST selects the neighbor with best path cost rather than closest
    /// distance.
    virtual Neighbor SelectNeighbor(const CfgType& _cfg,
        const std::vector<Neighbor>& _neighbors) override;

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
    /// @return True if the new node is active.
    bool UpdateSSTStructures(const VID _nearestVID, const VID _newVID,
        const LPOutput<MPTraits>& _lp);

    /// Add a new witness node and track its representative.
    /// @param _roadmapVID The node's VID in the roadmap.
    void AddNewWitness(const VID _roadmapVID);

    /// Update the representative for a witness.
    /// @param _witness The witness's VID in the witness map.
    /// @param _representative The new representative's VID in the roadmap.
    void UpdateRepresentative(const VID _witness, const VID _representative);

    // Find nearest witness to a vertex.
    Neighbor FindNearestWitness(const CfgType& _newCfg);

    /// Recursively prune the set of inactive leaves until none remain.
    void PruneInactiveLeaves();

    /// Find the best-cost VID for each goal so far.
    VertexSet GetBestGoalVIDs();

    ///@}
    ///@name Internal State
    ///@{

    /// The active set represents the 'sparse' tree.
    VertexSet m_active;
    /// Set of witnesses for enforcing sparseness.
    VertexSet m_witnesses;
    /// The inactive nodes with no children.
    VertexSet m_inactiveLeaves;
    /// Maps witness nodes to the best-cost representative for their region.
    std::unordered_map<VID, VID> m_representatives;

    std::string m_witnessNfLabel; ///< NF to use for finding witness nodes.

    /// Storage for witnessess. They must be stored separately from the main
    /// roadmap because they may be pruned from the tree.
    std::map<RoadmapType*, RoadmapType> m_witnessMaps;

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
  m_representatives.clear();
  m_witnessMaps.clear();

  if(this->m_debug)
    std::cout << "Initializing StableSparseRRT" << std::endl;

  // Issue a warning if we aren't using a radius NF.
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  if(nf->GetType() != NeighborhoodFinderMethod<MPTraits>::Type::RADIUS)
    std::cout << "Warning: neighborhood finder '" << this->m_nfLabel
              << "' is not of type 'RADIUS'. A radius-based NF is needed "
              << "to make SST's proofs work out."
              << std::endl;

  // Find the start node.
  auto goalTracker = this->GetGoalTracker();
  const auto& startVIDs = goalTracker->GetStartVIDs();
  if(startVIDs.empty())
    throw RunTimeException(WHERE) << "A start VID is required for this method.";
  const VID start = *startVIDs.begin();

  // We need to check each of the start nodes for active/witnesss/rep status to
  // make SST work with multiple roots.
  if(startVIDs.size() > 1)
    throw RunTimeException(WHERE) << "SST needs modification to support multiple "
                                  << "start nodes (" << startVIDs.size()
                                  << " detected)."
                                  << std::endl;

  // Add the start node to the SST structures.
  AddNewWitness(start);
}

/*----------------------- BasicRRTStrategy Overrides -------------------------*/

template <typename MPTraits>
typename StableSparseRRT<MPTraits>::VID
StableSparseRRT<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const VertexSet* const _candidates) {
  MethodTimer mt(this->GetStatClass(), "SST::FindNearestNeighbor");

  // If we are not interested in a particular candidate set, search the entire
  // active set for neighbors.
  if(!_candidates)
    return BasicRRTStrategy<MPTraits>::FindNearestNeighbor(_cfg, &m_active);

  // Otherwise we need to select only nodes that are both active and in the
  // candidate set.
  const VertexSet activeCandidates = VertexSetIntersection(m_active,
      *_candidates);

  // Ensure we found some active candidates.
  if(activeCandidates.empty())
    throw RunTimeException(WHERE) << "SST can't find any active candidates.";

  // Search only the active candidates using a radius NF.
  return BasicRRTStrategy<MPTraits>::FindNearestNeighbor(_cfg, &activeCandidates);
}


template <typename MPTraits>
Neighbor
StableSparseRRT<MPTraits>::
SelectNeighbor(const CfgType& _cfg, const std::vector<Neighbor>& _neighbors) {
  auto g = this->GetRoadmap();

  // Select the node with the best path cost.
  Neighbor best = _neighbors[0];
  double bestCost = g->GetVertex(best.target).GetStat("cost");
  for(const auto& n : _neighbors) {
    // Skip if this neighbor isn't better than the best.
    const double pathCost = g->GetVertex(n.target).GetStat("cost");
    if(pathCost >= bestCost)
      continue;

    best = n;
    bestCost = pathCost;
  }

  return best;
}


template <typename MPTraits>
typename StableSparseRRT<MPTraits>::VID
StableSparseRRT<MPTraits>::
Extend(const VID _nearest, const CfgType& _target, LPOutput<MPTraits>& _lp,
    const bool _requireNew) {
  /// @todo Add support for connecting extensions to enable bi-directional SST
  ///       and more options for path refinement. This also requires managing
  ///       the updates to path cost that occur when trees merge.
  const bool connecting = !_requireNew;
  if(connecting)
    throw RunTimeException(WHERE) << "SST can't yet support connection because "
                                  << "reconnecting to an existing node would "
                                  << "require updating all path costs."
                                  << std::endl;

  // Extend using the basic RRT method.
  const VID newVID = BasicRRTStrategy<MPTraits>::Extend(_nearest, _target, _lp,
      _requireNew);

  // If we succeeded, update the SST structures before we return.
  const bool success = newVID != INVALID_VID;
  if(success) {
    // If the node isn't active, then we will have deleted it if pruning is
    // used. Do not return a valid VID all cases to keep behavior the same
    // regardless of pruning setting.
    const bool active = UpdateSSTStructures(_nearest, newVID, _lp);
    if(!active)
      return INVALID_VID;
  }

  return newVID;
}

/*----------------------------- SST Functions --------------------------------*/

template <typename MPTraits>
bool
StableSparseRRT<MPTraits>::
UpdateSSTStructures(const VID _nearestVID, const VID _newVID,
    const LPOutput<MPTraits>& _lp) {
  MethodTimer mt(this->GetStatClass(), "SST::UpdateSSTStructures");

  auto g = this->GetRoadmap();

  // If the parent node was an inactive leaf, it isn't any longer.
  m_inactiveLeaves.erase(_nearestVID);

  // Set the cost of the new node based on the previous node plus lp info.
  const CfgType& nearest = g->GetVertex(_nearestVID);
  CfgType& newNode = g->GetVertex(_newVID);

  // Cost of new node is the cost from root to parent plus cost from parent to
  // new node.
  /// @todo Change this to only edge weights, nonholonomic extenders/local
  ///       planners should set the weight using desired cost (such as time).
  double cost = nearest.GetStat("cost");
  if(this->GetTask()->GetRobot()->IsNonholonomic())
    cost += _lp.m_edge.first.GetTimeSteps();
  else
    cost += _lp.m_edge.first.GetWeight();
  newNode.SetStat("cost", cost);

  // Find the nearest witness to newNode and the separation distance.
  const auto   witnessPair = FindNearestWitness(newNode);
  const VID    witness     = witnessPair.target;
  const double distance    = witnessPair.distance;

  // If nearest witness is invalid or more than m_witnessRadius away from
  // newNode, then newNode is a new witness and its own representative.
  if(witness == INVALID_VID or distance > m_witnessRadius) {
    AddNewWitness(_newVID);
    return true;
  }

  // Otherwise we need to see if this node is a better representative for the
  // nearby witness.
  const VID    representative     = m_representatives[witness];
  const double representativeCost = g->GetVertex(representative).GetStat("cost");

  // If the new node isn't better, it's an inactive leaf.
  const bool betterCost = cost < representativeCost;

  if(this->m_debug)
    std::cout << "\tNew node " << _newVID << " with cost "
              << cost << (betterCost ? " < " : " >= " ) << representativeCost
              << (betterCost ? "" : " not")
              << " better than previous representative " << representative
              << "." << std::endl;

  if(!betterCost)
    m_inactiveLeaves.insert(_newVID);
  else
    UpdateRepresentative(witness, _newVID);

  // Prune leaf nodes that are inactive.
  if(m_prune)
    PruneInactiveLeaves();

  return betterCost;
}


template <typename MPTraits>
void
StableSparseRRT<MPTraits>::
AddNewWitness(const VID _roadmapVID) {
  // Find the witness map for the current roadmap.
  auto r = this->GetRoadmap();
  auto iter = m_witnessMaps.find(r);
  if(iter == m_witnessMaps.end()) {
    auto pair = m_witnessMaps.emplace(r, RoadmapType(r->GetRobot()));
    iter = pair.first;
  }
  RoadmapType& witnessMap = iter->second;

  // Add the witness configuration to the witness map.
  const CfgType& cfg = r->GetVertex(_roadmapVID);
  const VID witnessMapVID = witnessMap.AddVertex(cfg);
  m_witnesses.insert(witnessMapVID);

  if(this->m_debug)
    std::cout << "\tAdding new witness node " << witnessMapVID
              << std::endl;

  // Update the representative.
  UpdateRepresentative(witnessMapVID, _roadmapVID);
}


template <typename MPTraits>
void
StableSparseRRT<MPTraits>::
UpdateRepresentative(const VID _witness, const VID _representative) {
  if(this->m_debug)
    std::cout << "\tSetting representative for witness node " << _witness
              << " to " << _representative
              << " with cost "
              << this->GetRoadmap()->GetVertex(_representative).GetStat("cost")
              << std::endl;

  // Add the new representative to the active set.
  m_active.insert(_representative);

  // If there isn't a representative for this witness yet, create a new entry.
  auto iter = m_representatives.find(_witness);
  const bool exists = iter != m_representatives.end();
  if(!exists) {
    m_representatives[_witness] = _representative;
    return;
  }

  // There is already a representative for this witness: we must erase it
  // from the active set.
  const VID oldRepresentative = iter->second;
  m_active.erase(iter->second);
  iter->second = _representative;

  // If the old representative has no children, it is now an inactive leaf.
  auto vi = this->GetRoadmap()->find_vertex(oldRepresentative);
  const bool noChildren = vi->begin() == vi->end();
  if(noChildren)
    m_inactiveLeaves.insert(oldRepresentative);
}


template <typename MPTraits>
Neighbor
StableSparseRRT<MPTraits>::
FindNearestWitness(const CfgType& _cfg) {
  MethodTimer mt(this->GetStatClass(), "SST::FindNearestWitness");

  std::vector<Neighbor> witnessNeighbors;

  // Search with the witness NF.
  const auto nf = this->GetNeighborhoodFinder(this->m_witnessNfLabel);
  RoadmapType& witnessMap = m_witnessMaps.at(this->GetRoadmap());
  nf->FindNeighbors(&witnessMap, _cfg, m_witnesses,
      std::back_inserter(witnessNeighbors));

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

  if(this->m_debug)
    std::cout << "Pruning tree, " << m_inactiveLeaves.size()
              << " inactive leaves."
              << std::endl;

  // We need to make sure we don't prune the best goal nodes. These could be
  // inactive leaves if there is a lower-cost node in their witness region, but
  // we obviously want to keep them so that we still have a valid solution to
  // our problem.
  const VertexSet goals = GetBestGoalVIDs();
  VertexSet leafGoals;

  // Prune the inactive leaves until there are none.
  size_t count = 0;
  while(!m_inactiveLeaves.empty()) {
    ++count;

    // Get the next leaf.
    auto leaf = m_inactiveLeaves.begin();
    const VID leafVID = *leaf;
    m_inactiveLeaves.erase(leaf);

    // If this leaf is a best goal, don't prune it.
    if(goals.count(leafVID)) {
      leafGoals.insert(leafVID);
      if(this->m_debug)
        std::cout << "\tNot pruning inactive leaf " << leafVID
                  << " which is a best goal node."
                  << std::endl;
      continue;
    }

    // Find parent.
    const VertexSet& predecessors = g->GetPredecessors(leafVID);
    switch(predecessors.size()) {
      case 0:
        throw RunTimeException(WHERE) << "Node " << leafVID << " is a root.";
      case 1:
        break;
      default:
        throw RunTimeException(WHERE) << "Node " << leafVID << " has "
                                      << predecessors.size() << " parents. "
                                      << "Graph is not a tree.";
    }
    const VID parent = *predecessors.begin();

    // Remove leaf.
    g->DeleteVertex(leafVID);
    if(this->m_debug)
      std::cout << "\tPruned inactive leaf " << leafVID << "." << std::endl;

    // If parent is now an inactive leaf, add it to m_inactiveLeaves.
    if(g->get_out_degree(parent) == 0) {
      const bool active = m_active.count(parent);
      if(!active)
        m_inactiveLeaves.insert(parent);
      if(this->m_debug)
        std::cout << "\t  Parent node '" << parent << "' is a leaf and "
                  << (active ? "" : "in") << "active."
                  << std::endl;
    }
    else if(this->m_debug)
      std::cout << "\t  Parent node '" << parent << "' has other children."
                << std::endl;
  }

  // If we skipped pruning any leaf goals, add those back to the inactive leaves
  // now.
  m_inactiveLeaves.insert(leafGoals.begin(), leafGoals.end());

  if(this->m_debug)
    std::cout << "Pruned " << count << " vertices." << std::endl;
}


template <typename MPTraits>
typename MPTraits::RoadmapType::VertexSet
StableSparseRRT<MPTraits>::
GetBestGoalVIDs() {
  auto goalTracker = this->GetGoalTracker();
  const size_t numGoals = this->GetTask()->GetNumGoals();
  auto r = this->GetRoadmap();

  VertexSet bestGoalVIDs;

  // Check each goal.
  for(size_t i = 0; i < numGoals; ++i) {
    // Get the VIDs that satisfy this goal. If there aren't any, move on.
    const VertexSet& goals = goalTracker->GetGoalVIDs(i);
    if(goals.empty())
      continue;

    // Find the best-cost VID.
    VID bestVID = INVALID_VID;
    double bestCost = std::numeric_limits<double>::infinity();
    for(const VID vid : goals) {
      // Get cost.
      const double cost = r->GetVertex(vid).GetStat("cost");;

      // Skip if not better.
      if(cost >= bestCost)
        continue;

      // This cost is better.
      bestCost = cost;
      bestVID = vid;
    }

    // Add the best VID to the output set.
    bestGoalVIDs.insert(bestVID);
  }

  return bestGoalVIDs;
}

/*----------------------------------------------------------------------------*/

#endif
