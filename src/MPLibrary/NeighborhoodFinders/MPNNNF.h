#ifndef PMPL_MPNN_NEIGHBORHOOD_FINDER_H_
#define PMPL_MPNN_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.h"
#include "DNN/multiann.h"

#include <functional>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// This is a specialized k-d tree approximate nearest-neighbor method for
/// motion planning problems.
///
/// This object uses roadmap hooks to track vertex additions and deletions. The
/// underlying k-d tree will not be updated until a query is requested (the
/// changes are buffered until then).
///
/// Reference:
///   Anna Yershova and Steven M. LaValle. "Improving Motion Planning
///   Algorithms by Efficient Nearest-Neighbor Searching." TRO 2007.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPNNNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::VI             VI;
    typedef typename RoadmapType::VID            VID;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    MPNNNF();

    MPNNNF(XMLNode& _node);

    virtual ~MPNNNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Interface
    ///@{

    template <typename InputIterator>
    void FindNeighbors(RoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template <typename InputIterator>
    void FindNeighbors(GroupRoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Update the internal k-d tree model if needed.
    void UpdateKdTree();

    /// Convert a cfg to the ANN equivalent, an ANNpoint.
    /// @param _cfg The configuration to convert.
    /// @return An ANN representation of _cfg.
    ANNpoint CfgToANNPoint(const CfgType& _cfg) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    std::vector<VID> m_added;    ///< The added vertices since last update.
    std::vector<VID> m_deleted;  ///< The deleted vertices since last update.

    /// Due to the way that MultiANN returns neighbors we need to keep track of
    /// vids added to the kd tree. When MultiANN returns indices of nearest
    /// neighbors we can access the corresponding vids through this vector.
    std::vector<VID> m_vids;

    double m_epsilon{0.};        ///< Approximateness used by internal kd tree

    /// MultiANN is an approximate nearest neighbor searching library which
    /// supports solving multiple nearest neighbor queries. This search is done
    /// over one of three data structures: kd tree, priority kd tree, or bbd tree.
    /// For our purposes the structure used is a kd tree.
    std::unique_ptr<MultiANN> m_kdTree{nullptr};

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
MPNNNF<MPTraits>::
MPNNNF() : NeighborhoodFinderMethod<MPTraits>() {
  this->SetName("MPNNNF");
  this->m_nfType = Type::K;
}


template <typename MPTraits>
MPNNNF<MPTraits>::
MPNNNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("MPNNNF");
  this->m_nfType = Type::K;
  this->m_k = _node.Read("k", true, 5, 0, MAX_INT, "Number of neighbors to find");
}


template <typename MPTraits>
MPNNNF<MPTraits>::
~MPNNNF() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
MPNNNF<MPTraits>::
Initialize() {
  m_added.clear();
  m_deleted.clear();
  m_vids.clear();

  // Create kd tree structure.
  const auto mb = this->GetTask()->GetRobot()->GetMultiBody();
  std::vector<int> topology(mb->DOF(), 1);
  auto scale = annAllocPt(topology.size(), 1.);

  m_kdTree.reset(new MultiANN((int) topology.size(), topology.data(), scale,
      m_epsilon));

  // As in KdTreeNF, adding or removing vertices triggers a hook which populates
  // the corresponding buffer.
  auto g = this->GetRoadmap();
  g->InstallHook(RoadmapType::HookType::AddVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->m_added.push_back(_vi->descriptor());});

  g->InstallHook(RoadmapType::HookType::DeleteVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->m_deleted.push_back(_vi->descriptor());});
}

template <typename MPTraits>
void
MPNNNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tk: " << this->m_k << endl;
}

/*----------------------- NeighborhoodFinder Interface -----------------------*/

template <typename MPTraits>
template <typename InputIterator>
void
MPNNNF<MPTraits>::
FindNeighbors(RoadmapType* _r, InputIterator _first, InputIterator _last,
    bool _fromFullRoadmap, const CfgType& _cfg, OutputIterator _out) {
  // First check for and make any updates to the internal kd tree.
  this->UpdateKdTree();

  // Perform k closest nieghbor search over kd tree.

  std::vector<Neighbor> neighbors;

  auto queryPoint = CfgToANNPoint(_cfg);

  // Arrays returned by NN call.

  // Distances, indices, and pointers of the k closest neighbors.
  auto distances = annAllocPt(this->GetK());
  int* indices = new int[this->GetK()];
  std::vector<CfgType*> pointers(this->GetK(), nullptr);

  m_kdTree->NearestNeighbor(queryPoint, this->GetK(), distances,
      indices, (void**) pointers.data());

  for(size_t i = 0; i < this->GetK(); ++i) {
    auto vid = m_vids.at(indices[i]);

    if(vid != INVALID_VID) {
      *_out = Neighbor(vid, distances[i]);
      ++_out;
    }
  }
}


template <typename MPTraits>
template <typename InputIterator>
void
MPNNNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
MPNNNF<MPTraits>::
UpdateKdTree() {
  // Check if any changes were made to the roadmap.
  if(m_added.empty() and m_deleted.empty())
    return;

  auto g = this->GetRoadmap();

  // Update the MultiAnn with newly added cfgs.
  for(auto& vid: m_added) {
    // Convert the cfg to an ANNpoint and add to the MultiANN structure.
    auto& cfg = g->GetVertex(vid);

    auto coordinates = CfgToANNPoint(cfg);
    m_kdTree->AddPoint(coordinates, &cfg);
    m_vids.push_back(vid);
  }

  m_added.clear();

  // Since MultiANN does not support point removal from the kd tree structure we
  // have to rebuild it if a vertex is removed from the roadmap.
  if(!m_deleted.empty()) {
    const auto mb = this->GetTask()->GetRobot()->GetMultiBody();
    std::vector<int> topology(mb->DOF(), 1);
    auto scale = annAllocPt(topology.size(), 1.);

    m_kdTree.reset(new MultiANN((int) topology.size(), topology.data(), scale,
          m_epsilon));

    for(auto vit = g->begin(); vit != g->end(); vit++) {
      auto coordinates = CfgToANNPoint(vit->property());
      m_kdTree->AddPoint(coordinates, &vit->property());
    }

    m_deleted.clear();
  }
}


template <typename MPTraits>
ANNpoint
MPNNNF<MPTraits>::
CfgToANNPoint(const CfgType& _cfg) const noexcept {
  auto point = annAllocPt(_cfg.DOF(), 0.);

  for(size_t i = 0; i < _cfg.DOF(); ++i)
    point[i] = _cfg[i];

  return point;
}

/*----------------------------------------------------------------------------*/

#endif
