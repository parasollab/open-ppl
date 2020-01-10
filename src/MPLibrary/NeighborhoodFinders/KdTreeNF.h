#ifndef PMPL_KDTREE_NF_H_
#define PMPL_KDTREE_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <CGAL/Cartesian_d.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

#include <unordered_map>


////////////////////////////////////////////////////////////////////////////////
/// Uses a k-d tree to find nearest-neighbors.
///
/// This class builds a separate kd tree for each roadmap and each of its
/// components. This is important for avoiding a continuous rebuild from scratch
/// for any candidate set other than the full roadmap.
///
/// @todo We should be able to use a Search_traits_adaptor to avoid
///       recopying the points and instead put VIDs into the kd tree.
///       However my attempt to do that resulted in bogus runtime behavior
///       within CGAL's Kd_tree::build (dimension is computed incorrectly
///       resulting in a bad array size). Figure out how to finagle this so
///       that we can avoid recopying Cfgs.
///
/// @todo This is limited to euclidean distance at present, but it should be
///       possible to use any minkowski or mahalonobis distance with the
///       appropriate distance adapter. It can *not* support arbitrary distances
///       despite the claims in CGAL's documentation because the implementation
///       requires finding the nearest and furthest point to an AABB; this is
///       not tractible unless the geometrically nearest/farthest points (i.e.
///       measured with euclidean) are also the nearest points for our metric.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class KdTreeNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VI            VI;
    typedef typename RoadmapType::VID           VID;
    typedef typename RoadmapType::VertexSet     VertexSet;
    typedef typename RoadmapType::CCTrackerType CCTrackerType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType     GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

    ///@}

  private:

    ///@name Internal Types
    ///@{

    /// The CGAL kernel type for the k-d tree.
    typedef CGAL::Cartesian_d<double> Kernel;

    /// CGAL representation of a roadmap configuration within a k-d tree as a
    /// d-dimensional point.
    struct PointD : public Kernel::Point_d {

      PointD() : Kernel::Point_d() {}

      PointD(const CfgType& _cfg) : PointD(INVALID_VID, _cfg)
      {}

      PointD(const VID _vid, const CfgType& _cfg) :
          Kernel::Point_d(_cfg.DOF(), _cfg.GetData().begin(),
              _cfg.GetData().end()), vid(_vid)
      {}

      VID vid{INVALID_VID}; ///< The configuration's VID in the roadmap.

    };

    /// The traits for the k-d tree and search algorithm.
    typedef CGAL::Search_traits<Kernel::FT,               // Field type.
            PointD,                                       // Point type.
            Kernel::Cartesian_const_iterator_d,           // Coordinate iterator.
            Kernel::Construct_cartesian_const_iterator_d  // Constructs iterator.
            > TreeTraits;

    /// The k-d tree search algorithm type.
    typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> NeighborSearch;

    /// The k-d tree type.
    typedef typename NeighborSearch::Tree KdTree;

    /// A key which identifies a particular kd tree model. We will use one model
    /// for each full roadmap and one for each of its connected components.
    typedef std::pair<RoadmapType*, const VertexSet*> ModelKey;

    ///@}

  public:

    ///@name Construction
    ///@{

    KdTreeNF();

    KdTreeNF(XMLNode& _node);

    virtual ~KdTreeNF() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Functions
    ///@{

    virtual void FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out) override;

    virtual void FindNeighbors(GroupRoadmapType* const _r,
        const GroupCfgType& _cfg, const VertexSet& _candidates,
        OutputIterator _out) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Set up model(s) for a roadmap.
    void SetupModels(RoadmapType* const _r);

    /// Get the kd-tree model for this query.
    /// @param _r The roadmap.
    /// @param _candidates The candidate VIDs.
    /// @return The best available model for finding the nearest neighbors in
    ///         _candidates within _r.
    KdTree* GetModel(RoadmapType* const _r, const VertexSet& _candidates);

    ///@}
    ///@name Internal State
    ///@{

    double m_epsilon{0.};        ///< Fuzzy factor s.t. d = (1 + eps)d*

    std::unordered_map<ModelKey, KdTree> m_trees; ///< Build one tree per map/CC.

    std::unique_ptr<KdTree> m_tmpTree; ///< Temp tree for partial map queries.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
KdTreeNF<MPTraits>::
KdTreeNF() : NeighborhoodFinderMethod<MPTraits>(Type::K) {
  this->SetName("KdTreeNF");
}


template <typename MPTraits>
KdTreeNF<MPTraits>::
KdTreeNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node, Type::K) {
  this->SetName("KdTreeNF");

  m_epsilon = _node.Read("epsilon", false, m_epsilon, 0., 100.,
      "Epsilon value for CGAL");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
KdTreeNF<MPTraits>::
Initialize() {
  m_trees.clear();
  m_tmpTree.reset(new KdTree);

  // Force the dm to be a euclidean type because the kd tree will not be
  // consistent with it otherwise.
  if(this->GetDistanceMetric(this->m_dmLabel)->GetName() != "Euclidean")
    throw ParseException(WHERE) << "KdTreeNF requires a Euclidean distance "
                                << "metric to be consistent with the CGAL "
                                << "kd tree.";
}


template <typename MPTraits>
void
KdTreeNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tepsilon: " << m_epsilon
      << std::endl;
}

/*----------------------- NeighborhoodFinder Functions -----------------------*/

template <typename MPTraits>
void
KdTreeNF<MPTraits>::
FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  // Get the best Kd tree model.
  KdTree* const tree = GetModel(_r, _candidates);

  // Search for nearest neighbors.
  NeighborSearch search(*tree, _cfg, this->m_k, m_epsilon);

  auto dm = this->GetDistanceMetric(this->m_dmLabel);
  for(const auto& n : search) {
    // Get the VID.
    const VID vid = n.first.vid;

    // Check for self.
    const auto& node = _r->GetVertex(vid);
    if(node == _cfg)
      continue;

    // Compute distance and output neighbor.
    const double distance = dm->Distance(_cfg, node);
    _out = Neighbor(vid, distance);
  }
}


template <typename MPTraits>
void
KdTreeNF<MPTraits>::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
void
KdTreeNF<MPTraits>::
SetupModels(RoadmapType* const _r) {
  // Create a kd tree for this roadmap.
  auto* tree = &(m_trees[ModelKey{_r, &_r->GetAllVIDs()}]);

  // Create a kd tree for each of its connected components.
  auto* ccTracker = _r->GetCCTracker();
  for(const VertexSet* cc : *ccTracker)
    m_trees[ModelKey{_r, cc}];

  // Create a hook to add new CC tracker models.
  ccTracker->InstallCreatedHook(this->GetNameAndLabel(),
      [this, _r](const CCTrackerType* const, const VertexSet* const _cc) {
        this->m_trees[ModelKey{_r, _cc}];
      }
  );
  // Create a hook to remove CC tracker models.
  ccTracker->InstallDeletedHook(this->GetNameAndLabel(),
      [this, _r](const CCTrackerType* const, const VertexSet* const _cc) {
        this->m_trees.erase(ModelKey{_r, _cc});
      }
  );
  // Create a hook to merge CC tracker models.
  ccTracker->InstallMergedHook(this->GetNameAndLabel(),
      [this, _r](const CCTrackerType* const, const VertexSet* const _target,
          const VertexSet* const _source) {
        // Add all points in _source to the tree for _target.
        auto* targetTree = &this->m_trees.at(ModelKey{_r, _target});
        for(const VID vid : *_source)
          targetTree->insert(PointD(vid, _r->GetVertex(vid)));

        // Remove the tree for _source.
        this->m_trees.erase(ModelKey{_r, _source});
      }
  );
  // Create a hook to split CC tracker models.
  ccTracker->InstallBrokenHook(this->GetNameAndLabel(),
      [this, _r](const CCTrackerType* const, const VertexSet* const _source,
          const VertexSet* const _target) {
        // Create a new tree for the _target CC and move its points from the
        // _source tree to their new tree.
        auto* targetTree = &this->m_trees[ModelKey{_r, _target}],
            * sourceTree = &this->m_trees.at(ModelKey{_r, _source});
        for(const VID vid : *_target) {
          PointD p(vid, _r->GetVertex(vid));
          targetTree->insert(p);
          sourceTree->remove(p);
        }
      }
  );

  // Create a function factory for adding vertices to the kd tree.
  typename RoadmapType::VertexHook adder = [tree, _r, this](const VI _vi) {
        const VID vid   = _vi->descriptor();
        const auto& cfg = _vi->property();
        PointD p(vid, cfg);

        // Add vertex to the full roadmap tree.
        tree->insert(PointD(vid, cfg));

        // Add vertex to its CC tree.
        auto* ccTracker = _r->GetCCTracker();
        auto* cc        = &ccTracker->GetCC(vid);
        auto* ccTree    = &this->m_trees.at(ModelKey{_r, cc});
        ccTree->insert(p);
      };
  // Create a function factory for deleting vertices from the kd tree.
  typename RoadmapType::VertexHook deleter = [tree, _r, this](const VI _vi) {
        const VID vid   = _vi->descriptor();
        const auto& cfg = _vi->property();
        PointD p(vid, cfg);

        // Remove vertex from the full roadmap tree.
        tree->remove(p);

        // Remove vertex from its CC tree.
        auto* ccTracker = _r->GetCCTracker();
        auto* cc        = &ccTracker->GetCC(vid);
        auto* ccTree    = &this->m_trees.at(ModelKey{_r, cc});
        ccTree->remove(p);
      };

  // Add each existing vertex to the set.
  for(auto vi = _r->begin(); vi != _r->end(); ++vi)
    adder(vi);

  // Each time we add a vertex, add it to the kd tree (the CGAL implementation
  // is internally buffered).
  _r->InstallHook(RoadmapType::HookType::AddVertex, this->GetNameAndLabel(),
      adder);
  // Each time we remove a vertex, remove it from the kd tree.
  _r->InstallHook(RoadmapType::HookType::DeleteVertex, this->GetNameAndLabel(),
      deleter);

}


template <typename MPTraits>
typename KdTreeNF<MPTraits>::KdTree*
KdTreeNF<MPTraits>::
GetModel(RoadmapType* const _r, const VertexSet& _candidates) {
  // Set up models for this roadmap if needed.
  auto iter = m_trees.find(ModelKey{_r, &_r->GetAllVIDs()});
  if(iter == m_trees.end())
    SetupModels(_r);

  // If we already have a model for this roadmap/candidate set pair, use it.
  iter = m_trees.find(ModelKey{_r, &_candidates});
  if(iter != m_trees.end())
    return &iter->second;

  // Else we have no model. Build a temporary tree.
  /// @todo Questionable, why use O(n lg n) operation of building and
  ///       searching a new tree rather than brute force for O(n)? Somehow it
  ///       appears to provide better performance anyway, perhaps due to
  ///       CGAL's use of incremental orthogonal distance calculations?
  m_tmpTree->clear();

  for(const VID vid : _candidates)
    m_tmpTree->insert(PointD(vid, _r->GetVertex(vid)));

  return m_tmpTree.get();
}

/*----------------------------------------------------------------------------*/

#endif
