#ifndef PMPL_KDTREE_NF_H_
#define PMPL_KDTREE_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <CGAL/Cartesian_d.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

#include <map>


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
class KdTreeNF : virtual public NeighborhoodFinderMethod {

  public:

    ///@name Motion Planning Types
    ///@{
    typedef typename MPBaseObject::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VI            VI;
    typedef typename RoadmapType::VID           VID;
    typedef typename RoadmapType::VertexSet     VertexSet;
    typedef typename RoadmapType::CCTrackerType CCTrackerType;
    typedef typename MPBaseObject::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPBaseObject::GroupCfgType      GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod::Type;
    using typename NeighborhoodFinderMethod::OutputIterator;

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

      PointD(const Cfg& _cfg) : PointD(INVALID_VID, _cfg)
      {}

      PointD(const VID _vid, const Cfg& _cfg) :
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

    virtual void FindNeighbors(RoadmapType* const _r, const Cfg& _cfg,
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

    std::map<ModelKey, KdTree> m_trees; ///< Build one tree per map/CC.

    std::unique_ptr<KdTree> m_tmpTree; ///< Temp tree for partial map queries.

    ///@}

};

#endif
