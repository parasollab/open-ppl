#ifndef KdTreeNF_H_
#define KdTreeNF_H_

#include <unordered_map>

#include <CGAL/Cartesian_d.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

#include "NeighborhoodFinderMethod.h"

typedef CGAL::Cartesian_d<double> Kernel;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
///
////////////////////////////////////////////////////////////////////////////////
class PMPLPointD : public Kernel::Point_d {
  public:
    PMPLPointD() : Kernel::Point_d(), m_it(-1) {}

    template <typename InputIterator>
      PMPLPointD(size_t d, InputIterator first, InputIterator last) :
        Kernel::Point_d(d, first, last), m_it(-1) {}

    template <typename InputIterator>
      PMPLPointD(unsigned long long _it, size_t d,
          InputIterator first, InputIterator last) :
        Kernel::Point_d(d, first, last), m_it(_it) {}

    unsigned long long m_it;
};

typedef CGAL::Search_traits<Kernel::FT,
        PMPLPointD,
        Kernel::Cartesian_const_iterator_d,
        Kernel::Construct_cartesian_const_iterator_d
        > TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> NeighborSearch;
typedef NeighborSearch::Tree Tree;
typedef PMPLPointD PointD;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinders
///
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class KdTreeNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename RoadmapType::GraphType     GraphType;
    typedef typename GraphType::VI              VI;
    typedef typename RoadmapType::VID           VID;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType     GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;

    ///@}
    ///@name Construction
    ///@{

    KdTreeNF(XMLNode& _node);

    // TODO: Keep this constructor?
    KdTreeNF(string _dmLabel = "", bool _unconnected = false, size_t _k = 5,
        double _epsilon = 0.0, bool _useScaling = false);

    virtual ~KdTreeNF();

    ///@}
    ///@name MPBaseObject OVerrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(ostream& _os) const;

    ///@}
    ///@name NeighborhoodFinder Functions
    ///@{

    template <typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
          const CfgType& _cfg, OutputIterator _out);

    /// K-closest that operate over two ranges of VIDS. K total pair<VID,VID>
    /// are returned that represent the k-closest pairs of VIDs between the two
    /// ranges.
    template <typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighborPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1,
          InputIterator _first2, InputIterator _last2,
          OutputIterator _out) {
        throw RunTimeException(WHERE, "FindNeighborPairs is not yet implemented.");
      }

    /// Group overloads
    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out) {
      throw RunTimeException(WHERE, "Not Supported for groups!");
    }

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(GroupRoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out) {
      throw RunTimeException(WHERE, "Not Supported for groups!");
    }

    ///@}

  private:

    ///@name Update Function
    ///@{

    template <typename InputIterator>
      void UpdateInternalModel(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, bool _fromFullRoadmap);

    ///@}

    // Buffer of added VIDs. Used to update roadmap before NF is called.
    vector<VID> m_added;

    // Same as above, but for deleted VIDs
    vector<VID> m_deleted;

    double m_epsilon{0.}; // approximation
    bool m_useScaling{false};

    unordered_map<RoadmapType*, Tree> m_trees;

    Tree* m_queryTree{nullptr};
    Tree* m_tmpTree{nullptr};
    double m_maxBBXRange{0.};
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
KdTreeNF<MPTraits>::
KdTreeNF(XMLNode& _node)
    : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("KdTreeNF");

  this->m_nfType = Type::K;
  this->m_k = _node.Read("k", true, 5, 0, MAX_INT,
      "Number of neighbors to find");
  m_epsilon = _node.Read("epsilon", false, 0.0, 0.0, 100.0,
      "Epsilon value for CGAL");
  m_useScaling = _node.Read("useScaling", false, m_useScaling,
      "Bounding-box scaling used on pos DOFs");
}


template <typename MPTraits>
KdTreeNF<MPTraits>::
KdTreeNF(string _dmLabel, bool _unconnected, size_t _k, double _epsilon,
    bool _useScaling)
    : NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected),
    m_epsilon(_epsilon), m_useScaling(_useScaling) {
  this->SetName("KdTreeNF");
  this->m_nfType = Type::K;
  this->m_k = _k;
}


template <typename MPTraits>
KdTreeNF<MPTraits>::
~KdTreeNF() {
  delete m_tmpTree;
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
KdTreeNF<MPTraits>::
Initialize() {
  m_trees.clear();
  m_added.clear();
  m_deleted.clear();

  const auto boundary = this->GetEnvironment()->GetBoundary();
  m_maxBBXRange = boundary->GetMaxDist();

  // Each time we add or remove a vertex, store it in a buffer used to update
  // the roadmap when the NF is called.
  auto g = this->GetRoadmap()->GetGraph();
  g->InstallHook(GraphType::HookType::AddVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->m_added.push_back(_vi->descriptor());});

  g->InstallHook(GraphType::HookType::DeleteVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->m_deleted.push_back(_vi->descriptor());});
}


template <typename MPTraits>
void
KdTreeNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tepsilon: " << m_epsilon
      << "\n\tuseScaling: " << m_useScaling
      << std::endl;
}

/*----------------------- NeighborhoodFinder Functions -----------------------*/

template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
KdTreeNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {

  // First, check for roadmap changes and update the Kd tree model.
  UpdateInternalModel(_rmp, _first, _last, _fromFullRoadmap);

  size_t dim = _cfg.DOF();

  // Insert scaled query (copy of original CFG).
  vector<double> queryCfg = _cfg.GetData();
  if(m_useScaling)
    for(size_t i = 0; i < _cfg.PosDOF(); ++i)
      queryCfg[i] /= m_maxBBXRange;

  PointD query(dim, queryCfg.begin(), queryCfg.end());

  NeighborSearch search(*m_queryTree, query, this->m_k, m_epsilon);

  auto g = _rmp->GetGraph();

  for(auto n : search) {
    VID vid = n.first.m_it;
    auto& node = g->GetVertex(vid);
    if(node == _cfg)
      continue;
    auto dmm = this->GetDistanceMetric(this->m_dmLabel);
    double dist = dmm->Distance(_cfg, node);
    *_out++ = Neighbor(vid, dist);
  }

  return _out;
}

/*----------------------------- Update Function ------------------------------*/

template <typename MPTraits>
template <typename InputIterator>
void
KdTreeNF<MPTraits>::
UpdateInternalModel(RoadmapType* _rmp, InputIterator _first,
    InputIterator _last, bool _fromFullRoadmap) {

  auto g = _rmp->GetGraph();

  if(!_fromFullRoadmap) {
    delete m_tmpTree;
    m_tmpTree = new Tree();

    for(auto vit = _first; vit != _last; ++vit) {
      auto& node = g->GetVertex(vit);
      m_tmpTree->insert(PointD(g->GetVID(vit), node.DOF(),
            node.GetData().begin(), node.GetData().end()));
    }

    m_queryTree = m_tmpTree;
  }
  else {
    // If the hasn't changed then we don't need to update the Kd tree model.
    if(m_added.empty() and m_deleted.empty())
      return;

    // Update the model with newly added Cfgs.
    for(auto& vid : m_added) {
      auto& cfg = g->GetVertex(vid);
      vector<double> cfgData = cfg.GetData();

      if(m_useScaling)
        for(size_t i = 0; i < cfg.PosDOF(); ++i)
          cfgData[i] /= m_maxBBXRange;

      m_trees[_rmp].insert(PointD(vid, cfg.DOF(), cfgData.begin(),
            cfgData.end()));
    }

    m_added.clear();

    // TODO: Remove this exception and uncomment the below for loop when CGAL is
    // updated to v4.11 or later.
    if(!m_deleted.empty())
      throw RunTimeException(WHERE, "Cannote update Kd tree model with vertex "
          "removal as the CGAL v4.6 Kd_tree class does not support removals.");

    /*
    // Do the same for any deleted Cfgs.
    for(auto& vid : m_deleted) {
      auto cfg = g->GetVertex(vid);

      m_trees[_rmp].remove(PointD(vid, cfg.DOF(), cfg.GetData().begin(),
            cfg.GetData().end()), nullptr);
    }

    m_deleted.clear();
    */

    // Should not be included if using the roadmap version.
    m_queryTree = &m_trees[_rmp];
  }
}

/*----------------------------------------------------------------------------*/

#endif
