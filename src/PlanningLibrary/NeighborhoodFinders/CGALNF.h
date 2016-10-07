#ifndef CGALNF_H_
#define CGALNF_H_

#include <unordered_map>

#include <CGAL/Cartesian_d.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

#include "NeighborhoodFinderMethod.h"

typedef CGAL::Cartesian_d<double> Kernel;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class PMPLPointD : public Kernel::Point_d {
  public:
    PMPLPointD() : Kernel::Point_d(), m_it(-1) {}

    template<typename InputIterator>
      PMPLPointD (size_t d, InputIterator first, InputIterator last) :
        Kernel::Point_d(d, first, last), m_it(-1) {}

    template<typename InputIterator>
      PMPLPointD (unsigned long long _it, size_t d,
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
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class CGALNF : public NeighborhoodFinderMethod<MPTraits> {
  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    CGALNF(string _dmLabel = "", bool _unconnected = false, size_t _k = 5,
        double _epsilon = 0.0, bool _useScaling = false);
    CGALNF(MPProblemType* _problem, XMLNode& _node);

    virtual ~CGALNF();

    virtual void Print(ostream& _os) const;

    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
          const CfgType& _cfg, OutputIterator _out);

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighborPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1,
          InputIterator _first2, InputIterator _last2,
          OutputIterator _out);

    template<typename InputIterator>
      void UpdateInternalModel(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, bool _fromFullRoadmap);

  private:
    double m_epsilon; // approximation
    int m_useScaling;
    unordered_map<RoadmapType*, pair<size_t, Tree>> m_trees;
    Tree* m_queryTree;
    Tree* m_tmpTree{nullptr};
    double m_maxBBXRange;
};

template<class MPTraits>
CGALNF<MPTraits>::
CGALNF(string _dmLabel, bool _unconnected, size_t _k,
    double _epsilon, bool _useScaling) :
  NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected),
  m_epsilon(_epsilon), m_useScaling(_useScaling) {
    this->SetName("CGALNF");
    this->m_nfType = K;
    this->m_k = _k;
  }

template<class MPTraits>
CGALNF<MPTraits>::
CGALNF(MPProblemType* _problem, XMLNode& _node) :
  NeighborhoodFinderMethod<MPTraits>(_problem, _node) {
    this->SetName("CGALNF");

    this->m_nfType = K;
    this->m_k = _node.Read("k", true, 5, 0, MAX_INT,
        "Number of neighbors to find");
    m_epsilon = _node.Read("epsilon", false, 0.0, 0.0, 100.0,
        "Epsilon value for CGAL");
    m_useScaling = _node.Read("useScaling", false, false,
        "Bounding-box scaling used on pos DOFs");

    shared_ptr<Boundary> b = _problem->GetEnvironment()->GetBoundary();
    m_maxBBXRange = b->GetMaxDist();
  }

template<class MPTraits>
CGALNF<MPTraits>::
~CGALNF() {
  delete m_tmpTree;
}

template<class MPTraits>
void
CGALNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "epsilon: " << m_epsilon << " "
    << "useScaling: " << m_useScaling << " ";
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
CGALNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {

  this->StartTotalTime();

  this->StartConstructionTime();
  UpdateInternalModel(_rmp, _first, _last, _fromFullRoadmap);
  this->EndConstructionTime();

  this->IncrementNumQueries();

  size_t dim = _cfg.DOF();

  // insert scaled query (copy of original CFG)
  vector<double> queryCfg = _cfg.GetData();
  if(m_useScaling) {
    //normalize x,y,z components to [0,1]
    queryCfg[0] /= m_maxBBXRange;
    queryCfg[1] /= m_maxBBXRange;
    queryCfg[2] /= m_maxBBXRange;
  }
  PointD query(dim, queryCfg.begin(), queryCfg.end());

  this->StartQueryTime();
  NeighborSearch search(*m_queryTree, query, this->m_k + 1, m_epsilon);
  this->EndQueryTime();

  GraphType* map = _rmp->GetGraph();
  for(auto n : search) {
    VID vid = n.first.m_it;
    CfgRef node = map->GetVertex(vid);
    if(node == _cfg)
      continue;
    auto dmm = this->GetDistanceMetric(this->m_dmLabel);
    double dist = dmm->Distance(_cfg, node);
    *_out++ = make_pair(vid, dist);
  }

  this->EndTotalTime();
  return _out;
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
CGALNF<MPTraits>::
FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw RunTimeException(WHERE, "FindNeighborPairs is not yet implemented.");
}

template<class MPTraits>
template<typename InputIterator>
void
CGALNF<MPTraits>::
UpdateInternalModel(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap) {

  GraphType* map = _rmp->GetGraph();

  if(!_fromFullRoadmap) {
    delete m_tmpTree;
    m_tmpTree = new Tree();
    InputIterator V1;
    for(V1 = _first; V1 != _last; ++V1){
      CfgRef node = map->GetVertex(V1);
      m_tmpTree->insert(PointD(map->GetVID(V1), node.DOF(), node.GetData().begin(), node.GetData().end()));
    }
    m_queryTree = m_tmpTree;
  }
  else {
    if(m_trees.count(_rmp) == 0)
      m_trees[_rmp].first = -1;

    typedef typename GraphType::RoadmapVCSType RoadmapVCSType;

    const RoadmapVCSType& rvcs = map->GetRoadmapVCS();

    size_t newVersion = rvcs.GetVersionNumber();
    if(m_trees[_rmp].first == newVersion)
      return;

    typename RoadmapVCSType::const_iterator start;
    if(m_trees[_rmp].first == static_cast<size_t>(-1))
      start = rvcs.begin();
    else
      start = rvcs.IteratorAt(m_trees[_rmp].first);

    typename RoadmapVCSType::const_iterator end = rvcs.end();
    typename RoadmapVCSType::const_iterator iter;

    for(iter = start; iter != end; iter++) {
      if((*iter).second.IsTypeAddVertex()) {

        VID vidToAdd = (*iter).second.GetAddVertexEvent()->GetVID();

        // scale roadmap CFGs
        CfgType cfgToAdd = map->GetVertex(vidToAdd);

        if (m_useScaling) {
          cfgToAdd[0] /= m_maxBBXRange;
          cfgToAdd[1] /= m_maxBBXRange;
          cfgToAdd[2] /= m_maxBBXRange;
        }

        m_trees[_rmp].second.insert(
            PointD(vidToAdd, cfgToAdd.DOF(),
              cfgToAdd.GetData().begin(), cfgToAdd.GetData().end())
            );
      }
    }

    m_trees[_rmp].first = newVersion;

    //should not be included if using the roadmap version
    m_queryTree = &m_trees[_rmp].second;
  }
}

#endif
