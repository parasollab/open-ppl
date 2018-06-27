#ifndef DPESNF_H_
#define DPESNF_H_

#include <unordered_map>
#include "NeighborhoodFinderMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Distance-based Projection onto Euclidean Space (DPES).
/// Given a dimension \f$m\f$, project all input points onto
/// \f$\mathcal{R}^m\f$. At query time, project query configuration to the lower
/// dimensional space and use the Euclidean distance metric to determine
/// proximity.
///
/// From: Plaku, E. and Kavraki, L., "Quantitative Analysis of Nearest-Neighbors
/// Search in High-Dimensional Sampling-Based Motion Planning," In. Proc. of the
/// Seventh Workshop on the Algorithmic Foundations of Robotics (WAFR), Springer
/// Tracts in Advanced Robotics, 47, pp. 3-18.
///
/// Currently it is unclear how dynamic construction works in the paper. So to
/// incrementally build pivots, basically, the first m points are chosen.
///
/// @todo Abstract the underlying storage structure to allow for KD-tree or
///       other searches other than brute force searching.
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DPESNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename MPTraits::CfgType                CfgType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::GraphType           GraphType;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::vector<double> Projected; ///< Projected point of dim m

    ////////////////////////////////////////////////////////////////////////////
    /// Model for DPES - includes points, pivots, and projected points
    ////////////////////////////////////////////////////////////////////////////
    struct DPESInfo {
      size_t m_currentRoadmapVersion{size_t(-1)};      ///< RDMPVersion info
      std::unordered_set<VID> m_points;                ///< Unprojected points
      std::vector<CfgType> m_pivots;                        ///< Pivots
      std::unordered_map<VID, Projected> m_projectedPoints; ///< Projected cfgs
    };

    ///@}
    ///@name Construction
    ///@{

    DPESNF(std::string _dmLabel = "", bool _unconnected = false,
        size_t _m = 3, size_t _k = 10);

    DPESNF(XMLNode& _node);

    virtual ~DPESNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Interface
    ///@{

    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(RoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);


    /// Group overloads:
    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out) {
      throw RunTimeException(WHERE, "Not Supported for groups!");
    }

    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(GroupRoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out) {
      throw RunTimeException(WHERE, "Not Supported for groups!");
    }

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Add nodes to the model, updating pivots if necessary
    /// @tparam InputIterator Input iterator of roadmap vertices
    /// @param _rdmp Roadmap
    /// @param _first Beginning iterator of input
    /// @param _last Ending iterator of input
    ///
    /// Add [_first, _last) to the appropriate DPES model. Will create m new
    /// pivots if it is the first call, or m pivots have not been made yet.
    template<typename InputIterator>
    void CreatePivots(RoadmapType* _rdmp,
        InputIterator _first, InputIterator _last);

    /// Project point _c to R^m of pivot space
    /// @param _c Cfg
    /// @return Projected point
    ///
    /// Project _c to R^m by computing a distance to each pivot. Each distance
    /// is one dimension of the projected point.
    Projected Project(const CfgType& _c);

    /// Brute force computation of the K-closest elements in the
    /// projected space.
    /// @tparam OutputIterator Output of vid, distance pairs for nearest
    ///                        elements
    /// @param _rdmp Roadmap
    /// @param _c Query point
    /// @param _out Output iterator
    template<typename OutputIterator>
    OutputIterator KClosest(RoadmapType* _rdmp, const CfgType& _c,
        OutputIterator _out);

    /// Euclidean distance in R^m
    /// @param _v1 Projected point 1
    /// @param _v2 Projected point 2
    /// @return Distance
    double Euclidean(const vector<double>& _v1, const vector<double>& _v2);

    ///@}
    ///@name Internal State
    ///@{

    size_t m_m;   ///< Number of pivots
    unordered_map<RoadmapType*, DPESInfo>
      m_dpesInfo; ///< DPES info for all roadmaps

    DPESInfo* m_queryInfo; ///< Current model to construct and query on
    DPESInfo* m_tmpInfo{nullptr}; ///< Model for when query is not from an
                                  ///< entire roadmap

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
DPESNF<MPTraits>::
DPESNF(std::string _dmLabel, bool _unconnected, size_t _m, size_t _k)
  : NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected), m_m(_m) {
  this->SetName("DPESNF");
  this->m_nfType = K;
  this->m_k = _k;
}


template <typename MPTraits>
DPESNF<MPTraits>::
DPESNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("DPESNF");
  this->m_nfType = K;
  this->m_k = _node.Read("k", true, 5, 0, MAX_INT, "k value");
  m_m = _node.Read("m", true, 3, 1, MAX_INT, "m value for DPES");
}


template <typename MPTraits>
DPESNF<MPTraits>::
~DPESNF() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
DPESNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os
    << "\tk: " << this->m_k << endl
    << "\tm: " << m_m << endl;
}

/*----------------------- NeighborhoodFinder Interface -----------------------*/

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
DPESNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(), "DPESNF::FindNeighbors");
  this->IncrementNumQueries();

  size_t& currRdmp = m_dpesInfo[_rmp].m_currentRoadmapVersion;
  if(_fromFullRoadmap) {
    m_queryInfo = &m_dpesInfo[_rmp];
    size_t rdmp = _rmp->GetGraph()->GetTimestamp();
    if(currRdmp == size_t(-1) ||
        currRdmp < rdmp) {
      CreatePivots(_rmp, _first, _last);
      currRdmp = rdmp;
    }
  }
  else {
    delete m_tmpInfo;
    m_tmpInfo = new DPESInfo;
    m_queryInfo = m_tmpInfo;
    CreatePivots(_rmp, _first, _last);
  }

  _out = KClosest(_rmp, _cfg, _out);

  return _out;
}


template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
DPESNF<MPTraits>::
FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw RunTimeException(WHERE, "FindNeighborPairs is not yet implemented.");
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
template<typename InputIterator>
void
DPESNF<MPTraits>::
CreatePivots(RoadmapType* _rdmp,
    InputIterator _first, InputIterator _last) {

  typename RoadmapType::GraphType* g = _rdmp->GetGraph();
  auto dm = this->GetDistanceMetric(this->GetDMLabel());

  //Compute more pivots if necessary
  if(m_queryInfo->m_pivots.size() < m_m) {
    //Add all points first
    for(InputIterator i = _first; i != _last; ++i)
      m_queryInfo->m_points.insert(g->GetVID(i));

    m_queryInfo->m_pivots.clear();

    //First pivot selected randomly
    size_t sz = m_queryInfo->m_points.size();
    size_t indx = LRand() % sz;
    auto i = m_queryInfo->m_points.begin();
    advance(i, indx);
    m_queryInfo->m_pivots.push_back(g->GetVertex(i));

    //pivots 2..m - jth pivot selected by selecting a point in Points
    //which maximizes min_{i=i}^{j-1} d(p_i, p_j)
    for(size_t j = 1; j < m_m && j < sz; ++j) {
      auto maxI = m_queryInfo->m_points.begin();
      double maxV = 0;
      for(auto i = m_queryInfo->m_points.begin(); i != m_queryInfo->m_points.end(); ++i) {
        double minV = MAX_DBL;
        CfgType& c = g->GetVertex(i);
        for(auto& p : m_queryInfo->m_pivots)
          minV = min(minV, dm->Distance(p, c));

        if(minV > maxV) {
          maxI = i;
          maxV = minV;
        }
      }
      m_queryInfo->m_pivots.push_back(g->GetVertex(maxI));
    }

    //project all input points in R^m
    m_queryInfo->m_projectedPoints.clear();
    for(auto& v : m_queryInfo->m_points)
      m_queryInfo->m_projectedPoints[v] = Project(g->GetVertex(v));
  }
  //Otherwise just tack on projected points
  else {
    for(InputIterator i = _first; i != _last; ++i) {
      VID v = g->GetVID(i);
      if(!m_queryInfo->m_points.count(v)) {
        m_queryInfo->m_points.insert(v);
        m_queryInfo->m_projectedPoints[v] = Project(g->GetVertex(i));
      }
    }
  }
}

template <typename MPTraits>
typename DPESNF<MPTraits>::Projected
DPESNF<MPTraits>::
Project(const CfgType& _c) {
  auto dm = this->GetDistanceMetric(this->GetDMLabel());
  Projected proj(m_m, 0);
  for(size_t i = 0; i < m_queryInfo->m_pivots.size(); ++i)
    proj[i] = dm->Distance(_c, m_queryInfo->m_pivots[i]);
  return proj;
}

template <typename MPTraits>
template<typename OutputIterator>
OutputIterator
DPESNF<MPTraits>::
KClosest(RoadmapType* _rdmp, const CfgType& _c, OutputIterator _out) {
  Projected v = Project(_c);

  //K == 0 || query is less than the number of pivots -- Return all
  if(this->m_k == 0 || this->m_k > m_queryInfo->m_points.size()) {
    for(auto& p : m_queryInfo->m_projectedPoints)
      if(_rdmp->GetGraph()->GetVertex(p.first) != _c)
        *_out++ = make_pair(p.first, Euclidean(v, p.second));
    return _out;
  }

  // Keep sorted list of k best so far
  priority_queue<
    pair<VID, double>,
    vector<pair<VID, double>>,
    CompareSecond<VID, double>> pq;

  for(auto& p : m_queryInfo->m_projectedPoints) {

    if(this->CheckUnconnected(_rdmp, _c, p.first))
      continue;

    CfgType& node = _rdmp->GetGraph()->GetVertex(p.first);

    if(node == _c) // Don't connect to self
      continue;

    double dist = Euclidean(v, p.second);

    if(pq.size() < this->m_k){
      pq.push(make_pair(p.first, dist));
    }
    // If better than the worst so far, replace worst so far
    else if(dist < pq.top().second) {
      pq.pop();
      pq.push(make_pair(p.first, dist));
    }
  }

  // Transfer k closest to vector, sorted greatest to least dist
  vector<pair<VID, double> > closest;
  closest.reserve(pq.size());
  while(!pq.empty()) {
    closest.push_back(pq.top());
    pq.pop();
  }

  // Reverse order
  return copy(closest.rbegin(), closest.rend(), _out);
}

template <typename MPTraits>
double
DPESNF<MPTraits>::
Euclidean(const Projected& _v1, const Projected& _v2) {
  double dist = 0;
  for(size_t i = 0; i < m_m; ++i)
    dist += sqr(_v1[i] - _v2[i]);
  return sqrt(dist);
}

/*----------------------------------------------------------------------------*/

#endif
