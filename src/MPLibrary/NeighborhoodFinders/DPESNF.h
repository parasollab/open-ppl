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
/// Reference:
///   Erion Plaku and Lydia Kavraki. "Quantitative Analysis of Nearest-Neighbors
///   Search in High-Dimensional Sampling-Based Motion Planning." WAFR 2008.
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
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

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

    DPESNF();

    DPESNF(XMLNode& _node);

    virtual ~DPESNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

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

    /// Add nodes to the model, updating pivots if necessary
    /// @tparam InputIterator Input iterator of roadmap vertices
    /// @param _rdmp Roadmap
    /// @param _first Beginning iterator of input
    /// @param _last Ending iterator of input
    ///
    /// Add [_first, _last) to the appropriate DPES model. Will create m new
    /// pivots if it is the first call, or m pivots have not been made yet.
    template <typename InputIterator>
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
    /// @param _rdmp Roadmap
    /// @param _c Query point
    /// @param _out Output iterator
    void KClosest(RoadmapType* _rdmp, const CfgType& _c, OutputIterator _out);

    /// Euclidean distance in R^m
    /// @param _v1 Projected point 1
    /// @param _v2 Projected point 2
    /// @return Distance
    double Euclidean(const std::vector<double>& _v1,
        const std::vector<double>& _v2);

    ///@}
    ///@name Internal State
    ///@{

    size_t m_m;   ///< Number of pivots.

    /// DPES info for all roadmaps.
    std::unordered_map<RoadmapType*, DPESInfo> m_dpesInfo;

    DPESInfo* m_queryInfo{nullptr}; ///< The current model.

    /// Model for when query is not from an entire roadmap.
    DPESInfo* m_tmpInfo{nullptr};

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
DPESNF<MPTraits>::
DPESNF() : NeighborhoodFinderMethod<MPTraits>() {
  this->SetName("DPESNF");
  this->m_nfType = Type::K;
}


template <typename MPTraits>
DPESNF<MPTraits>::
DPESNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("DPESNF");
  this->m_nfType = Type::K;
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
  _os << "\tk: " << this->m_k << endl
      << "\tm: " << m_m << endl;
}

/*----------------------- NeighborhoodFinder Interface -----------------------*/

template <typename MPTraits>
template<typename InputIterator>
void
DPESNF<MPTraits>::
FindNeighbors(RoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  size_t& currRdmp = m_dpesInfo[_r].m_currentRoadmapVersion;

  if(_fromFullRoadmap) {
    m_queryInfo = &m_dpesInfo[_r];
    size_t rdmp = _r->GetTimestamp();
    if(currRdmp == size_t(-1) or currRdmp < rdmp) {
      CreatePivots(_r, _first, _last);
      currRdmp = rdmp;
    }
  }
  else {
    delete m_tmpInfo;
    m_tmpInfo = new DPESInfo;
    m_queryInfo = m_tmpInfo;
    CreatePivots(_r, _first, _last);
  }

  KClosest(_r, _cfg, _out);
}


template <typename MPTraits>
template <typename InputIterator>
void
DPESNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
template<typename InputIterator>
void
DPESNF<MPTraits>::
CreatePivots(RoadmapType* _rdmp,
    InputIterator _first, InputIterator _last) {

  auto dm = this->GetDistanceMetric(this->GetDMLabel());

  //Compute more pivots if necessary
  if(m_queryInfo->m_pivots.size() < m_m) {
    //Add all points first
    for(InputIterator i = _first; i != _last; ++i)
      m_queryInfo->m_points.insert(_rdmp->GetVID(i));

    m_queryInfo->m_pivots.clear();

    //First pivot selected randomly
    size_t sz = m_queryInfo->m_points.size();
    size_t indx = LRand() % sz;
    auto i = m_queryInfo->m_points.begin();
    std::advance(i, indx);
    m_queryInfo->m_pivots.push_back(_rdmp->GetVertex(i));

    //pivots 2..m - jth pivot selected by selecting a point in Points
    //which maximizes min_{i=i}^{j-1} d(p_i, p_j)
    for(size_t j = 1; j < m_m and j < sz; ++j) {
      auto maxI = m_queryInfo->m_points.begin();
      double maxV = 0;
      for(auto i = m_queryInfo->m_points.begin();
          i != m_queryInfo->m_points.end(); ++i) {
        double minV = MAX_DBL;
        CfgType& c = _rdmp->GetVertex(i);
        for(auto& p : m_queryInfo->m_pivots)
          minV = min(minV, dm->Distance(p, c));

        if(minV > maxV) {
          maxI = i;
          maxV = minV;
        }
      }
      m_queryInfo->m_pivots.push_back(_rdmp->GetVertex(maxI));
    }

    //project all input points in R^m
    m_queryInfo->m_projectedPoints.clear();
    for(auto& v : m_queryInfo->m_points)
      m_queryInfo->m_projectedPoints[v] = Project(_rdmp->GetVertex(v));
  }
  //Otherwise just tack on projected points
  else {
    for(InputIterator i = _first; i != _last; ++i) {
      VID v = _rdmp->GetVID(i);
      if(!m_queryInfo->m_points.count(v)) {
        m_queryInfo->m_points.insert(v);
        m_queryInfo->m_projectedPoints[v] = Project(_rdmp->GetVertex(i));
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
void
DPESNF<MPTraits>::
KClosest(RoadmapType* _rdmp, const CfgType& _c, OutputIterator _out) {
  const Projected v = Project(_c);

  // If m_k == 0 or query is less than the number of pivots, return all without
  // sorting.
  if(this->m_k == 0 or this->m_k > m_queryInfo->m_points.size()) {
    for(const auto& p : m_queryInfo->m_projectedPoints)
      if(_rdmp->GetVertex(p.first) != _c)
        *_out++ = Neighbor(p.first, Euclidean(v, p.second));
    return;
  }

  // Keep sorted list of k best so far
  std::priority_queue<Neighbor> pq;

  for(const auto& p : m_queryInfo->m_projectedPoints) {
    // Get the VID and check connectedness.
    const VID vid = p.first;
    if(this->DirectEdge(_rdmp, _c, vid))
      continue;

    // Get the configuration and check against connection to self.
    const CfgType& node = _rdmp->GetVertex(vid);
    if(node == _c)
      continue;

    // Check distance. If it is infinite, these are not connectable.
    const double distance = Euclidean(v, p.second);
    if(std::isinf(distance))
      continue;

    // Track the closest m_k neighbors.
    if(pq.size() < this->m_k)
      pq.emplace(vid, distance);
    else if(distance < pq.top().distance) {
      pq.pop();
      pq.emplace(vid, distance);
    }
  }

  // Transfer k closest to vector, sorted greatest to least distance
  std::vector<Neighbor> closest;
  closest.reserve(pq.size());
  while(!pq.empty()) {
    closest.push_back(pq.top());
    pq.pop();
  }

  // Reverse order
  std::copy(closest.rbegin(), closest.rend(), _out);
}


template <typename MPTraits>
double
DPESNF<MPTraits>::
Euclidean(const Projected& _v1, const Projected& _v2) {
  double distance = 0;
  for(size_t i = 0; i < m_m; ++i)
    distance += mathtool::sqr(_v1[i] - _v2[i]);
  return std::sqrt(distance);
}

/*----------------------------------------------------------------------------*/

#endif
