#ifndef PMPL_DPESNF_H_
#define PMPL_DPESNF_H_

#include "NeighborhoodFinderMethod.h"

#include "nonstd/container_ops.h"

#include <unordered_map>


////////////////////////////////////////////////////////////////////////////////
/// Distance-based Projection onto Euclidean Space (DPES).
///
/// Given a dimension m, project* all input points onto R^m. At query time,
/// project query configuration to R^m and use the Euclidean distance metric to
/// determine proximity.
///
/// The projection is done in a special way by choosing m 'pivot' points p_i in
/// the original configuration space s.t. p_1 is random and p_j for j:[2,m]
/// maximize the minimum distance between p_j and any prior pivot with i < j.
/// The projected representation for a query configuration q in R^m is then
/// sum_{i in 1 to m}(dm->Distance(q, p_i)). Thus, the dimensions of the
/// projected space R^m represent distances to the pivot points as measured with
/// the original metric.
///
/// The idea (I think) is to leverage the projected space as a cheaper way to
/// compute approximate nearest-neighbors in the original space. The intuition
/// seems to be that euclidean distance in projected space is equivalent or at
/// least representative of metric distance in the original space. However I
/// believe this essentially requires that m > dimension of the original space,
/// based on intution about triangulation in R^2 and R^3. In both of those
/// cases, one requires d+1 points for d dimensions to fully disambiguate a
/// unique location, and no two points can be co-linear without introducing
/// degeneracies. This method doesn't see to those considerations, but you
/// probably want to do that if you are expecting it to perform well.
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
///
/// @todo Verify this against the paper and ensure that it works properly with
///       multiple roadmaps.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DPESNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::VertexSet           VertexSet;
    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

    ////////////////////////////////////////////////////////////////////////////
    /// Model of projected space.
    ////////////////////////////////////////////////////////////////////////////
    struct ProjectedSpaceModel {

      typedef typename MPTraits::RoadmapType            RoadmapType;
      typedef typename RoadmapType::VID                 VID;
      typedef typename RoadmapType::VI                  VI;
      typedef typename RoadmapType::VertexSet           VertexSet;
      typedef typename MPTraits::CfgType                CfgType;
      typedef typename MPTraits::MPLibrary::DistanceMetricPointer
                                                        DistanceMetricPointer;
      typedef typename MPTraits::MPLibrary::SamplerPointer
                                                        SamplerPointer;

      /// Constructor.
      /// @param _r  The corresponding roadmap modeled by this.
      /// @param _dm The distance metric used in projection.
      /// @param _dimension The dimension of the projected space.
      /// @param _s The sampler used for generating pivots.
      /// @param _pivotCandidates The number of configurations to generate as
      ///                         candidate pivots.
      ProjectedSpaceModel(DPESNF<MPTraits>* const _parent, RoadmapType* const _r,
          DistanceMetricPointer _dm, const size_t _dimension,
          SamplerPointer _s, const size_t _pivotCandidates);

      /// Query.
      std::vector<Neighbor> operator()(const CfgType& _query) noexcept;

      private:

        typedef std::vector<double> Projected; ///< Projected point.

        /// Update the model from the buffer.
        void FlushBuffer() noexcept;

        /// Project a configuration.
        Projected Project(const CfgType& _cfg) const noexcept;

        /// Squared euclidean distance in R^m (we won't use this distance anyway
        /// so there is no point in taking the sqrt).
        double Euclidean(const Projected& _v1, const Projected& _v2) const
          noexcept;

        ///@name Internal State
        ///@{

        DPESNF<MPTraits>* const m_parent; ///< Parent object.
        RoadmapType* const m_roadmap; ///< The related roadmap.
        DistanceMetricPointer m_dm;   ///< Distance metric for original space.

        std::vector<CfgType> m_pivots;               ///< Pivot points.
        std::unordered_map<VID, Projected> m_points; ///< Projected points.

        VertexSet m_added;            ///< Buffer for added vertices.
        VertexSet m_deleted;          ///< Buffer for deleted vertices.

        ///@}

    };

    ///@}
    ///@name Construction
    ///@{

    DPESNF();

    DPESNF(XMLNode& _node);

    virtual ~DPESNF() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Overrides
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

    /// Get the projected space model for a roadmap, creating it if necessary.
    /// @param _r The roadmap.
    /// @return The projected space model for _r.
    ProjectedSpaceModel& GetModel(RoadmapType* const _r);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_samplerLabel;    ///< Sampler for pivots.
    size_t m_pivotCount;           ///< Number of pivots & projection dimension.
    size_t m_pivotCandidates;      ///< Number of pivot candidates.

    /// DPES info for all roadmaps.
    std::unordered_map<RoadmapType*, ProjectedSpaceModel> m_models;

    ///@}

};

/*~~~~~~~~~~~~~~~~~~~~~~~~~~ ProjectedSpaceModel ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

template <typename MPTraits>
DPESNF<MPTraits>::ProjectedSpaceModel::
ProjectedSpaceModel(DPESNF<MPTraits>* const _parent, RoadmapType* const _r,
    DistanceMetricPointer _dm, const size_t _dimension,
    SamplerPointer _s, const size_t _pivotCandidates)
    : m_parent(_parent), m_roadmap(_r), m_dm(_dm) {
  // Create candidate pivots to form the basis of our space.
  std::vector<CfgType> candidates;
  candidates.reserve(_pivotCandidates);
  _s->Sample(_pivotCandidates, 10000 * _pivotCandidates,
      m_parent->GetEnvironment()->GetBoundary(), std::back_inserter(candidates));
  // Ensure that the candidates are unique.
  candidates.erase(std::unique(candidates.begin(), candidates.end()),
                   candidates.end());
  if(candidates.size() < _dimension)
    throw RunTimeException(WHERE) << "Failed to generate enough unique pivots ("
                                  << candidates.size() << "/" << _dimension
                                  << ").";

  // Select the set of pivots from the candidates.
  m_pivots.reserve(_dimension);
  m_pivots.push_back(candidates.back());
  candidates.pop_back();
  for(size_t i = 1; i < _dimension; ++i) {
    double bestDistance = 0;
    size_t bestCandidate = 0;

    // Evaluate each remaining candidate.
    for(size_t j = 0; j < candidates.size(); ++j) {
      const CfgType& candidate = candidates[j];

      // Check the minimum distance to the current pivots.
      double minDistance = std::numeric_limits<double>::infinity();
      std::for_each(m_pivots.begin(), m_pivots.end(),
          [&](const CfgType& _pivot) {
            minDistance = std::min(minDistance,
                                   m_dm->Distance(_pivot, candidate));
          }
      );

      // If the min distance is higher than the current best, this is the
      // current best candidate.
      if(minDistance > bestDistance) {
        bestDistance = minDistance;
        bestCandidate = j;
      }
    }

    // Ensure that we got a non-zero min distance.
    if(bestDistance == 0)
      throw RunTimeException(WHERE) << "Can't have a zero distance between "
                                    << "unique pivots, the DM '"
                                    << m_dm->GetNameAndLabel()
                                    << "' is malformed.";

    // Extract the new pivot from the candidate set.
    m_pivots.emplace_back(candidates[bestCandidate]);
    candidates[bestCandidate] = candidates.back();
    candidates.pop_back();
  }

  // Define a function for adding VIDs to the buffer.
  auto adder = [this](const VI _vi) {
    const VID vid = _vi->descriptor();
    this->m_added.insert(vid);
  };
  auto deleter = [this](const VI _vi) {
    const VID vid = _vi->descriptor();
    this->m_deleted.insert(vid);
    this->m_added.erase(vid);
  };

  // Add the roadmap configurations.
  for(auto vi = _r->begin(); vi != _r->end(); ++vi)
    adder(vi);

  // Install the adder and deleter as roadmap hooks.
  const std::string label = _parent->GetNameAndLabel();
  _r->InstallHook(RoadmapType::HookType::AddVertex, label, adder);
  _r->InstallHook(RoadmapType::HookType::DeleteVertex, label, deleter);
}


template <typename MPTraits>
std::vector<Neighbor>
DPESNF<MPTraits>::ProjectedSpaceModel::
operator()(const CfgType& _query) noexcept {
  FlushBuffer();

  // Project the query.
  Projected q = Project(_query);

  // Search for the k best neighbors.
  std::priority_queue<Neighbor> pq;
  for(const auto& keyValue : m_points) {
    const VID vid      = keyValue.first;
    const Projected& p = keyValue.second;

    // Skip self.
    if(p == q)
      continue;

    // Compute euclidean distance in projected space.
    const double distance = Euclidean(p, q);
    if(std::isinf(distance))
      continue;

    // Track the closest m_k neighbors.
    if(pq.size() < m_parent->GetK())
      pq.emplace(vid, distance);
    else if(distance < pq.top().distance) {
      pq.pop();
      pq.emplace(vid, distance);
    }
  }

  // Extract neighbors from pq.
  std::vector<Neighbor> neighbors;
  neighbors.reserve(pq.size());
  while(!pq.empty()) {
    neighbors.push_back(pq.top());
    pq.pop();
  }

  // We won't bother reversing the order (currently greatest -> least) since
  // DPESNF will re-order based on the original DM.
  return neighbors;
}


template <typename MPTraits>
void
DPESNF<MPTraits>::ProjectedSpaceModel::
FlushBuffer() noexcept {
  // Delete points.
  for(const VID vid : m_deleted)
    m_points.erase(vid);
  m_deleted.clear();

  // Add points.
  for(const VID vid : m_added) {
    const CfgType& cfg = m_roadmap->GetVertex(vid);
    m_points.emplace(vid, Project(cfg));
  }
  m_added.clear();
}


template <typename MPTraits>
typename DPESNF<MPTraits>::ProjectedSpaceModel::Projected
DPESNF<MPTraits>::ProjectedSpaceModel::
Project(const CfgType& _cfg) const noexcept {
  Projected out;
  out.reserve(m_pivots.size());
  for(const CfgType& pivot : m_pivots)
    out.push_back(m_dm->Distance(_cfg, pivot));
  return out;
}


template <typename MPTraits>
double
DPESNF<MPTraits>::ProjectedSpaceModel::
Euclidean(const Projected& _v1, const Projected& _v2) const noexcept {
  double distance = 0;
  for(size_t i = 0; i < _v1.size(); ++i)
    distance += mathtool::sqr(_v1[i] - _v2[i]);
  return distance;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DPESNF ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
DPESNF<MPTraits>::
DPESNF() : NeighborhoodFinderMethod<MPTraits>(Type::K) {
  this->SetName("DPESNF");
}


template <typename MPTraits>
DPESNF<MPTraits>::
DPESNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node, Type::K) {
  this->SetName("DPESNF");

  m_samplerLabel = _node.Read("samplerLabel", true, "",
      "Sampler for generating pivot points.");

  m_pivotCount = _node.Read("pivotCount", true,
      m_pivotCount, size_t(1), std::numeric_limits<size_t>::max(),
      "Number of pivots and dimension of the projected space.");

  m_pivotCandidates = _node.Read("pivotCandidates", false,
      m_pivotCount * 10, size_t(1), std::numeric_limits<size_t>::max(),
      "Number of candidate pivots to sample. Default is 10x pivot count.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
DPESNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tk: " << this->m_k
      << "\n\tm: " << m_pivotCount
      << std::endl;
}

/*----------------------- NeighborhoodFinder Interface -----------------------*/

template <typename MPTraits>
void
DPESNF<MPTraits>::
FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  // Get the projected space model.
  ProjectedSpaceModel& model = GetModel(_r);

  // Get the nearest neighbors according to the model.
  std::vector<Neighbor> neighbors = model(_cfg);

  // Re-order the neighbors accoring to our distance metric.
  auto dm = this->GetDistanceMetric(this->m_dmLabel);
  for(Neighbor& n : neighbors) {
    const VID vid = n.target;
    const Cfg& cfg = _r->GetVertex(vid);
    n.distance = dm->Distance(_cfg, cfg);
  }
  std::sort(neighbors.begin(), neighbors.end());

  // Write neighbors to output.
  for(const Neighbor& n : neighbors)
    _out = n;
}


template <typename MPTraits>
void
DPESNF<MPTraits>::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
typename DPESNF<MPTraits>::ProjectedSpaceModel&
DPESNF<MPTraits>::
GetModel(RoadmapType* const _r) {
  // Check if the model already exists.
  auto iter = m_models.find(_r);
  if(iter != m_models.end())
    return iter->second;

  // No model found, create a new one.
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::CreateModel");

  auto iterBool = m_models.emplace(std::piecewise_construct,
      std::forward_as_tuple(_r),
      std::forward_as_tuple(this, _r, this->GetDistanceMetric(this->m_dmLabel),
          m_pivotCount, this->GetSampler(this->m_samplerLabel),
          m_pivotCandidates));
  iter = iterBool.first;
  return iter->second;
}

/*----------------------------------------------------------------------------*/

#endif
