#ifndef SYCLOP_H_
#define SYCLOP_H_

#include <iomanip>
#include <map>
#include <vector>

#include "BasicRRTStrategy.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief   This method is the 'Synergistic Combination of Layers of Planning'
///          technique that adds workspace guidance to RRT methods.
///
/// Paper reference:
///
/// Erion Plaku, Lydia E. Kavraki, Moshe Y. Vardi. "Synergistic Combination of
///   Layers of Planning". IEEE Transactions on Robotics. 2010.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class Syclop : public BasicRRTStrategy<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::CfgRef           CfgRef;
    typedef typename MPTraits::WeightType       WeightType;
    typedef typename MPTraits::MPProblemType    MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType   GraphType;
    typedef typename MPProblemType::VID         VID;

    ///@}
    ///\name Local Types
    ///@{

    typedef vector<VID>                         TreeType;
    typedef typename vector<TreeType>::iterator TreeIter;

    ///@}
    ///\name Construction
    ///@{

    Syclop();

    Syclop(MPProblemType* _problem, XMLNode& _node);

    virtual ~Syclop() = default;

    ///@}
    ///\name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;

    ///@}

  protected:

    ///\name Neighbor Helpers
    ///@{

    VID FindNearestNeighbor(const CfgType& _cfg, const TreeIter& _tree);

    ///@}
    ///\name Growth Helpers
    ///@{

    virtual VID Extend(const VID _nearVID, const CfgType& _qRand,
        const bool _lp = false) override;

    virtual pair<VID, bool> AddNode(const CfgType& _newCfg) override;

    ///@}
    ///\name Tree Helpers
    ///@{

    virtual VID ExpandTree(const VID _nearestVID, const CfgType& _target) override;

    ///@}
    ///@name Syclop Functions
    ///@{

    typedef int* RegionPointer;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute a high-level plan (a sequence of regions).
    vector<RegionPointer> DiscreteLead();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute a set of potential regions from the discrete lead.
    vector<RegionPointer> AvailableRegions(vector<RegionPointer> _lead);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Select a region from a set of available regions.
    RegionPointer SelectRegion(vector<RegionPointer> _available);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Select a vertex from within a given region.
    VID SelectVertex(RegionPointer _r);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Find the workspace region that holds a given configuration.
    RegionPointer LocateRegion(const VID _v);
    RegionPointer LocateRegion(const Point3d& _p);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Update the coverage of the region containing a new vertex.
    void UpdateCoverage(const VID _v);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Update the connectivity information after connecting a vertex v1
    ///        in a region r1 to a vertex v2 in a different region r2.
    void UpdateConnections(const VID _v1, const VID _v2);

    ///@}
    ///@name Syclop Helpers
    ///@}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Find the number of occupied sub-cells in a given region.
    size_t Cov(RegionPointer _r);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Estimate the free state space volume of a given region.
    double FreeVol(RegionPointer _r);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Count the number of times that _r1 and _r2 have been selected as
    ///        part of a discrete lead.
    size_t Sel(RegionPointer _r1, RegionPointer _r2);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Estimate the progress made in connecting _r1 to _r2.
    size_t Conn(RegionPointer _r1, RegionPointer _r2);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute the edge weight in the region graph from _r1 to _r2.
    double Cost(RegionPointer _r1, RegionPointer _r2);

    ///@}
    ///@name Syclop State
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Holds all external data related to a specific workspace region.
    ////////////////////////////////////////////////////////////////////////////
    struct RegionData {

      ///@name Internal State
      ///@{

      double weight; ///< Relative probability of selecting this region from a lead.
      double alpha;  ///< The edge-weight coefficient for this region.

      double freeVolume; ///< The estimated free state-space volume of this region.

      size_t numTimesSelected{0}; ///< The number of times this region has been selected.

      vector<VID> vertices; ///< The VID's of the configurations in this region.

      void UpdateAlpha(RegionPointer _r) {
        alpha = 1. / ((1. + Cov(_r)) * pow(FreeVol(_r), 4));
      }

      void UpdateWeight(RegionPointer _r) {
        weight = pow(FreeVol(_r), 4) /
            ((1. + Cov(_r)) * (1. + pow(numTimesSelected, 2)));
      }

      ///@}

    };

    /// Holds extra data associated with the regions.
    map<RegionPointer, RegionData> m_regionData;

    // Some kind of coverage map/structure

    // Some kind of connectivity map/structure

    ///@}
    ///@name Pre-processing Stuff
    ///@{

    void ComputeFreeVolumes();

    ///@}
};

/*----------------------------- construction ---------------------------------*/

template<class MPTraits>
Syclop<MPTraits>::
Syclop() : BasicRRTStrategy<MPTraits>() {
  this->SetName("Syclop");
}


template<class MPTraits>
Syclop<MPTraits>::
Syclop(MPProblemType* _problem, XMLNode& _node) :
    BasicRRTStrategy<MPTraits>(_problem, _node) {
  this->SetName("Syclop");
}

/*-------------------------- MPStrategy overrides ----------------------------*/

template<class MPTraits>
void
Syclop<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();

  // TODO: Decompose workspace.

  ComputeFreeVolumes();
}


template<class MPTraits>
void
Syclop<MPTraits>::
Iterate() {
  BasicRRTStrategy<MPTraits>::Iterate();
}

/*---------------------------- Neighbor Helpers ------------------------------*/

template<class MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const TreeIter& _tree) {
  this->GetStatClass()->StartClock("NeighborhoodFinding");

  vector<pair<VID, double>> neighbors;
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  nf->FindNeighbors(this->GetRoadmap(),
      _tree->begin(), _tree->end(),
      _tree->size() == this->GetRoadmap()->GetGraph()->get_num_vertices(),
      _cfg, back_inserter(neighbors));
  VID nearestVID = neighbors[0].first;

  this->GetStatClass()->StopClock("NeighborhoodFinding");
  return nearestVID;
}

/*----------------------------- Growth Helpers -------------------------------*/

template<class MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
Extend(const VID _nearVID, const CfgType& _qRand, const bool _lp) {
  // TODO: if we extended into a region that wasn't previously available, add it
  // to the list of available regions.
  return BasicRRTStrategy<MPTraits>::Extend(_nearVID, _qRand, _lp);
}


template<class MPTraits>
pair<typename BasicRRTStrategy<MPTraits>::VID, bool>
Syclop<MPTraits>::
AddNode(const CfgType& _newCfg) {
  auto added = BasicRRTStrategy<MPTraits>::AddNode(_newCfg);

  // If node is new and not invalid, update region data.
  if(added.second) {
    // Add this VID to the appropriate region.
    RegionPointer r = LocateRegion(added.first);
    m_regionData[r].vertices.push_back(added.first);

    // Update region data.
    m_regionData[r].UpdateAlpha(r);
    m_regionData[r].UpdateWeight(r);
  }

  return added;
}


/*------------------------------ Tree Helpers --------------------------------*/

template<class MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
ExpandTree(const VID _nearestVID, const CfgType& _target) {
  VID newVID = BasicRRTStrategy<MPTraits>::ExpandTree(_nearestVID, _target);
  return newVID;
}

/*------------------------------ Syclop Functions ----------------------------*/

template <typename MPTraits>
vector<typename Syclop<MPTraits>::RegionPointer>
Syclop<MPTraits>::
DiscreteLead() {
  static constexpr double probabilityOfDijkstras = .95;
  // TODO: Apply weights to region graph edges.

  // Search region graph from start to goal.
  vector<RegionPointer> path;

  if(DRand() < probabilityOfDijkstras) {
    // TODO: Search with djikstra's.
  }
  else {
    // TODO: Search with DFS, random child ordering.
  }

  // Return path.
  return path;
}


template <typename MPTraits>
vector<typename Syclop<MPTraits>::RegionPointer>
Syclop<MPTraits>::
AvailableRegions(vector<RegionPointer> _lead) {
  vector<RegionPointer> available;
  // TODO: select the available regions from the lead.

  return available;
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
SelectRegion(vector<RegionPointer> _available) {
  // Compute the total weight of the available regions.
  double totalWeight = 0.;
  for(const auto& region : _available)
    totalWeight += m_regionData[region].weight;

  // Roll a die in range [0, 1].
  const double roll = DRand();

  // Choose a region based on relative weight.
  double cumulative = 0.;
  for(const auto& region : _available) {
    cumulative += m_regionData[region].weight / totalWeight;
    if(cumulative > roll) {
      // Select this region!
      ++m_regionData[region].numTimesSelected;
      m_regionData[region].UpdateWeight(region);
      return region;
    }
  }

  throw RunTimeException(WHERE, "Failed to select a region by weight!");
  return nullptr;
}


template <typename MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
SelectVertex(RegionPointer _r) {
  const auto& vertices = m_regionData[_r].vertices;

  if(vertices.empty())
    throw RunTimeException(WHERE, "Tried to select a vertex from a region with "
        "no vertices.");

  size_t randomIndex = Rand() % vertices.size();
  return vertices[randomIndex];
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
LocateRegion(const VID _v) {
  // Get the reference point for this configuration.
  auto point = this->GetRoadmap()->GetGraph()->find_vertex(_v)->property().
      GetPoint();
  return LocateRegion(point);
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
LocateRegion(const Point3d& _p) {
  // Search region graph to see which region contains this point.
  {
    // Option 1: brute-force linear search
    auto regionGraph = this->GetEnvironment()->GetDecomposition();
    for(const auto& r : *regionGraph) {
      // TODO If point is inside r, return r.
    }
  }
  {
    // Option 2: find the coverage grid cell that holds this vertex, then map the
    // possible regions based on which ones touch that coverage cell.
    // TODO
  }

  return nullptr;
}


template <typename MPTraits>
void
Syclop<MPTraits>::
UpdateCoverage(const VID _v) {
  // TODO
}


template <typename MPTraits>
void
Syclop<MPTraits>::
UpdateConnections(const VID _v1, const VID _v2) {
  // TODO
}

/*------------------------------ Syclop Helpers ------------------------------*/

template <typename MPTraits>
size_t
Syclop<MPTraits>::
Cov(RegionPointer _r) {
  // Initially, just approximate the number of occupied cells as number of
  // vertices in the region.
  return m_regionData[_r].vertices.size();

  // TODO use the number of occupied coverage grid cells instead.
}


template <typename MPTraits>
inline
double
Syclop<MPTraits>::
FreeVol(RegionPointer _r) {
  return m_regionData[_r].freeVolume;
}


template <typename MPTraits>
size_t
Syclop<MPTraits>::
Sel(RegionPointer _r1, RegionPointer _r2) {
  // TODO
  return 0;
}


template <typename MPTraits>
size_t
Syclop<MPTraits>::
Conn(RegionPointer _r1, RegionPointer _r2) {
  // TODO
  return 0;
}


template <typename MPTraits>
double
Syclop<MPTraits>::
Cost(RegionPointer _r1, RegionPointer _r2) {
  double& a1 = m_regionData[_r1].alpha;
  double& a2 = m_regionData[_r2].alpha;
  return a1 * a2 * (1. + pow(Sel(_r1, _r2), 2)) / (1. + pow(Conn(_r1, _r2), 2));
}

/*--------------------------- Pre-processing Stuff ---------------------------*/

template <typename MPTraits>
double
Syclop<MPTraits>::
ComputeFreeVolumes() {
  static constexpr double eps = 1;
  static constexpr size_t numSamples = 5000;

  this->GetStatClass()->StartClock("ComputeFreeVolumes");

  // Make a fixed number of samples.
  auto sampler = this->GetSampler("UniformRandom");
  auto boundary = this->GetEnvironment()->GetBoundary();

  vector<CfgType> samples;
  sampler->Sample(numSamples, 1, boundary, back_inserter(samples));

  // Assert that we made the right number of samples.
  if(samples.size() != numSamples)
    throw RunTimeException(WHERE, "Tried to make " + to_string(numSamples) +
        " samples, but instead produced " + to_string(samples.size()));

  // Count the number of valid and invalid samples in each region.
  auto vc = this->GetValidityChecker("pqp_solid");

  map<RegionPointer, pair<size_t, size_t>> results;
  for(const auto& sample : samples) {
    RegionPointer r = LocateRegion(sample.GetPoint());
    if(vc->IsValid(samples[i], "Syclop::ComputeFreeVolumes"))
      ++results[r].first;
    else
      ++results[r].second;
  }

  // For each region, compute the approximate free volume using the samples.
  for(auto& regionData : m_regionData) {
    auto& region = regionData.first;
    auto& data = regionData.second;

    const size_t numValid = results[region].first;
    const size_t numInvalid = results[region].second;

    data.freeVolume = (eps + numValid) / (eps + numValid + numInvalid);
    data.freeVolume *= 1./* TODO: replace with volume of region */;
  }

  this->GetStatClass()->StopClock("ComputeFreeVolumes");
}

/*----------------------------------------------------------------------------*/

#endif
