#ifndef SYCLOP_H_
#define SYCLOP_H_

#include <iomanip>
#include <map>
#include <vector>

#include "BasicRRTStrategy.h"
#include "Workspace/GridOverlay.h"
#include "Workspace/WorkspaceDecomposition.h"
#include "Workspace/WorkspaceRegion.h"


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

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::CfgRef           CfgRef;
    typedef typename MPTraits::WeightType       WeightType;
    typedef typename MPTraits::MPProblemType    MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType   GraphType;
    typedef typename MPProblemType::VID         VID;

    ///@}
    ///@name Local Types
    ///@{

    typedef vector<VID>                         TreeType;
    typedef typename vector<TreeType>::iterator TreeIter;

    ///@}
    ///@name Construction
    ///@{

    Syclop();

    Syclop(MPProblemType* _problem, XMLNode& _node);

    virtual ~Syclop() = default;

    ///@}
    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;

    ///@}

  protected:

    ///@name Neighbor Helpers
    ///@{

    VID FindNearestNeighbor(const CfgType& _cfg, const TreeIter& _tree);

    ///@}
    ///@name Growth Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// As basic RRT, but also logs extension attempts.
    virtual VID Extend(const VID _nearVID, const CfgType& _qRand,
        const bool _lp = false) override;

    ////////////////////////////////////////////////////////////////////////////
    /// As basic RRT, but also updates coverage information.
    virtual pair<VID, bool> AddNode(const CfgType& _newCfg) override;

    ////////////////////////////////////////////////////////////////////////////
    /// As basic RRT, but also updates edge connectivity information.
    virtual void AddEdge(VID _source, VID _target,
        const LPOutput<MPTraits>& _lpOutput);

    ///@}
    ///@name Syclop Functions
    ///@{

    typedef WorkspaceRegion* RegionPointer;

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
    RegionPointer LocateRegion(const VID _v) const;
    RegionPointer LocateRegion(const CfgType& _c) const; ///< @overload
    RegionPointer LocateRegion(const Point3d& _p) const; ///< @overload

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Find the coverage cell index that holds a given configuration.
    size_t LocateCoverageCell(const VID _v) const;
    size_t LocateCoverageCell(const CfgType& _c) const; ///< @overload
    size_t LocateCoverageCell(const Point3d& _p) const; ///< @overload

    ///@}
    ///@name Syclop Helpers
    ///@}

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

      /// Relative probability of selecting this region from a lead.
      double weight;

      /// The edge-weight coefficient for this region.
      double alpha;

      /// The estimated free state-space volume of this region.
      double freeVolume;

      /// The number of times this region has been selected.
      size_t numTimesSelected{0};

      vector<VID> vertices; ///< The VID's of the configurations in this region.

      set<size_t> cells; ///< The set of coverage cells in the target that
                         ///< were reached from the source.

      size_t Coverage() const {return cells.size();}

      ///@}
      ///@name Update Functions
      ///@{

      void AddCell(size_t _index) {
        cells.insert(_index);
        UpdateAlpha();
        UpdateWeight();
      }

      void UpdateAlpha() {
        alpha = 1. / ((1. + Coverage()) * pow(freeVolume, 4));
      }

      void UpdateWeight() {
        weight = pow(freeVolume, 4) /
            ((1. + Coverage()) * (1. + pow(numTimesSelected, 2)));
      }

      ///@}

    };

    /// Holds extra data associated with the regions.
    map<RegionPointer, RegionData> m_regionData;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Tracks data related to edges between regions in the decomposition
    ///        graph.
    ////////////////////////////////////////////////////////////////////////////
    struct RegionPairData {

      // TODO: compute numLeadUses
      size_t numLeadUses{0};  ///< Times this edge was used in a lead.
      size_t numAttempts{0};  ///< Attempted extensions across this edge.

      /// Add the index of a cell containing a vertex in the target that was
      /// reached by extending from the source.
      void AddCell(size_t _index) {cells.insert(_index);}

      /// Get the coverage for this edge.
      size_t Coverage() const {return cells.size();}

      private:

        /// The set of coverage cells in the target that were reached from the
        /// source.
        set<size_t> cells;
    };

    /// Holds extra data associated with region pairs.
    map<pair<RegionPointer, RegionPointer>, RegionPairData> m_regionPairData;

    /// Coverage grid
    GridOverlay* m_grid{nullptr}; // TODO: validate impl.

    ///@}
    ///@name Pre-processing Stuff
    ///@{

    void ComputeFreeVolumes(); ///< Estimate the free volume of each region.

    ///@}
};

/*----------------------------- construction ---------------------------------*/

template<class MPTraits>
Syclop<MPTraits>::
Syclop() : BasicRRTStrategy<MPTraits>() {
  // Done.
  this->SetName("Syclop");
}


template<class MPTraits>
Syclop<MPTraits>::
Syclop(MPProblemType* _problem, XMLNode& _node) :
    BasicRRTStrategy<MPTraits>(_problem, _node) {
  // Done.
  this->SetName("Syclop");
}

/*-------------------------- MPStrategy overrides ----------------------------*/

template<class MPTraits>
void
Syclop<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();

  // Decompose workspace.
  auto env = this->GetEnvironment();
  env->Decompose(TetGenDecomposition(this->GetBaseFilename()));

  // Create coverage grid.
  delete m_grid;
  // TODO: Choose length intelligently instead of using 1.
  m_grid = new GridOverlay(env->GetBoundary(), 1);

  // Estimate the free c-space volume of each region.
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

  // Log this extension attempt.
  RegionPointer r1 = LocateRegion(_nearVID);
  RegionPointer r2 = LocateRegion(_qRand);
  ++(m_regionPairData[make_pair(r1, r2)].numAttempts);

  return BasicRRTStrategy<MPTraits>::Extend(_nearVID, _qRand, _lp);
}


template<class MPTraits>
pair<typename Syclop<MPTraits>::VID, bool>
Syclop<MPTraits>::
AddNode(const CfgType& _newCfg) {
  // Done.
  auto added = BasicRRTStrategy<MPTraits>::AddNode(_newCfg);

  // If node is new and not invalid, update region data.
  if(added.second) {
    RegionPointer r = LocateRegion(added.first);
    size_t cellIndex = LocateCoverageCell(_newCfg.GetPoint());

    // Add this VID to the appropriate region and update coverage.
    m_regionData[r].vertices.push_back(added.first);
    m_regionData[r].AddCell(cellIndex);
  }

  return added;
}


template<class MPTraits>
void
Syclop<MPTraits>::
AddEdge(VID _source, VID _target, const LPOutput<MPTraits>& _lpOutput) {
  // Done.
  BasicRRTStrategy<MPTraits>::AddEdge(_source, _target, _lpOutput);

  // Update the list of cells in the target region that were reached by
  // extending from the source region.
  RegionPointer r1 = LocateRegion(_source);
  RegionPointer r2 = LocateRegion(_target);
  size_t cellIndex = LocateCoverageCell(_target);

  m_regionPairData[make_pair(r1, r2)].AddCell(cellIndex);
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
  // Done.
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
      m_regionData[region].UpdateWeight();
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
  // Done.
  const auto& vertices = m_regionData[_r].vertices;

  if(vertices.empty())
    throw RunTimeException(WHERE, "Tried to select a vertex from a region with "
        "no vertices.");

  size_t randomIndex = rand() % vertices.size();
  return vertices[randomIndex];
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
LocateRegion(const VID _v) const {
  // Done.
  return LocateRegion(this->GetRoadmap()->GetGraph()->GetVertex(_v));
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
LocateRegion(const CfgType& _c) const {
  // Done.
  return LocateRegion(_c.GetPoint());
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
LocateRegion(const Point3d& _p) const {
  // Search region graph to see which region contains this point.
  {
    // Option 1: brute-force linear search
    //auto regionGraph = this->GetEnvironment()->GetDecomposition();
    //for(const auto& r : *regionGraph) {
    //  // TODO If point is inside r, return r.
    //}
  }
  {
    // Option 2: find the coverage grid cell that holds this vertex, then map the
    // possible regions based on which ones touch that coverage cell.
    // TODO
  }

  return nullptr;
}


template <typename MPTraits>
size_t
Syclop<MPTraits>::
LocateCoverageCell(const VID _v) const {
  // Done.
  return LocateCoverageCell(this->GetRoadmap()->GetGraph()->GetVertex(_v));
}


template <typename MPTraits>
size_t
Syclop<MPTraits>::
LocateCoverageCell(const CfgType& _c) const {
  // Done.
  return LocateCoverageCell(_c.GetPoint());
}


template <typename MPTraits>
size_t
Syclop<MPTraits>::
LocateCoverageCell(const Point3d& _p) const {
  // Done.
  return m_grid->LocateCell(_p);
}

/*------------------------------ Syclop Helpers ------------------------------*/

template <typename MPTraits>
size_t
Syclop<MPTraits>::
Sel(RegionPointer _r1, RegionPointer _r2) {
  // Done.
  const auto& rd1 = m_regionData[_r1];
  const auto& rd2 = m_regionData[_r2];
  const auto& edgeData = m_regionPairData[make_pair(_r1, _r2)];

  // If _r1 and _r2 have no samples, then return number of times discrete lead
  // has included the edge _r1, _r2.
  // Else, return number of times we tried to extend from _r1 to _r2.
  if(rd1.vertices.empty() && rd2.vertices.empty())
    return edgeData.numLeadUses;
  else
    return edgeData.numAttempts;
}


template <typename MPTraits>
size_t
Syclop<MPTraits>::
Conn(RegionPointer _r1, RegionPointer _r2) {
  // Done.
  return m_regionPairData[make_pair(_r1, _r2)].Coverage();
}


template <typename MPTraits>
double
Syclop<MPTraits>::
Cost(RegionPointer _r1, RegionPointer _r2) {
  // Done.
  double a1 = m_regionData[_r1].alpha;
  double a2 = m_regionData[_r2].alpha;
  return a1 * a2 * (1. + pow(Sel(_r1, _r2), 2)) / (1. + pow(Conn(_r1, _r2), 2));
}

/*--------------------------- Pre-processing Stuff ---------------------------*/

template <typename MPTraits>
void
Syclop<MPTraits>::
ComputeFreeVolumes() {
  // Done.
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
  for(auto& sample : samples) {
    RegionPointer r = LocateRegion(sample);
    if(vc->IsValid(sample, "Syclop::ComputeFreeVolumes"))
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

    auto tetrahedronVolume = [](const RegionPointer _r) -> double {
      /// @sa Formula taken from reference:
      ///     http://mathworld.wolfram.com/Tetrahedron.html
      const Vector3d& base = _r->GetPoint(0);
      const Vector3d a = _r->GetPoint(1) - base;
      const Vector3d b = _r->GetPoint(2) - base;
      const Vector3d c = _r->GetPoint(3) - base;
      return std::abs(a * (b % c)) / 6.;
    };

    data.freeVolume = tetrahedronVolume(region) * (eps + numValid) /
        (eps + numValid + numInvalid);
  }

  this->GetStatClass()->StopClock("ComputeFreeVolumes");
}

/*----------------------------------------------------------------------------*/

#endif
