#ifndef PMPL_SYCLOP_H_
#define PMPL_SYCLOP_H_

#include <algorithm>
#include <iomanip>
#include <map>
#include <set>
#include <vector>

#include "BasicRRTStrategy.h"
#include "MPLibrary/MPTools/TetGenDecomposition.h"
#include "Workspace/GridOverlay.h"
#include "Workspace/WorkspaceDecomposition.h"
#include "Workspace/WorkspaceRegion.h"

#include <containers/sequential/graph/algorithms/depth_first_search.h>


////////////////////////////////////////////////////////////////////////////////
/// This method is the 'Synergistic Combination of Layers of Planning' technique
/// that adds workspace guidance to RRT methods.
///
/// Reference:
///   Erion Plaku, Lydia E. Kavraki, Moshe Y. Vardi. "Synergistic Combination of
///   Layers of Planning". IEEE Transactions on Robotics. 2010.
///
/// @warning This method doesn't support bi-directional growth.
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class Syclop : public BasicRRTStrategy<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Local Types
    ///@{

    using typename BasicRRTStrategy<MPTraits>::TreeType;

    ///@}
    ///@name Construction
    ///@{

    Syclop();

    Syclop(XMLNode& _node);

    virtual ~Syclop() = default;

    ///@}

  protected:

    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name BasicRRTStrategy Overrides
    ///@{

    /// As basic RRT, but picks nearest neighbor from only the available
    /// regions.
    virtual VID FindNearestNeighbor(const CfgType& _cfg,
        const TreeType* const _tree = nullptr) override;

    /// As basic RRT, but also logs extension attempts.
    virtual VID Extend(const VID _nearVID, const CfgType& _qRand,
        LPOutput<MPTraits>& _lp, const bool _requireNew = true) override;

    /// As basic RRT, but also updates coverage information.
    virtual std::pair<VID, bool> AddNode(const CfgType& _newCfg) override;

    /// As basic RRT, but also updates region edge connectivity information.
    virtual void AddEdge(const VID _source, const VID _target,
        const LPOutput<MPTraits>& _lpOutput) override;

    ///@}
    ///@name Syclop Functions
    ///@{

    /// Compute a high-level plan (a sequence of regions).
    std::vector<VID> DiscreteLead();

    /// Compute a set of potential regions from the discrete lead.
    void FindAvailableRegions(std::vector<VID> _lead);

    /// Select a region from a set of available regions.
    const WorkspaceRegion* SelectRegion();

    /// Select a vertex from within a given region.
    VID SelectVertex(const WorkspaceRegion* const _r);

    ///@}
    ///@name Syclop Helpers
    ///@}

    /// Count the number of times that _r1 and _r2 have been selected as part of
    /// a discrete lead.
    size_t Sel(const WorkspaceRegion* const _r1,
               const WorkspaceRegion* const _r2);

    /// Estimate the progress made in connecting _r1 to _r2.
    size_t Conn(const WorkspaceRegion* const _r1,
                const WorkspaceRegion* const _r2);

    /// Compute the edge weight in the region graph from _r1 to _r2.
    double Cost(const WorkspaceRegion* const _r1,
                const WorkspaceRegion* const _r2);
    double Cost(const WorkspacePortal& _p); ///< @overload

    ///@}
    ///@name Pre-processing Stuff
    ///@{

    void ComputeFreeVolumes(); ///< Estimate the free volume of each region.

    ///@}
    ///@name Auxiliary Classes
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// Holds all external data related to a specific workspace region.
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

      std::vector<VID> vertices; ///< The VID's of the configurations in this region.

      std::set<size_t> cells; ///< The set of coverage cells in the target that
                         ///< were reached from the source.

      size_t Coverage() const {return cells.size();}

      ///@}
      ///@name Update Functions
      ///@{

      /// Add a cell to this region's cell indexes and update alpha/weight.
      void AddCell(size_t _index) {
        cells.insert(_index);
        UpdateAlpha();
        UpdateWeight();
      }

      void UpdateAlpha() {
        alpha = 1. / ((1. + Coverage()) * std::pow(freeVolume, 4.));
      }

      void UpdateWeight() {
        weight = std::pow(freeVolume, 4.) /
            ((1. + Coverage()) * (1. + std::pow(numTimesSelected, 2.)));
      }

      ///@}

    };

    //////////////////////////////////////////////////////////////////////////////
    /// Tracks data related to edges between regions in the decomposition graph.
    //////////////////////////////////////////////////////////////////////////////
    struct RegionPairData {

      /// Add the index of a cell containing a vertex in the target that was
      /// reached by extending from the source.
      void AddCell(size_t _index) {cells.insert(_index);}

      /// Get the coverage for this edge.
      size_t Coverage() const {return cells.size();}

      ///@name Internal State
      ///@{

      size_t numLeadUses{0};  ///< Times this edge was used in a lead.
      size_t numAttempts{0};  ///< Attempted extensions across this edge.

      private:

        /// The set of coverage cells in the target that were reached from the
        /// source.
        std::set<size_t> cells;

        ///@}
    };

    //////////////////////////////////////////////////////////////////////////////
    /// A functor for getting associating region graph edges with the cost
    /// function.
    //////////////////////////////////////////////////////////////////////////////
    struct WeightFunctor {

      ///@name Local Types
      ///@{

      typedef WorkspacePortal  property_type;
      typedef double           value_type;
      typedef Syclop<MPTraits> strategy_type;

      ///@}
      ///@name Internal State
      ///@{

      strategy_type* m_syclop;

      ///@}
      ///@name Construction
      ///@{

      WeightFunctor(strategy_type* _s) : m_syclop(_s) {}

      ///@}
      ///@name Edge Map Functor Interface
      ///@{

      value_type get(property_type& _p) {return m_syclop->Cost(_p);}

      void put(property_type& _p, value_type& _v) {}

      template <typename functor>
      void apply(property_type& _p, functor _f) {_f(_p);}

      ///@}

    };

    //////////////////////////////////////////////////////////////////////////////
    /// A visitor to track the parent-child relationships discovered during DFS.
    //////////////////////////////////////////////////////////////////////////////
    struct DFSVisitor : public stapl::visitor_base<WorkspaceDecomposition> {

      ///@name Local Types
      ///@{

      typedef WorkspaceDecomposition graph_type;
      typedef stapl::visitor_return  visitor_return;
      typedef std::map<VID, VID>          map_type;

      ///@}
      ///@name Internal State
      ///@{

      map_type& m_parentMap;

      ///@}
      ///@name Construction
      ///@{

      DFSVisitor(map_type& _pm) : m_parentMap(_pm) {}

      ///@}
      ///@name Visitor Interface
      ///@{

      virtual visitor_return tree_edge(graph_type::vertex_iterator _vit,
          graph_type::adj_edge_iterator _eit) override {
        m_parentMap[_eit->target()] = _eit->source();
        return visitor_return::CONTINUE;
      }

      ///@}
    };

    ///@}
    ///@name Syclop State
    ///@{

    /// Holds extra data associated with the regions.
    std::map<const WorkspaceRegion*, RegionData> m_regionData;

    /// Holds extra data associated with region pairs.
    std::map<std::pair<const WorkspaceRegion*, const WorkspaceRegion*>, RegionPairData>
        m_regionPairData;

    std::string m_tmLabel; ///< The topological map label.

    /// The currently available regions.
    std::set<const WorkspaceRegion*> m_availableRegions;

    ///@}
    ///@name Switch Tracking
    ///@{
    /// Data for knowing when to change regions/leads.

    size_t m_currentLeadUses{0};
    size_t m_maxLeadUses{6};

    size_t m_currentRegionUses{0};
    size_t m_maxRegionUses{6};

    const WorkspaceRegion* m_currentRegion{nullptr};

    bool m_improvement{false}; ///< Have we improved the map w/ current region?

    ///@}

};


/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
Syclop<MPTraits>::
Syclop() : BasicRRTStrategy<MPTraits>() {
  this->SetName("Syclop");
  this->m_growGoals = false;
}


template <typename MPTraits>
Syclop<MPTraits>::
Syclop(XMLNode& _node) : BasicRRTStrategy<MPTraits>(_node) {
  this->SetName("Syclop");

  if(this->m_growGoals)
    throw ParseException(WHERE) << "Syclop does not support bi-directional "
                                << "growth.";


  const size_t min = 1;
  const size_t max = numeric_limits<size_t>::max();

  m_maxLeadUses = _node.Read("maxLeadUses", false, m_maxLeadUses, min, max,
      "Number of times to use a discrete lead.");
  m_maxRegionUses = _node.Read("maxRegionUses", false, m_maxRegionUses, min, max,
      "Number of times to use a region.");
  m_tmLabel = _node.Read("tmLabel", true, "", "The topological map label.");
}

/*-------------------------- MPStrategy Overrides ----------------------------*/

template <typename MPTraits>
void
Syclop<MPTraits>::
Initialize() {
  // Only support single-goal tasks; this is inherent to the method. The problem
  // is solvable but hasn't been solved yet.
  const auto& goalConstraints = this->GetTask()->GetGoalConstraints();
  if(goalConstraints.size() != 1)
    throw RunTimeException(WHERE) << "Only supports single-goal tasks. "
                                  << "Multi-step tasks will need new skeletons "
                                  << "for each sub-component.";

  // Standard RRT initialization.
  BasicRRTStrategy<MPTraits>::Initialize();

  // Clear algorithm state.
  m_regionData.clear();
  m_regionPairData.clear();
  m_availableRegions.clear();
  m_currentRegion = nullptr;
  m_currentRegionUses = 0;
  m_currentLeadUses = 0;
  m_improvement = false;

  // Estimate the free c-space volume of each region.
  ComputeFreeVolumes();

  // Add start vertex to regionData.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto startRegion = tm->LocateRegion(0);
  m_regionData[startRegion].vertices.push_back(0);
}

/*------------------------ BasicRRTStrategy Overrides ------------------------*/

template <typename MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const TreeType* const) {
  // Select an available region and get the vertices within.
  const WorkspaceRegion* r = SelectRegion();
  const auto& vertices = m_regionData[r].vertices;

  // Find the nearest neighbor from within the selected region.
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "BasicRRT::FindNearestNeighbor");

  auto roadmap = this->GetRoadmap();
  auto g = roadmap->GetGraph();

  std::vector<Neighbor> neighbors;
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  nf->FindNeighbors(roadmap,
      vertices.begin(), vertices.end(),
      vertices.size() == g->Size(), _cfg, std::back_inserter(neighbors));

  VID nearestVID = INVALID_VID;

  if(!neighbors.empty())
    nearestVID = neighbors[0].target;
  else
    // We really don't want this to happen. If you see high numbers for this,
    // you likely have problems with parameter or algorithm selection.
    stats->IncStat("BasicRRT::FailedNF");

  return nearestVID;
}


template <typename MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
Extend(const VID _nearVID, const CfgType& _qRand, LPOutput<MPTraits>& _lp,
    const bool _requireNew) {
  // Log this extension attempt.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  const WorkspaceRegion* r1 = tm->LocateRegion(_nearVID);
  const WorkspaceRegion* r2 = tm->LocateRegion(_qRand);
  ++(m_regionPairData[std::make_pair(r1, r2)].numAttempts);

  return BasicRRTStrategy<MPTraits>::Extend(_nearVID, _qRand, _lp, _requireNew);
}


template <typename MPTraits>
std::pair<typename Syclop<MPTraits>::VID, bool>
Syclop<MPTraits>::
AddNode(const CfgType& _newCfg) {
  auto added = BasicRRTStrategy<MPTraits>::AddNode(_newCfg);

  // If node is new and not invalid, update region data.
  if(added.second) {
    // Find the region and coverage cell for this node.
    auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
    const WorkspaceRegion* r = tm->LocateRegion(added.first);
    const size_t cellIndex = tm->LocateCell(_newCfg.GetPoint());
    auto& data = m_regionData[r];

    size_t previousCoverage = data.Coverage();

    // Add this node to the appropriate region and update coverage.
    data.vertices.push_back(added.first);
    data.AddCell(cellIndex);

    // If we extended into a region that wasn't previously available, add it
    // to the list of available regions.
    auto iter = find(m_availableRegions.begin(), m_availableRegions.end(), r);
    if(iter == m_availableRegions.end())
      m_availableRegions.insert(r);

    // Mark that we improved the map.
    if(data.Coverage() != previousCoverage)
      m_improvement = true;
  }

  return added;
}


template <typename MPTraits>
void
Syclop<MPTraits>::
AddEdge(const VID _source, const VID _target, const LPOutput<MPTraits>& _lpOutput)
{
  BasicRRTStrategy<MPTraits>::AddEdge(_source, _target, _lpOutput);

  // Update the list of cells in the target region that were reached by
  // extending from the source region.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  const WorkspaceRegion* const r1 = tm->LocateRegion(_source);
  const WorkspaceRegion* const r2 = tm->LocateRegion(_target);
  const size_t cellIndex = tm->LocateCell(_target);

  m_regionPairData[std::make_pair(r1, r2)].AddCell(cellIndex);
}

/*------------------------------ Syclop Functions ----------------------------*/

template <typename MPTraits>
std::vector<typename Syclop<MPTraits>::VID>
Syclop<MPTraits>::
DiscreteLead() {
  this->GetStatClass()->StartClock("GenerateDiscreteLead");

  static constexpr double probabilityOfDijkstras = .95;

  if(this->m_debug)
    std::cout << "Generating new discrete lead..." << std::endl;

  // Get the start and goal point from the goal tracker.
  // Try to prevent non-point tasks by requiring single VIDs for the start and
  // goal.
  auto goalTracker = this->GetGoalTracker();
  const auto& startVIDs = goalTracker->GetStartVIDs();
  const auto& goalVIDs  = goalTracker->GetGoalVIDs(0);
  if(startVIDs.size() != 1)
    throw RunTimeException(WHERE) << "Exactly one start VID is required, but "
                                  << startVIDs.size() << " were found.";
  if(goalVIDs.size() != 1)
    throw RunTimeException(WHERE) << "Exactly one goal VID is required, but "
                                  << goalVIDs.size() << " were found.";

  // Find the start and goal regions.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto startRegion = tm->LocateRegion(*startVIDs.begin());
  auto goalRegion  = tm->LocateRegion(*goalVIDs.begin());

  // Find the start and goal in the graph
  auto regionGraph = this->GetMPTools()->GetTopologicalMap(m_tmLabel)->
      GetDecomposition();
  const VID start = regionGraph->GetDescriptor(*startRegion),
            goal  = regionGraph->GetDescriptor(*goalRegion);

  if(start == INVALID_VID)
    throw RunTimeException(WHERE) << "Could not locate start region.";
  if(goal == INVALID_VID)
    throw RunTimeException(WHERE) << "Could not locate goal region.";

  // Search region graph for a path from start to goal.
  std::vector<VID> path;
  if(DRand() < probabilityOfDijkstras) {
    // Search with djikstra's.
    if(this->m_debug)
      std::cout << "\tSearching with dijkstras algorithm..." << std::endl;

    // Set up an edge weight map that maps edges to the cost function.
    stapl::sequential::edge_property_map<WorkspaceDecomposition, WeightFunctor>
        edgeMap(const_cast<WorkspaceDecomposition&>(*regionGraph),
        WeightFunctor(this));

    // Apply dijkstra's search with the weight map.
    stapl::sequential::find_path_dijkstra(
        const_cast<WorkspaceDecomposition&>(*regionGraph), edgeMap, start, goal,
        path);
  }
  else {
    // Search with DFS, random child ordering.
    if(this->m_debug)
      std::cout << "\tSearching with DFS..." << std::endl;

    // Set up a parent map to capture the search path.
    std::map<VID, VID> parentMap;
    parentMap[start] = INVALID_VID;

    stapl::sequential::vector_property_map<GraphType, size_t> cmap;
    auto visitor = DFSVisitor(parentMap);
    stapl::sequential::depth_first_search(
        const_cast<WorkspaceDecomposition&>(*regionGraph), start, visitor, cmap);

    // Add the parent VIDs to the path starting from the end
    VID current = goal;
    path.push_back(goal);
    while(current != INVALID_VID) {
      path.push_back(parentMap[current]);
      current = parentMap[current];
    }

    if(this->m_debug)
      std::cout << "\t\tGenerated a parent map of size " << parentMap.size()
                << "."
                << std::endl;

    reverse(path.begin(), path.end());
  }

  // Increment the usage count for each region pair in this lead.
  for(auto i1 = path.begin(), i2 = i1 + 1; i2 != path.end(); ++i1, ++i2) {
    auto r1 = &regionGraph->GetRegion(*i1);
    auto r2 = &regionGraph->GetRegion(*i2);
    ++(m_regionPairData[std::make_pair(r1, r2)].numLeadUses);
  }

  // Mark this lead as unused.
  m_currentLeadUses = 0;

  // Sanity check.
  if(this->m_debug)
    std::cout << "\tGenerated lead of length " << path.size() << "."
              << std::endl;

  if(path.empty())
    throw RunTimeException(WHERE) << "Path is empty.";

  this->GetStatClass()->StopClock("GenerateDiscreteLead");
  return path;
}


template <typename MPTraits>
void
Syclop<MPTraits>::
FindAvailableRegions(std::vector<VID> _lead) {
  static constexpr double probabilityOfQuitting = .5;

  if(this->m_debug)
    std::cout << "Finding available regions from lead...\n";

  m_availableRegions.clear();

  auto regionGraph = this->GetMPTools()->GetTopologicalMap(m_tmLabel)->
      GetDecomposition();

  // Work backwards through the lead, adding non-empty regions until we quit or
  // hit the end.
  for(auto iter = _lead.rbegin(); iter != _lead.rend(); ++iter) {
    auto region = &regionGraph->GetRegion(*iter);
    const auto& data = m_regionData[region];

    // If there are no vertices in this region, move on to the next one.
    if(data.vertices.empty())
      continue;
    else
      m_availableRegions.insert(region);

    // Check for random quit.
    if(DRand() < probabilityOfQuitting)
      break;
  }

  // Sanity check.
  if(this->m_debug)
    std::cout << "\tFound " << m_availableRegions.size() << " available regions."
              << std::endl;

  if(m_availableRegions.empty())
    throw RunTimeException(WHERE) << "Available regions is empty";
}


template <typename MPTraits>
const WorkspaceRegion*
Syclop<MPTraits>::
SelectRegion() {
  static constexpr double probabilityOfAbandoningLead = .25;

  // We need a new region if we don't have one or if no uses remain.
  const bool needNewRegion = !m_currentRegion
                          or m_currentRegionUses >= m_maxRegionUses;

  if(this->m_debug) {
    std::cout << "Selecting a region...\n";
    if(m_currentRegion)
      std::cout << "\tCurrent region has been used " << m_currentRegionUses << "/"
                << m_maxRegionUses << " times.\n";
    std::cout << "\t"
         << (needNewRegion ? "Selecting a new region." : "Using current region.")
         << std::endl;
  }

  if(needNewRegion) {
    // We will choose a new region: reset its uses and improvement.
    m_currentRegionUses = 0;
    m_improvement = false;

    // If no regions are available or if we are out of uses for this lead,
    // compute a new lead and extract the available regions.
    const bool needNewLead = m_availableRegions.empty() ||
        m_currentLeadUses >= m_maxLeadUses;

    const bool abandonEarly = !m_improvement &&
        DRand() < probabilityOfAbandoningLead;

    if(this->m_debug) {
      if(!m_availableRegions.empty())
        std::cout << "\tCurrent lead has been used " << m_currentLeadUses << "/"
             << m_maxLeadUses << " times.\n";
      else
        std::cout << "\tNo discrete lead available.\n";
      if(!needNewLead && abandonEarly)
        std::cout << "\tAbandoning current lead early!\n";
      std::cout << "\t"
           << (needNewLead || abandonEarly ? "Generating new" : "Using current")
           << " lead." << std::endl;
    }

    if(needNewLead || abandonEarly)
      FindAvailableRegions(DiscreteLead());

    ++m_currentLeadUses;

    // Compute the total weight of the available regions.
    double totalWeight = 0.;
    for(const auto& region : m_availableRegions)
      totalWeight += m_regionData[region].weight;

    // Roll a die in range [0, 1].
    const double roll = DRand();

    // Choose a region based on relative weight.
    double cumulative = 0.;
    size_t count = 0;
    for(const auto& region : m_availableRegions) {
      ++count;
      const double relativeWeight = m_regionData[region].weight / totalWeight;
      cumulative += relativeWeight;
      if(cumulative > roll) {
        // Select this region!
        m_currentRegion = region;

        if(this->m_debug)
          std::cout << "\tSelected " << count << "th available region with "
                    << "relative weight " << setprecision(4) << relativeWeight
                    << " out of " << m_availableRegions.size() << " candidates."
                    << std::endl;
        break;
      }
    }
  }

  // Sanity check.
  if(!m_currentRegion)
    throw RunTimeException(WHERE) << "Failed to select a region!";

  // Update region data.
  ++m_currentRegionUses;
  ++m_regionData[m_currentRegion].numTimesSelected;
  m_regionData[m_currentRegion].UpdateWeight();

  return m_currentRegion;
}


template <typename MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
SelectVertex(const WorkspaceRegion* const _r) {
  const auto& vertices = m_regionData[_r].vertices;

  // Sanity check.
  if(vertices.empty())
    throw RunTimeException(WHERE) << "Tried to select a vertex from a region "
                                  << "with no vertices.";

  const size_t randomIndex = LRand() % vertices.size();
  const VID selected = vertices[randomIndex];

  if(this->m_debug)
    std::cout << "Selected VID " << selected << " out of " << vertices.size()
              << " vertices in the current region." << std::endl;
  return selected;
}

/*------------------------------ Syclop Helpers ------------------------------*/

template <typename MPTraits>
size_t
Syclop<MPTraits>::
Sel(const WorkspaceRegion* const _r1, const WorkspaceRegion* const _r2) {
  const auto& rd1 = m_regionData[_r1];
  const auto& rd2 = m_regionData[_r2];
  const auto& edgeData = m_regionPairData[std::make_pair(_r1, _r2)];

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
Conn(const WorkspaceRegion* const _r1, const WorkspaceRegion* const _r2) {
  return m_regionPairData[std::make_pair(_r1, _r2)].Coverage();
}


template <typename MPTraits>
double
Syclop<MPTraits>::
Cost(const WorkspacePortal& _p) {
  return Cost(&_p.GetSource(), &_p.GetTarget());
}


template <typename MPTraits>
double
Syclop<MPTraits>::
Cost(const WorkspaceRegion* const _r1, const WorkspaceRegion* const _r2) {
  const double a1 = m_regionData[_r1].alpha;
  const double a2 = m_regionData[_r2].alpha;
  return a1 * a2 * (1. + std::pow(Sel(_r1, _r2), 2)) / (1. + std::pow(Conn(_r1, _r2), 2));
}

/*--------------------------- Pre-processing Stuff ---------------------------*/

template <typename MPTraits>
void
Syclop<MPTraits>::
ComputeFreeVolumes() {
  MethodTimer mt(this->GetStatClass(), "ComputeFreeVolumes");

  if(this->m_debug)
    std::cout << "Computing region volumes and initializing region data...\n";

  static constexpr double eps = 1;
  static constexpr size_t numSamples = 5000;

  // Make a fixed number of samples.
  /// @todo Remove dependence on magic XML values.
  auto sampler = this->GetSampler("UniformRandom");
  auto boundary = this->GetEnvironment()->GetBoundary();

  std::vector<CfgType> samples;
  sampler->Sample(numSamples, 1, boundary, std::back_inserter(samples));

  // Assert that we made the right number of samples.
  if(samples.size() != numSamples)
    throw RunTimeException(WHERE) << "Tried to make " << numSamples
                                  << " samples, but instead produced "
                                  << std::to_string(samples.size()) << ".";

  // Count the number of valid and invalid samples in each region.
  /// @todo Remove dependence on magic XML values.
  auto vc = this->GetValidityChecker("pqp_solid");
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);

  std::map<const WorkspaceRegion*, std::pair<size_t, size_t>> results;
  for(auto& sample : samples) {
    const WorkspaceRegion* r = tm->LocateRegion(sample);
    if(!r)
      continue; // Skip samples whos projection is outside any region.
    else if(vc->IsValid(sample, "Syclop::ComputeFreeVolumes"))
      ++results[r].first;
    else
      ++results[r].second;
  }

  // For each region, compute the approximate free volume using the samples.
  auto tetrahedronVolume = [](const WorkspaceRegion* const _r) -> double {
    /// @sa Formula taken from reference:
    ///     http://mathworld.wolfram.com/Tetrahedron.html
    const Vector3d& base = _r->GetPoint(0);
    const Vector3d a = _r->GetPoint(1) - base;
    const Vector3d b = _r->GetPoint(2) - base;
    const Vector3d c = _r->GetPoint(3) - base;
    return std::abs(a * (b % c)) / 6.;
  };

  auto regionGraph = this->GetMPTools()->GetTopologicalMap(m_tmLabel)->
      GetDecomposition();
  for(auto iter = regionGraph->begin(); iter != regionGraph->end(); ++iter) {
    auto region = &iter->property();
    auto& data = m_regionData[region];

    const size_t numValid = results[region].first;
    const size_t numInvalid = results[region].second;

    data.freeVolume = tetrahedronVolume(region) * (eps + numValid) /
        (eps + numValid + numInvalid);
    data.UpdateWeight();
    data.UpdateAlpha();
  }

  if(this->m_debug)
    std::cout << "\tThere are " << regionGraph->GetNumRegions() << " regions.\n";
}

/*----------------------------------------------------------------------------*/

#endif
