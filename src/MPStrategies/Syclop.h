#ifndef SYCLOP_H_
#define SYCLOP_H_

#include <algorithm>
#include <iomanip>
#include <map>
#include <set>
#include <vector>

#include "BasicRRTStrategy.h"
#include "Utilities/TetGenDecomposition.h"
#include "Workspace/GridOverlay.h"
#include "Workspace/WorkspaceDecomposition.h"
#include "Workspace/WorkspaceRegion.h"

#include <containers/sequential/graph/algorithms/depth_first_search.h>


////////////////////////////////////////////////////////////////////////////////
/// This method is the 'Synergistic Combination of Layers of Planning' technique
/// that adds workspace guidance to RRT methods.
///
/// Paper reference:
///
/// Erion Plaku, Lydia E. Kavraki, Moshe Y. Vardi. "Synergistic Combination of
///   Layers of Planning". IEEE Transactions on Robotics. 2010.
/// @warning This method doesn't support bi-directional growth.
/// @ingroup MotionPlanningStrategies
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

    virtual ~Syclop();

    ///@}
    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;

    ///@}

  protected:

    ///@name Neighbor Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// As basic RRT, but picks nearest neighbor from only the available
    /// regions.
    virtual VID FindNearestNeighbor(const CfgType& _cfg, const TreeType& _tree)
        override;

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
    /// As basic RRT, but also updates region edge connectivity information.
    virtual void AddEdge(VID _source, VID _target,
        const LPOutput<MPTraits>& _lpOutput);

    ///@}
    ///@name Syclop Functions
    ///@{

    typedef const WorkspaceRegion* RegionPointer;

    ////////////////////////////////////////////////////////////////////////////
    /// Compute a high-level plan (a sequence of regions).
    vector<VID> DiscreteLead();

    ////////////////////////////////////////////////////////////////////////////
    /// Compute a set of potential regions from the discrete lead.
    void FindAvailableRegions(vector<VID> _lead);

    ////////////////////////////////////////////////////////////////////////////
    /// Select a region from a set of available regions.
    RegionPointer SelectRegion();

    ////////////////////////////////////////////////////////////////////////////
    /// Select a vertex from within a given region.
    VID SelectVertex(RegionPointer _r);

    ////////////////////////////////////////////////////////////////////////////
    /// Find the workspace region that holds a given configuration.
    RegionPointer LocateRegion(const VID _v) const;
    RegionPointer LocateRegion(const CfgType& _c) const; ///< @overload
    RegionPointer LocateRegion(const Point3d& _p) const; ///< @overload

    ////////////////////////////////////////////////////////////////////////////
    /// Find the coverage cell index that holds a given configuration.
    size_t LocateCoverageCell(const VID _v) const;
    size_t LocateCoverageCell(const CfgType& _c) const; ///< @overload
    size_t LocateCoverageCell(const Point3d& _p) const; ///< @overload

    ///@}
    ///@name Syclop Helpers
    ///@}

    ////////////////////////////////////////////////////////////////////////////
    /// Count the number of times that _r1 and _r2 have been selected as
    ///        part of a discrete lead.
    size_t Sel(RegionPointer _r1, RegionPointer _r2);

    ////////////////////////////////////////////////////////////////////////////
    /// Estimate the progress made in connecting _r1 to _r2.
    size_t Conn(RegionPointer _r1, RegionPointer _r2);

    ////////////////////////////////////////////////////////////////////////////
    /// Compute the edge weight in the region graph from _r1 to _r2.
    double Cost(RegionPointer _r1, RegionPointer _r2);
    double Cost(const WorkspacePortal& _p); ///< @overload

    ///@}
    ///@name Pre-processing Stuff
    ///@{

    void ComputeFreeVolumes(); ///< Estimate the free volume of each region.

    void ComputeGridMap(); ///< Compute map from coverage grid to regions.

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

      vector<VID> vertices; ///< The VID's of the configurations in this region.

      set<size_t> cells; ///< The set of coverage cells in the target that
                         ///< were reached from the source.

      size_t Coverage() const {return cells.size();}

      ///@}
      ///@name Update Functions
      ///@{

      //////////////////////////////////////////////////////////////////////////
      /// Add a cell to this region's cell indexes and update alpha/weight.
      void AddCell(size_t _index) {
        cells.insert(_index);
        UpdateAlpha();
        UpdateWeight();
      }

      void UpdateAlpha() {
        alpha = 1. / ((1. + Coverage()) * pow(freeVolume, 4.));
      }

      void UpdateWeight() {
        weight = pow(freeVolume, 4.) /
            ((1. + Coverage()) * (1. + pow(numTimesSelected, 2.)));
      }

      ///@}

    };

    //////////////////////////////////////////////////////////////////////////////
    /// Tracks data related to edges between regions in the decomposition graph.
    //////////////////////////////////////////////////////////////////////////////
    struct RegionPairData {

      ////////////////////////////////////////////////////////////////////////////
      /// Add the index of a cell containing a vertex in the target that was
      /// reached by extending from the source.
      void AddCell(size_t _index) {cells.insert(_index);}

      ////////////////////////////////////////////////////////////////////////////
      /// Get the coverage for this edge.
      size_t Coverage() const {return cells.size();}

      ///@name Internal State
      ///@{

      size_t numLeadUses{0};  ///< Times this edge was used in a lead.
      size_t numAttempts{0};  ///< Attempted extensions across this edge.

      private:

        /// The set of coverage cells in the target that were reached from the
        /// source.
        set<size_t> cells;

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
      typedef map<VID, VID>          map_type;

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
    map<RegionPointer, RegionData> m_regionData;

    /// Holds extra data associated with region pairs.
    map<pair<RegionPointer, RegionPointer>, RegionPairData> m_regionPairData;

    /// The coverage grid.
    GridOverlay* m_grid{nullptr};

    /// A map from grid cells to regions.
    vector<vector<RegionPointer>> m_gridMap;

    /// The currently available regions.
    set<RegionPointer> m_availableRegions;

    ///@}
    ///@name Switch Tracking
    ///@{
    /// Data for knowing when to change regions/leads.

    size_t m_currentLeadUses{0};
    size_t m_maxLeadUses{6};

    size_t m_currentRegionUses{0};
    size_t m_maxRegionUses{6};

    RegionPointer m_currentRegion{nullptr};

    bool m_improvement{false}; ///< Have we improved the map w/ current region?

    ///@}

};


/*----------------------------- Construction ---------------------------------*/

template<class MPTraits>
Syclop<MPTraits>::
Syclop() : BasicRRTStrategy<MPTraits>() {
  this->SetName("Syclop");
  this->m_growGoals = false;
}


template<class MPTraits>
Syclop<MPTraits>::
Syclop(MPProblemType* _problem, XMLNode& _node) :
    BasicRRTStrategy<MPTraits>(_problem, _node) {
  this->SetName("Syclop");
  this->m_growGoals = false;

  const size_t min = 1;
  const size_t max = numeric_limits<size_t>::max();
  m_maxLeadUses = _node.Read("maxLeadUses", false, m_maxLeadUses, min, max,
      "Number of times to use a discrete lead");
  m_maxRegionUses = _node.Read("maxRegionUses", false, m_maxRegionUses, min, max,
      "Number of times to use a region");
}


template<class MPTraits>
Syclop<MPTraits>::
~Syclop() {
  delete m_grid;
}

/*-------------------------- MPStrategy overrides ----------------------------*/

template<class MPTraits>
void
Syclop<MPTraits>::
Initialize() {
  // Standard RRT initialization.
  BasicRRTStrategy<MPTraits>::Initialize();

  // Decompose workspace.
  this->GetStatClass()->StartClock("Decomposition");
  auto env = this->GetEnvironment();
  env->Decompose(TetGenDecomposition(this->GetBaseFilename()));
  this->GetStatClass()->StopClock("Decomposition");

#ifdef VIZMO
  // Add workspace model if we are in vizmo land.
  GetVizmo().GetEnv()->AddWorkspaceDecompositionModel(env->GetDecomposition());
  GetMainWindow()->GetModelSelectionWidget()->CallResetLists();
#endif

  // Clear algorithm state.
  m_regionData.clear();
  m_regionPairData.clear();
  m_availableRegions.clear();
  m_gridMap.clear();
  m_currentRegion = nullptr;
  m_currentRegionUses = 0;
  m_currentLeadUses = 0;
  m_improvement = false;

  // Create coverage grid.
  const double gridLength = env->GetPositionRes() * 10;
  delete m_grid;
  m_grid = new GridOverlay(env->GetBoundary(), gridLength);

  // Map the coverage grid cells to the workspace regions.
  ComputeGridMap();

  // Estimate the free c-space volume of each region.
  ComputeFreeVolumes();

  // Add start vertex to regionData.
  auto startRegion = LocateRegion(0);
  m_regionData[startRegion].vertices.push_back(0);
}

/*---------------------------- Neighbor Helpers ------------------------------*/

template<class MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const TreeType&) {
  // Select an available region and get the vertices within.
  RegionPointer r = SelectRegion();
  const auto& vertices = m_regionData[r].vertices;

  // Find the nearest neighbor from within the selected region.
  return BasicRRTStrategy<MPTraits>::FindNearestNeighbor(_cfg, vertices);
}

/*----------------------------- Growth Helpers -------------------------------*/

template<class MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
Extend(const VID _nearVID, const CfgType& _qRand, const bool _lp) {
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
  auto added = BasicRRTStrategy<MPTraits>::AddNode(_newCfg);

  // If node is new and not invalid, update region data.
  if(added.second) {
    // Find the region and coverage cell for this node.
    RegionPointer r = LocateRegion(added.first);
    size_t cellIndex = LocateCoverageCell(_newCfg.GetPoint());
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


template<class MPTraits>
void
Syclop<MPTraits>::
AddEdge(VID _source, VID _target, const LPOutput<MPTraits>& _lpOutput) {
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
vector<typename Syclop<MPTraits>::VID>
Syclop<MPTraits>::
DiscreteLead() {
  this->GetStatClass()->StartClock("GenerateDiscreteLead");

  static constexpr double probabilityOfDijkstras = .95;

  if(this->m_debug)
    cout << "Generating new discrete lead..." << endl;

  auto regionGraph = this->GetEnvironment()->GetDecomposition();

  // Get start and goal configurations from the query.
  const CfgType& startCfg = this->m_query->GetQuery()[0];
  const CfgType& goalCfg = this->m_query->GetQuery()[1];

  // Find the start and goal regions.
  auto startRegion = LocateRegion(startCfg);
  auto goalRegion = LocateRegion(goalCfg);

  // Find the start and goal in the graph
  size_t start, goal;
  for(auto iter = regionGraph->begin(); iter != regionGraph->end(); ++iter) {
    if(&iter->property() == startRegion)
      start = iter->descriptor();
    if(&iter->property() == goalRegion)
      goal = iter->descriptor();
  }

  // Search region graph for a path from start to goal.
  vector<VID> path;
  if(DRand() < probabilityOfDijkstras) {
    // Search with djikstra's.
    if(this->m_debug)
      cout << "\tSearching with dijkstras algorithm..." << endl;

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
      cout << "\tSearching with DFS..." << endl;

    // Set up a parent map to capture the search path.
    map<VID, VID> parentMap;
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
      cout << "\t\tGenerated a parent map of size " << parentMap.size() << ".\n";

    reverse(path.begin(), path.end());
  }

  // Increment the usage count for each region pair in this lead.
  for(auto i1 = path.begin(), i2 = i1 + 1; i2 != path.end(); ++i1, ++i2) {
    auto r1 = &regionGraph->GetRegion(*i1);
    auto r2 = &regionGraph->GetRegion(*i2);
    ++(m_regionPairData[make_pair(r1, r2)].numLeadUses);
  }

  // Mark this lead as unused.
  m_currentLeadUses = 0;

  // Sanity check.
  if(this->m_debug)
    cout << "\tGenerated lead of length " << path.size() << "." << endl;
  if(path.empty())
    throw RunTimeException(WHERE, "Path is empty");

  this->GetStatClass()->StopClock("GenerateDiscreteLead");
  return path;
}


template <typename MPTraits>
void
Syclop<MPTraits>::
FindAvailableRegions(vector<VID> _lead) {
  static constexpr double probabilityOfQuitting = .5;

  if(this->m_debug)
    cout << "Finding available regions from lead...\n";

  m_availableRegions.clear();

  auto regionGraph = this->GetEnvironment()->GetDecomposition();

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
    cout << "\tFound " << m_availableRegions.size() << " available regions.\n";
  if(m_availableRegions.empty())
    throw RunTimeException(WHERE, "Available regions is empty");
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
SelectRegion() {
  static constexpr double probabilityOfAbandoningLead = .25;

  // We need a new region if we don't have one or if no uses remain.
  const bool needNewRegion = !m_currentRegion ||
      m_currentRegionUses >= m_maxRegionUses;

  if(this->m_debug) {
    cout << "Selecting a region...\n";
    if(m_currentRegion)
      cout << "\tCurrent region has been used " << m_currentRegionUses << "/"
           << m_maxRegionUses << " times.\n";
    cout << "\t"
         << (needNewRegion ? "Selecting a new region." : "Using current region.")
         << endl;
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
        cout << "\tCurrent lead has been used " << m_currentLeadUses << "/"
             << m_maxLeadUses << " times.\n";
      else
        cout << "\tNo discrete lead available.\n";
      if(!needNewLead && abandonEarly)
        cout << "\tAbandoning current lead early!\n";
      cout << "\t"
           << (needNewLead || abandonEarly ? "Generating new" : "Using current")
           << " lead." << endl;
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
          cout << "\tSelected " << count << "th available region with "
               << "relative weight " << setprecision(4) << relativeWeight
               << " out of " << m_availableRegions.size() << " candidates."
               << endl;
        break;
      }
    }
  }

  // Sanity check.
  if(!m_currentRegion)
    throw RunTimeException(WHERE, "Failed to select a region!");

  // Update region data.
  ++m_currentRegionUses;
  ++m_regionData[m_currentRegion].numTimesSelected;
  m_regionData[m_currentRegion].UpdateWeight();

  return m_currentRegion;
}


template <typename MPTraits>
typename Syclop<MPTraits>::VID
Syclop<MPTraits>::
SelectVertex(RegionPointer _r) {
  const auto& vertices = m_regionData[_r].vertices;

  // Sanity check.
  if(vertices.empty())
    throw RunTimeException(WHERE, "Tried to select a vertex from a region with "
        "no vertices.");

  const size_t randomIndex = rand() % vertices.size();
  const VID selected = vertices[randomIndex];

  if(this->m_debug)
    cout << "Selected VID " << selected << " out of " << vertices.size()
         << " vertices in the current region." << endl;
  return selected;
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
LocateRegion(const VID _v) const {
  return LocateRegion(this->GetRoadmap()->GetGraph()->GetVertex(_v));
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
LocateRegion(const CfgType& _c) const {
  return LocateRegion(_c.GetPoint());
}


template <typename MPTraits>
typename Syclop<MPTraits>::RegionPointer
Syclop<MPTraits>::
LocateRegion(const Point3d& _p) const {
  this->GetStatClass()->StartClock("LocateRegion");

  // Find the coverage grid cell that holds this vertex, then use the grid map
  // to find the set of regions that touch the grid cell.
  const size_t cell = LocateCoverageCell(_p);
  auto candidateRegions = m_gridMap.at(cell);

  // Scan through the candidate regions and return the first one that contains
  // our point.
  for(auto region : candidateRegions) {
    if(region->GetBoundary()->InBoundary(_p)) {
      this->GetStatClass()->StopClock("LocateRegion");
      return region;
    }
  }

  this->GetStatClass()->StopClock("LocateRegion");
  return nullptr;
}


template <typename MPTraits>
size_t
Syclop<MPTraits>::
LocateCoverageCell(const VID _v) const {
  return LocateCoverageCell(this->GetRoadmap()->GetGraph()->GetVertex(_v));
}


template <typename MPTraits>
size_t
Syclop<MPTraits>::
LocateCoverageCell(const CfgType& _c) const {
  return LocateCoverageCell(_c.GetPoint());
}


template <typename MPTraits>
size_t
Syclop<MPTraits>::
LocateCoverageCell(const Point3d& _p) const {
  this->GetStatClass()->StartClock("LocateCoverageCell");
  const size_t cell = m_grid->LocateCell(_p);
  this->GetStatClass()->StopClock("LocateCoverageCell");
  return cell;
}

/*------------------------------ Syclop Helpers ------------------------------*/

template <typename MPTraits>
size_t
Syclop<MPTraits>::
Sel(RegionPointer _r1, RegionPointer _r2) {
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
  return m_regionPairData[make_pair(_r1, _r2)].Coverage();
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
Cost(RegionPointer _r1, RegionPointer _r2) {
  double a1 = m_regionData[_r1].alpha;
  double a2 = m_regionData[_r2].alpha;
  return a1 * a2 * (1. + pow(Sel(_r1, _r2), 2)) / (1. + pow(Conn(_r1, _r2), 2));
}

/*--------------------------- Pre-processing Stuff ---------------------------*/

template <typename MPTraits>
void
Syclop<MPTraits>::
ComputeFreeVolumes() {
  this->GetStatClass()->StartClock("ComputeFreeVolumes");

  if(this->m_debug)
    cout << "Computing region volumes and initializing region data...\n";

  static constexpr double eps = 1;
  static constexpr size_t numSamples = 5000;

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
    if(!r)
      continue; // Skip samples whos projection is outside any region.
    else if(vc->IsValid(sample, "Syclop::ComputeFreeVolumes"))
      ++results[r].first;
    else
      ++results[r].second;
  }

  // For each region, compute the approximate free volume using the samples.
  auto regionGraph = this->GetEnvironment()->GetDecomposition();
  for(auto iter = regionGraph->begin(); iter != regionGraph->end(); ++iter) {
    auto region = &iter->property();
    auto& data = m_regionData[region];

    const size_t numValid = results[region].first;
    const size_t numInvalid = results[region].second;

    auto tetrahedronVolume = [](RegionPointer _r) -> double {
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
    data.UpdateWeight();
    data.UpdateAlpha();
  }

  if(this->m_debug)
    cout << "\tThere are " << regionGraph->GetNumRegions() << " regions.\n";

  this->GetStatClass()->StopClock("ComputeFreeVolumes");
}


template <typename MPTraits>
void
Syclop<MPTraits>::
ComputeGridMap() {
  this->GetStatClass()->StartClock("ComputeGridMap");

  if(this->m_debug)
    cout << "Computing grid map...";

  m_gridMap.resize(m_grid->Size());

  // For each region, find the coverage cells that are associated with it.
  auto regionGraph = this->GetEnvironment()->GetDecomposition();
  for(auto iter = regionGraph->begin(); iter != regionGraph->end(); ++iter) {
    auto region = &iter->property();
    auto cells = m_grid->LocateBBXCells(region->GetBoundary());
    for(auto index : cells)
      m_gridMap[index].push_back(region);
  }

  if(this->m_debug)
    cout << "done. Grid has " << m_grid->Size() << " cells ("
         << m_grid->Size(0) << "x" << m_grid->Size(1) << "x" << m_grid->Size(2)
         << ")." << endl;

  this->GetStatClass()->StopClock("ComputeGridMap");
}

/*----------------------------------------------------------------------------*/

#endif
