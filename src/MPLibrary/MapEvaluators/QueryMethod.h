#ifndef QUERY_METHOD_H_
#define QUERY_METHOD_H_

#include "MapEvaluatorMethod.h"

#include "ConfigurationSpace/Path.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/DynamicWeightMap.h"
#include "Utilities/MetricUtils.h"
#include "nonstd/io.h"

#include "containers/sequential/graph/algorithms/astar.h"


////////////////////////////////////////////////////////////////////////////////
/// Heuristic for A* graph search. Uses FindIncrement to estimate the distance
/// to a given goal configuration.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
struct Heuristic {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType                     CfgType;
    typedef typename MPTraits::WeightType                  WeightType;

    ///@}
    ///@name Construction
    ///@{

    Heuristic(const CfgType& _goal, double _posRes, double _oriRes) :
        m_goal(_goal), m_posRes(_posRes), m_oriRes(_oriRes) { }

    ///@}
    ///@name Interface
    ///@{

    /// Return the number of ticks as the estimated distance from a given
    /// configuration to the goal.
    /// @param[in] _c The configuration of interest.
    /// @return The estimated distance from _c to m_goal.
    WeightType operator()(const CfgType& _c) {
      int tick;
      CfgType incr(_c.GetRobot());
      incr.FindIncrement(_c, m_goal, &tick, m_posRes, m_oriRes);
      return WeightType("", tick / 2);
    }

    ///@}

  private:

    ///@name Internal State
    ///@{

    CfgType m_goal;  ///< The goal configuration for this search.
    double m_posRes; ///< The position resolultion to use.
    double m_oriRes; ///< The orientation resolution to use.

    ///@}
};


////////////////////////////////////////////////////////////////////////////////
/// Base class for all query methods. These objects evaluate a roadmap under
/// construction to see if a planning query has been satisfied.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class QueryMethod : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType            CfgType;
    typedef typename MPTraits::WeightType         WeightType;
    typedef typename MPTraits::RoadmapType        RoadmapType;
    typedef typename RoadmapType::GraphType       GraphType;
    typedef typename GraphType::VID               VID;
    typedef typename GraphType::EID::edge_id_type EID;

    typedef stapl::sequential::map_property_map<GraphType, size_t> ColorMap;

    ///@}
    ///@name Construction
    ///@{

    QueryMethod();
    QueryMethod(XMLNode& _node);
    virtual ~QueryMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;
    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluator Interface
    ///@{

    virtual bool operator()() override;

    ///@}
    ///@name Query Interface
    ///@{

    const vector<CfgType>& GetQuery() const {return m_query;}
    const vector<CfgType>& GetGoals() const {return m_goals;}

    /// Check whether a path can be drawn through all query points using the
    /// configurations in a given roadmap.
    /// @param[in] _r The roadmap to search.
    /// @return A bool indicating whether a path in _r traversing all goals was
    ///         found.
    virtual bool PerformQuery(RoadmapType* const _r);

    /// Check whether a path connecting a given start and goal exists in the
    /// roadmap.
    /// @param[in] _start The starting configuration to use.
    /// @param[in] _goal  The goal configuration to use.
    /// @return A bool indicating whether the path was found.
    virtual bool PerformSubQuery(const CfgType& _start, const CfgType& _goal) = 0;

    /// Read a Query file.
    /// @param[in] _filename The query file to read.
    void ReadQuery(string _filename);

    /// Output the discovered path to file.
    void WritePath() const;

    /// Reset the path and list of undiscovered goals.
    virtual void Reset(RoadmapType* const _r);

    /// Generate a path through the roadmap from a start node to an end node.
    /// Made public for	disassembly planning.
    /// @param[in] _start The start node.
    /// @param[in] _end The end node.
    std::vector<VID> GeneratePath(const VID _start, const VID _end);

    /// Generate a path through the roadmap from a start node to an end node.
    /// Called when using dynamic obstacles.
    /// @param[in] _start The start node.
    /// @param[in] _end The end node.
    // TODO: Verify that this is correct with LazyPRM
    std::vector<VID> GeneratePathDynamic(const VID _start, const VID _end);
    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Set the search algorithm choice from a string.
    /// @param[in] _alg The search algorithm to use ('astar' or 'dijkstras').
    /// @param[in] _where Error location info in case _alg isn't recognized.
    void SetSearchAlgViaString(string _alg, const string& _where);

    /// Check if a two nodes are connected by the roadmap.
    /// @param[in] _start The starting node's descriptor.
    /// @param[in] _end The ending node's descriptor.
    /// @return True if _start and _goal are connected.
    bool SameCC(const VID _start, const VID _end) const;

    /// Determine whether a vertex is used or has been marked as ignored somehow
    /// (as in lazy query).
    /// @param _vid The vertex ID.
    /// @return True if _vid is considered for paths by this query, false if it
    ///         is marked unused.
    virtual bool IsVertexUsed(const VID _vid) const;

    /// Determine whether an edge is used or has been marked as ignored somehow
    /// (as in lazy query).
    /// @param _eid The edge ID.
    /// @return True if _eid is considered for paths by this query, false if it
    ///         is marked unused.
    virtual bool IsEdgeUsed(const EID _eid) const;

    /// Generate a color map for CC operations which marks the unused vertices
    /// as 'black' (already completed), which causes them to be skipped in graph
    /// operations.
    ColorMap GetColorMap() const;

    /// Generate query configurations from the task constraints.
    void GenerateQuery();

    ///@}
    ///@name Query State
    ///@{

    std::vector<CfgType> m_query;    ///< The start and all goal configurations.
    std::vector<CfgType> m_goals;    ///< The undiscovered goal configurations.

    bool m_fullRecreatePath{true};     ///< Create full paths or just VIDs?

    RoadmapType* m_roadmap{nullptr};   ///< Last roadmap queried.

    ///@}
    ///@name Graph Search
    ///@{

    enum GraphSearchAlg {DIJKSTRAS, ASTAR}; ///< The supported sssp algorithms.
    GraphSearchAlg m_searchAlg{DIJKSTRAS};  ///< The sssp algorithm to use.

    ///@}

    std::string m_safeIntervalLabel;    ///< The XML label for the SafeIntervalTool.

    std::string m_vcLabel;             ///< The validity checker for generating the query.
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
QueryMethod<MPTraits>::
QueryMethod() : MapEvaluatorMethod<MPTraits>() {
  this->SetName("QueryMethod");
}


template <typename MPTraits>
QueryMethod<MPTraits>::
QueryMethod(XMLNode& _node) :
    MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("QueryMethod");

  string searchAlg = _node.Read("graphSearchAlg", false, "dijkstras", "Graph "
      "search algorithm");
  m_fullRecreatePath = _node.Read("fullRecreatePath", false, true, "Whether or "
      "not to recreate path");

  m_safeIntervalLabel = _node.Read("safeIntervalToolLabel", false, "",
      "Label of the SafeIntervalTool");
  
  m_vcLabel = _node.Read("vcLabel", true, "",
      "Label of the Validity Checker for generating the Query");

  m_vcLabel = _node.Read("vcLabel", true, "",
      "Label of the Validity Checker for generating the Query");

  SetSearchAlgViaString(searchAlg, _node.Where());
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
QueryMethod<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << "::"
      << "\n\tSearch Alg: " << m_searchAlg
      << "\n\tFull Paths: " << m_fullRecreatePath << endl;
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
Initialize() {
  // Clear previous state.
  m_query.clear();
  m_goals.clear();

  Reset(nullptr);
}

/*-------------------------- MapEvaluator Interface --------------------------*/

template <typename MPTraits>
bool
QueryMethod<MPTraits>::
operator()() {
  return this->PerformQuery(this->GetRoadmap());
}

/*--------------------------- Query Interface --------------------------------*/

template <typename MPTraits>
bool
QueryMethod<MPTraits>::
PerformQuery(RoadmapType* const _r) {
  if(m_query.empty())
    GenerateQuery();

  if(this->m_debug)
    cout << "Evaluating query, " << m_goals.size() << " goals not connected.\n";

  // If no goals remain, then this must be a refinement step (as in optimal
  // planning). In this case or the roadmap has changed, reinitialize and
  // rebuild the whole path.
  if(m_goals.empty() || _r != m_roadmap)
    Reset(_r);

  // Search for a sequential path through each query point in order.
  for(auto it = m_goals.begin(); it < m_goals.end();) {
    // Start from the last reached query point.
    const auto& start = m_query[m_query.size() - m_goals.size() - 1];
    if(!PerformSubQuery(start, *it))
      return false;
    else
      it = m_goals.erase(it);
  }
  this->GetStatClass()->AddToHistory("pathlength", this->GetPath()->Length());
  WritePath();

  if(this->m_debug)
    cout << "\tConnected all goals!" << endl;

  return true;
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
ReadQuery(string _filename) {
  _filename = MPProblem::GetPath(_filename);
  auto robot = this->GetTask()->GetRobot();
  if(this->m_debug)
    cout << "Reading query file \'" << _filename << "\'. Robot has "
         << robot->GetMultiBody()->DOF() << " DOFs." << endl;

  ifstream in(_filename);
  if(!in.good())
    throw ParseException(WHERE, "Can't open query file '" + _filename + "'.");

  m_query.clear();
  CfgType tempCfg(robot);
  while(in >> tempCfg)
    m_query.push_back(tempCfg);

  if(this->m_debug)
    cout << "\tWe read " << m_query.size() << " cfgs." << endl;
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
WritePath() const {
  if(!this->GetPath())
    return;
  const std::string base = this->GetBaseFilename();
  if(m_fullRecreatePath)
    ::WritePath(base + ".full.path", this->GetPath()->FullCfgs(this->GetMPLibrary()));
  else
    ::WritePath(base + ".rdmp.path", this->GetPath()->Cfgs());
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
Reset(RoadmapType* const _r) {
  // Set the roadmap.
  m_roadmap = _r;

  // Reset the goals.
  m_goals.clear();
  std::copy(m_query.begin() + 1, m_query.end(), back_inserter(m_goals));

  // Reset the path.
  this->GetPath()->Clear();
}

/*------------------------------- Helpers ------------------------------------*/

template <typename MPTraits>
void
QueryMethod<MPTraits>::
SetSearchAlgViaString(string _alg, const string& _where) {
  transform(_alg.begin(), _alg.end(), _alg.begin(), ::tolower);
  if(_alg == "dijkstras")
    m_searchAlg = DIJKSTRAS;
  else if(_alg == "astar")
    m_searchAlg = ASTAR;
  else
    throw ParseException(_where, "Invalid graph search algorithm '" + _alg +
        "'. Choices are 'dijkstras' or 'astar'.");
}


template <typename MPTraits>
bool
QueryMethod<MPTraits>::
SameCC(const VID _start, const VID _end) const {
  if(this->m_debug)
    cout << "\tChecking connectivity..." << endl;

  auto g = this->GetRoadmap()->GetGraph();
  auto stats = this->GetStatClass();

  MethodTimer mt(stats, "QueryMethod::CCTesting");
  stats->IncStat("CC Operations");

  // If either the start or goal is unused, these nodes cannot be connected.
  const bool noStart = !IsVertexUsed(_start),
             noEnd   = !IsVertexUsed(_end);
  if(noStart or noEnd) {
    if(this->m_debug)
      std::cout << "\t\t"
                << (noStart ? "Start" : "")
                << (noStart and noEnd ? " and " : "")
                << (noEnd ? "End" : "")
                << " marked as unused, cannot connect."
                << std::endl;
    return false;
  }

  bool connected = _start == _end;

  // We cannot use stapl's is_same_cc because it offers no way to ignore
  // specific edges. We will instead do a BFS from _start manually and
  // terminate upon either running out of nodes or finding _end.
  std::unordered_map<VID, bool> finished;
  std::queue<VID> queue;
  queue.push(_start);
  while(!connected and !queue.empty())
  {
    // Get the next node in the queue.
    const VID current = queue.front();
    queue.pop();
    finished[current] = true;

    // Enqueue undiscovered children.
    auto vi = g->find_vertex(current);
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      const VID child = ei->target();
      const bool discovered = finished.count(child);

      // Skip if the child is discovered or either child/edge is unused.
      if(discovered or !this->IsEdgeUsed(ei->id()) or !this->IsVertexUsed(child))
        continue;

      // If this is the goal, we are done.
      if(child == _end) {
        connected = true;
        break;
      }

      // Enqueue the child.
      queue.push(child);
      finished[child] = false;
    }
  }

  if(this->m_debug)
    std::cout << "\t\tNodes " << _start << " and " << _end
              << " are " << (connected ? "" : "not ") << "connected."
              << std::endl;

  return connected;
}


template <typename MPTraits>
std::vector<typename QueryMethod<MPTraits>::VID>
QueryMethod<MPTraits>::
GeneratePath(const VID _start, const VID _end) {

  if(!(this->GetMPProblem()->GetDynamicObstacles().empty())){
    std::cout << "Number of Dynamic Obstacles: " << this->GetMPProblem()->GetDynamicObstacles().size() << std::endl;
    return GeneratePathDynamic(_start, _end);
  }

  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "QueryMethod::GraphSearch");
  stats->IncStat("Graph Search");

  // Define types we will need for the graph search.
  using namespace stapl::sequential;
  using namespace stapl;

  typedef typename GraphType::vertex_reference     VR;
  typedef map_property_map<GraphType, VID>         ParentMap;
  //typedef map_property_map<GraphType, WeightType>  DistanceMap;
  typedef DynamicWeightMap<GraphType, WeightType>  DistanceMap;
  typedef std::less<WeightType>                    Comparator;
  typedef std::plus<WeightType>                    Combiner;
  typedef d_ary_heap_indirect<VR, 4, ParentMap, DistanceMap, Comparator> Queue;

  // Build structures for STAPL graph search.
  static const WeightType weightMax("", std::numeric_limits<double>::max()),
                          weightZero("", 0.);
  static const Combiner combiner;
  static const Comparator comparator;
  ParentMap parentMap, indexMap, colorMap = GetColorMap();
  DistanceMap weightMap, distanceMap;

  auto g = this->GetRoadmap()->GetGraph();

  // Initialize the various maps.
  for(auto vi = g->begin(); vi != g->end(); ++vi) {
    parentMap.put(*vi, vi->descriptor());

    // Intialize the distance to this vertex as 0 if we are not using it. This
    // causes it to be skipped by the graph searches. Otherwise initialize to
    // max weight.
    const bool usingThis = IsVertexUsed(vi->descriptor());
    distanceMap.put(*vi, usingThis ? weightMax : weightZero);

    // Initialize the edge weight map for each adjacent edge.
    for(auto ei = vi->begin(); ei != vi->end(); ++ei)
      weightMap.put(*ei, this->IsEdgeUsed(ei->id()) ? ei->property() : weightMax);
  }

  // Make sure the start vertex has zero distance.
  distanceMap.put(_start, weightZero);

  Queue queue(distanceMap, indexMap, comparator);

  // Run the graph search.
  switch(m_searchAlg) {
    case DIJKSTRAS:
      dijkstra_sssp(*g, parentMap, distanceMap, weightMap,
                    _start, _end, comparator, weightMax,
                    combiner, colorMap, queue);
      break;
    case ASTAR:
      Heuristic<MPTraits> heuristic(g->GetVertex(_end),
          this->GetEnvironment()->GetPositionRes(),
          this->GetEnvironment()->GetOrientationRes());
      DistanceMap dummyMap;
      astar(*g, parentMap, distanceMap, dummyMap, weightMap,
            _start, _end, heuristic, comparator,
            combiner, colorMap, queue);
      break;
  }

  // Extract path from sssp results.
  vector<VID> path;
  path.push_back(_end);
  VID temp = _end;
  while(parentMap.get(temp) != temp)
  {
    path.push_back(parentMap.get(temp));
    temp = parentMap.get(temp);
  }

  // A path was found if temp is the start node. In that case reverse it so that
  // it goes from start to goal.
  if(temp == _start)
    std::reverse(path.begin(), path.end());
  else
    path.clear();

  return path;
}

template <typename MPTraits>
std::vector<typename QueryMethod<MPTraits>::VID>
QueryMethod<MPTraits>::
GeneratePathDynamic(const VID _start, const VID _end) {
  /// @TODO Homogenize our custom SSSP implementations. We have them now at
  ///       least in WorkspaceDecomposition, TopologicalMap, and QueryMethod.

  if(_start == _end){
    return std::vector<VID>{_start};
  }

  typedef typename GraphType::adj_edge_iterator   EI;
  typedef std::unordered_map<VID, VID>            ParentMap;

  MethodTimer mt(this->GetStatClass(), "QueryMethod::GeneratePathDynamic");

  auto g = this->GetRoadmap()->GetGraph();

  SafeIntervalTool<MPTraits>* iTool = this->GetMPTools()->GetSafeIntervalTool(m_safeIntervalLabel);

  ParentMap parentMap;
  /// An element in the PQ for dijkstra's. Elements will store
  /// their time, which will be used to check possible edges for
  /// containment within a Safe Interval.
  struct element{

    VID parent;         ///< The parent vertex descriptor.
    VID vd;             ///< This vertex descriptor.
    double distance;    ///< Computed distance (time) at the time of insertion.

    /// Total order by decreasing distance.
    bool operator>(const element& _e) const noexcept {
      return distance > _e.distance;
    }

  };

  // Define a min priority queue for dijkstras. We will not update elements when
  // better distances are found - instead we will trak the most up-to-date
  // distance and ignore elements with different values.
  std::priority_queue<element,
                      std::vector<element>,
                      std::greater<element>> pq;

  std::unordered_map<VID, bool> visited;
  std::unordered_map<VID, double> distance;

  auto relax = [&distance, this, &pq, iTool, g, &parentMap](EI& _ei) {
    const double sourceDistance = distance.count(_ei->source())
                                  ? distance[_ei->source()]
                                  : std::numeric_limits<double>::infinity(),
                 targetDistance = distance.count(_ei->target())
                                  ? distance[_ei->target()]
                                  : std::numeric_limits<double>::infinity(),
                 edgeWeight     = _ei->property().GetTimeSteps(),
                 newDistance    = sourceDistance + edgeWeight;

    if(newDistance >= targetDistance){
      if(this->m_debug)
        std::cout << "Breaking because the path is not optimal." << std::endl;
      return;
    }

    // Ensure that the target vertex is contained within a SafeInterval when
    // arriving.
    auto vertexIntervals = iTool->ComputeIntervals(g->GetVertex(_ei->target()));
    if(!(iTool->ContainsTimestep(vertexIntervals, newDistance))){
      if(this->m_debug){
        std::cout << "Breaking because the vertex is invalid." << std::endl;
        std::cout << "vertexIntervals: " << vertexIntervals << std::endl;
      }
      return;
    }

    // Ensure that the edge is contained within a SafeInterval if leaving now.
    auto edgeIntervals = iTool->ComputeIntervals(_ei->property());
    if(!(iTool->ContainsTimestep(edgeIntervals, sourceDistance))){
      if(this->m_debug)
        std::cout << "Breaking because the edge is invalid." << std::endl;
      return;
    }

    distance[_ei->target()] = newDistance;
    pq.push(element{_ei->source(), _ei->target(), newDistance});
    parentMap[_ei->target()] = _ei->source();
    if(this->m_debug)
      std::cout << "======Relaxed Edge======" << std::endl;
  };

  distance[_start] = 0;
  pq.push(element{_start, _start, 0});
  parentMap[_start] = _start;

  // Dijkstras
  while(!pq.empty()) {
    // Get the next element
    const element current = pq.top();
    pq.pop();

    // If we are done with this node, discard the element.
    if(visited.count(current.vd))
      continue;
    visited[current.vd] = true;

    // Relax edges to unvisited adjacent vertices.
    auto vi = g->find_vertex(current.vd);
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      const size_t target = ei->target();
      if(!visited.count(target))
        relax(ei);
    }
  }

  std::vector<VID> path;
  if(!parentMap.count(_end))
    return path;

  VID current = _end;
  while(current != _start){
    path.push_back(current);
    current = parentMap[current];
  }
  path.push_back(_start);
  std::reverse(path.begin(), path.end());
  return path;
}

template <typename MPTraits>
bool
QueryMethod<MPTraits>::
IsVertexUsed(const VID) const {
  return true;
}


template <typename MPTraits>
bool
QueryMethod<MPTraits>::
IsEdgeUsed(const EID) const {
  return true;
}


template <typename MPTraits>
typename QueryMethod<MPTraits>::ColorMap
QueryMethod<MPTraits>::
GetColorMap() const {
  auto g = this->GetRoadmap()->GetGraph();

  // Define a constant for the color marking.
  static const auto black = stapl::graph_color<size_t>::black(),
                    white = stapl::graph_color<size_t>::white();

  // Mark unused vertices.
  ColorMap colorMap;
  for(auto vi = g->begin(); vi != g->end(); ++vi)
    colorMap.put(*vi, IsVertexUsed(vi->descriptor()) ? white : black);

  return colorMap;
}

template <typename MPTraits>
void
QueryMethod<MPTraits>::
GenerateQuery() {
  // Generate the query configurations.
  /// @TODO Incorporate path constraints when generating the start and goal.
  ///       This should probably be done in the task's representation of the
  ///       start/goal boundaries. I.e., GetStartBoundary should return the
  ///       intersection of the actual start constraint boundary and the path
  ///       constraint boundaries.
  /// @TODO Also incorporate env boundaries. That is not a property of the task
  ///       and should be done here.
  MethodTimer mt(this->GetStatClass(), "QueryMethod::GeneratingQuery");

  auto task = this->GetTask();
  auto robot = task->GetRobot();
  auto startBoundary = task->GetStartConstraint()->GetBoundary();
  const auto& goalConstraints = task->GetGoalConstraints();
  auto vc = this->GetValidityChecker(m_vcLabel);

  // Generate valid query configurations. We will try up to some number of times
  // before giving up and declaring failure.
  static constexpr size_t maxTries = 100;

  // Generate a start configuration if we have a start constraint.
  if(!startBoundary)
    throw RunTimeException(WHERE, "A start constraint is required for query "
        "methods.");

  CfgType start(robot);
  size_t tries = maxTries;
  do {
    start.GetRandomCfg(startBoundary);
    --tries;
    if(tries == 0) {
      std::ostringstream ss;
      ss << "Could not generate valid start configuration in boundary "
         << *startBoundary << " with validity checker '" << m_vcLabel << "'."
         << std::endl;
      throw RunTimeException(WHERE, ss.str());
    }
  } while(!vc->IsValid(start, "QueryMethod::GenerateQuery"));

  if(this->m_debug)
    std::cout << "Tried " << maxTries - tries << " times to generate the start."
              << std::endl;
  m_query.push_back(start);

  // Generate a goal configuration if we have a start constraint.
  if(goalConstraints.empty())
    throw RunTimeException(WHERE, "A goal constraint is required for query "
        "methods.");

  for(const auto& constraint : goalConstraints) {
    auto goalBoundary = constraint->GetBoundary();
    CfgType goal(robot);
    tries = maxTries;
    do {
      goal.GetRandomCfg(goalBoundary);
      --tries;
      if(tries == 0) {
        std::ostringstream ss;
        ss << "Could not generate valid goal configuration in boundary "
           << *goalBoundary << " with validity checker '" << m_vcLabel << "'."
           << std::endl;
        throw RunTimeException(WHERE, ss.str());
      }
    } while(!vc->IsValid(goal, "QueryMethod::GenerateQuery"));

    m_query.push_back(goal);
    if(this->m_debug)
      std::cout << "Tried " << maxTries - tries << " times to generate a goal."
                << std::endl;
  }

  if(m_query.empty())
    throw RunTimeException(WHERE, this->GetNameAndLabel() + "::PerformQuery "
        "error: m_query is empty. This is sometimes caused by reading the wrong "
        "query file in the XML.");
}
/*----------------------------------------------------------------------------*/

#endif
