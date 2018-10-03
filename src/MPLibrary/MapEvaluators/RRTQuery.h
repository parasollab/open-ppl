#ifndef RRT_QUERY_H_
#define RRT_QUERY_H_

#include "QueryMethod.h"

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "Utilities/MetricUtils.h"
#include <containers/sequential/graph/algorithms/astar.h>

////////////////////////////////////////////////////////////////////////////////
/// Evaluate a roadmap under construction to see if a query has been satisfied.
///
/// This query is specialized for RRT methods. The first query point is treated
/// as the start, and the remaining points are treated as sub-goals. This object
/// will attempt to find a path from the start through all sub-goals. If a
/// particular sub-goal is not already in the map, an extension towards it will
/// be attempted if the nearest node is within the provided extender's range.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RRTQuery : public QueryMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    RRTQuery(const std::vector<CfgType>& _queryCfgs = std::vector<CfgType>(),
             const std::vector<CfgType>& _goalCfgs  = std::vector<CfgType>());
    RRTQuery(XMLNode& _node);
    virtual ~RRTQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name QueryMethod Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Query Interface
    ///@{

    /// Get a random unconnected goal.
    const CfgType& GetRandomGoal() const;

    virtual bool PerformSubQuery(const CfgType& _start, const CfgType& _goal)
        override;

    virtual void Reset(RoadmapType* const _r) override;

    ///@}
    ///@name Substrategy helpers
    ///@{
    /// @todo Extract query configuration from 'query' evaluators to an object
    ///       owned by MPSolution. The Query abstraction should support both
    ///       single-cfg and region-based start/goals, and should
    ///       compute/maintain the list of satisfying VIDs.

    void SetQuery(const std::vector<CfgType>& _queryCfgs);

    void SetGoals(const std::vector<CfgType>& _goalCfgs);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Find the nearest node to a goal configuration, assuming that the goal is
    /// not already in the map.
    /// @param[in] _goal The goal node of interest.
    /// @return The descriptor of the map node that is nearest to _goal, or
    ///         INVALID_VID if we have already checked every node in the map. The
    ///         second item is the distance from said node to _goal.
    Neighbor FindNearestNeighbor(const CfgType& _goal);

    /// Find the nearest node to a goal configuration that is already connected
    /// to the start node. The start is assumed to be in the map, but the goal
    /// may or may not be.
    /// @param[in] _start The descriptor of the start node in the map.
    /// @param[in] _goal  The goal configuration, possibly in the map.
    /// @return If the goal isn't in the map, return the descriptor of the map
    ///         node that is nearest to it and their separation distance. If
    ///         the goal is already in the map, return its VID if it is connected
    ///         to the start and INVALID_VID otherwise.
    Neighbor FindNearestConnectedNeighbor(const VID _start,
        const CfgType& _goal);

    /// Extend towards a goal from the nearest node in the map.
    /// @param[in] _nearest The descriptor of the nearest node and its distance
    ///                     to _goal.
    /// @param[in] _goal The goal configuration to extend towards.
    /// @return The descriptor of the newly extended node and its distance to
    ///         _goal, or INVALID_VID if the extension failed.
    Neighbor ExtendToGoal(const Neighbor& _nearest, const CfgType& _goal) const;

    ///@}
    ///@name Query State
    ///@{

    double m_goalDist{0.};      ///< Getting at least this close = success.
    VID m_highestCheckedVID;    ///< The highest VID we have tried to connect to
                                ///< the goal.

    ///@}
    ///@name MP Object Labels
    ///@{

    string m_nfLabel{"Nearest"};       ///< Neighborhood finder label.
    string m_exLabel{"BERO"};          ///< Extender label.

    ///@}
    ///@name Unhide QueryMethod names.
    ///@{

    using QueryMethod<MPTraits>::m_goals;

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(const std::vector<CfgType>& _queryCfgs,
         const std::vector<CfgType>& _goalCfgs) : QueryMethod<MPTraits>() {
  this->SetName("RRTQuery");
  this->m_query = _queryCfgs;
  this->m_goals = _goalCfgs;
}


template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(XMLNode& _node) : QueryMethod<MPTraits>(_node) {
  this->SetName("RRTQuery");

  m_nfLabel = _node.Read("nfLabel", false, m_nfLabel, "Neighborhood finder "
      "method");
  m_exLabel = _node.Read("exLabel", true, m_exLabel, "Extender method");
  m_goalDist = _node.Read("goalDist", false, 0., 0.,
      numeric_limits<double>::max(), "Minimum Distance for valid query");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
RRTQuery<MPTraits>::
Print(ostream& _os) const {
  QueryMethod<MPTraits>::Print(_os);
  _os << "\n\tNeighborhood Finder: " << m_nfLabel
      << "\n\tExtender: " << m_exLabel
      << "\n\tGoal distance: " << m_goalDist << endl;
}


/*---------------------------- QueryMethod Overrides -------------------------*/


template <typename MPTraits>
void
RRTQuery<MPTraits>::
Initialize() {
  if(this->GetTask()) {
    // If we have a task set, then use standard behavior.
    QueryMethod<MPTraits>::Initialize();
  }
  else {
    if(this->m_debug)
      std::cout << "No task detected for RRTQuery, assuming this will be "
          "properly set up in runtime. Right now the use case for this is only "
          "with the RRT LP." << std::endl;
    // Clear previous state, but don't generate query.
    this->m_dmLabel.clear();
    this->m_query.clear();
    this->m_goals.clear();
    Reset(nullptr);
  }
}

/*--------------------------- Query Interface --------------------------------*/

template <typename MPTraits>
const typename MPTraits::CfgType&
RRTQuery<MPTraits>::
GetRandomGoal() const {
  if(m_goals.empty())
    throw RunTimeException(WHERE, "Random goal requested, but none are "
        "available.");
  return m_goals[LRand() % m_goals.size()];
}


template <typename MPTraits>
bool
RRTQuery<MPTraits>::
PerformSubQuery(const CfgType& _start, const CfgType& _goal) {
  if(this->m_debug)
    std::cout << "Evaluating sub-query:"
              << "\n\tfrom " << _start.PrettyPrint()
              << "\n\tto   " << _goal.PrettyPrint()
              << std::endl;

  VID start = this->GetRoadmap()->GetGraph()->GetVID(_start);
  Neighbor nearest;
  bool connected = false;

  // Find the nearest node to _goal that is also connected to _start.
  nearest = FindNearestConnectedNeighbor(start, _goal);

  if(nearest.target == INVALID_VID)
    // If the nearest node is invalid, it means that the goal is in the map and
    // not connected to start. In this case, we can't connect.
    connected = false;
  else if(nearest.distance <= m_goalDist)
    // The nearest node is within the goal distance, so we are close enough.
    connected = true;
  else {
    // The nearest node is too far and the goal isn't already connected. Try to
    // extend toward goal if we are within extender's delta range. If we can't
    // extend, we can't connect.
    nearest = ExtendToGoal(nearest, _goal);
    connected = nearest.target != INVALID_VID and nearest.distance <= m_goalDist;
  }

  // If we have a successful connection through the roadmap, attempt to generate
  // the path.
  if(connected) {
    auto path = this->GeneratePath(start, nearest.target);

    if(!path.empty()) {
      *this->GetPath() += path;
      if(this->m_debug)
        cout << "\tSuccess: found path from start to nearest node "
             << nearest.target << " at a distance of " << nearest.distance
             << " from the goal." << endl;
      return true;
    }
    else if(this->m_debug)
      std::cout << "\tStart and goal are connected, but path is not valid."
                << std::endl;
  }

  if(this->m_debug)
    std::cout << "\tFailed to connect (distance threshold is " << m_goalDist << ")."
              << std::endl;

  return false;
}


template <typename MPTraits>
void
RRTQuery<MPTraits>::
Reset(RoadmapType* const _r) {
  // Reset the highest checked VID if the roadmap has changed.
  if(_r != this->m_roadmap)
    m_highestCheckedVID = 0;
  QueryMethod<MPTraits>::Reset(_r);
}


template <typename MPTraits>
void
RRTQuery<MPTraits>::
SetQuery(const std::vector<CfgType>& _queryCfgs) {
  this->m_query = _queryCfgs;
}

template <typename MPTraits>
void
RRTQuery<MPTraits>::
SetGoals(const std::vector<CfgType>& _goalCfgs) {
  this->m_goals = _goalCfgs;
}

/*------------------------------- Helpers ------------------------------------*/

template <typename MPTraits>
Neighbor
RRTQuery<MPTraits>::
FindNearestNeighbor(const CfgType& _goal) {
  MethodTimer mt(this->GetStatClass(), "RRTQuery::FindNearestNeighbor");

  if(this->m_debug)
    cout << "\t\tFinding the nearest node to the goal..." << endl;

  auto g = this->GetRoadmap()->GetGraph();

  const VID highestVID = (--g->end())->descriptor();

  // Quit if we have already tried with the current set of vertices.
  if(m_highestCheckedVID == highestVID) {
    if(this->m_debug)
      cout << "\t\t\tAll nodes have already been checked." << endl;
    return Neighbor();
  }
  else if(this->m_debug)
    cout << "\t\t\tNodes 0 through " << m_highestCheckedVID
         << " have already been checked." << endl
         << "\t\t\tSearching nodes " << m_highestCheckedVID + 1 << " through "
         << highestVID << "..." << endl;

  // If we haven't tried all vertexes, find the nearest untried vertex.
  // Start by getting the last checked node.
  auto lastChecked = g->find_vertex(m_highestCheckedVID);

  // If the node was deleted, scan backwards through the graph until we find the
  // first node that needs checked.
  if(lastChecked == g->end())
    do {
      --lastChecked;
    } while(lastChecked->descriptor() > m_highestCheckedVID);
  auto firstUnchecked = ++lastChecked;

  // Now check the nodes that haven't been checked yet.
  std::vector<Neighbor> neighbors;
  const bool wholeRoadmap = firstUnchecked == g->begin();
  this->GetNeighborhoodFinder(m_nfLabel)->FindNeighbors(this->GetRoadmap(),
      firstUnchecked, g->end(), wholeRoadmap, _goal,
      std::back_inserter(neighbors));
  m_highestCheckedVID = highestVID;

  if(this->m_debug) {
    if(neighbors.empty())
      std::cout << "\t\t\tNo nearest neighbor found."
                << std::endl;
    else
      std::cout << "\t\t\tFound nearest node " << neighbors.back().target << " at "
                << "distance " << neighbors.back().distance << "."
                << std::endl;
  }

  if(neighbors.empty())
    return Neighbor();
  else
    return neighbors.back();
}


template <typename MPTraits>
Neighbor
RRTQuery<MPTraits>::
FindNearestConnectedNeighbor(const VID _start, const CfgType& _goal) {
  MethodTimer mt(this->GetStatClass(), "RRTQuery::FindNearestConnectedNeighbor");

  if(this->m_debug)
    cout << "\tSearching for the nearest connected neighbor..." << endl;

  auto g = this->GetRoadmap()->GetGraph();
  Neighbor nearest;

  if(g->IsVertex(_goal)) {
    // The goal is already in the roadmap. It is it's own neighbor if it shares
    // a CC with _start and disconnected otherwise.
    VID goal = g->GetVID(_goal);
    if(this->SameCC(_start, goal))
      nearest = Neighbor(goal, 0);
  }
  else
    // If _goal isn't already connected, find the nearest connected node.
    nearest = FindNearestNeighbor(_goal);

  if(this->m_debug) {
    if(nearest.target != INVALID_VID)
      cout << "\t\tClosest neighbor to goal is node " << nearest.target << " at "
           << "distance " << setprecision(4) << nearest.distance << ".\n";
    else
      cout << "\t\tNo valid node was found." << endl;
  }
  return nearest;
}


template <typename MPTraits>
Neighbor
RRTQuery<MPTraits>::
ExtendToGoal(const Neighbor& _nearest, const CfgType& _goal) const {
  MethodTimer mt(this->GetStatClass(), "RRTQuery::ExtendToGoal");

  auto g = this->GetRoadmap()->GetGraph();
  auto e = this->GetExtender(m_exLabel);

  VID newVID = INVALID_VID;
  double distance = numeric_limits<double>::max();

  // If the nearest node is outside the extender's range, return invalid.
  if(_nearest.distance > e->GetMaxDistance())
    return Neighbor(newVID, distance);

  if(this->m_debug)
    cout << "\tTrying extension from node " << _nearest.target
         << " toward goal.\n";

  // Use the NeighborhoodFinder's Distance metric for consistency.
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto dm = this->GetDistanceMetric(nf->GetDMLabel());

  // Otherwise, try to extend from _nearest to _goal.
  CfgType qNew(this->GetTask()->GetRobot());
  LPOutput<MPTraits> lpOutput;
  if(e->Extend(g->GetVertex(_nearest.target), _goal, qNew, lpOutput)) {
    distance = lpOutput.m_edge.first.GetWeight();
    // If we went far enough, add the new node and edge.
    if(distance > e->GetMinDistance() && !g->IsVertex(qNew)) {
      newVID = g->AddVertex(qNew);
      qNew.SetStat("Parent", _nearest.target);
      g->AddEdge(_nearest.target, newVID, lpOutput.m_edge);
      distance = dm->Distance(qNew, _goal);
      if(this->m_debug)
        cout << "\t\tExtension succeeded, created new node " << newVID << " at "
             << "distance " << distance << " from goal." << endl;
    }
    else if(this->m_debug)
      cout << "\t\tExtension failed." << endl;
  }

  return Neighbor(newVID, distance);
}

/*----------------------------------------------------------------------------*/

#endif
