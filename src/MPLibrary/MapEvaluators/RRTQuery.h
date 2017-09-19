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

    RRTQuery();
    RRTQuery(XMLNode& _node);
    virtual ~RRTQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Query Interface
    ///@{

    /// Get a random unconnected goal.
    const CfgType& GetRandomGoal() const;

    /// Check whether a path connecting a given start and goal exists in the
    /// roadmap.
    /// @param[in] _start The starting configuration to use.
    /// @param[in] _goal  The goal configuration to use.
    /// @return A bool indicating whether the path was found.
    virtual bool PerformSubQuery(const CfgType& _start, const CfgType& _goal)
        override;

    /// Reset the path and list of undiscovered goals.
    virtual void Reset(RoadmapType* const _r) override;

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
    pair<VID, double> FindNearestNeighbor(const CfgType& _goal);

    /// Find the nearest node to a goal configuration that is already connected
    /// to the start node. The start is assumed to be in the map, but the goal
    /// may or may not be.
    /// @param[in] _start The descriptor of the start node in the map.
    /// @param[in] _goal  The goal configuration, possibly in the map.
    /// @return If the goal isn't in the map, return the descriptor of the map
    ///         node that is nearest to it and their separation distance. If
    ///         the goal is already in the map, return its VID if it is connected
    ///         to the start and INVALID_VID otherwise.
    pair<VID, double> FindNearestConnectedNeighbor(const VID _start,
        const CfgType& _goal);

    /// Extend towards a goal from the nearest node in the map.
    /// @param[in] _nearest The descriptor of the nearest node and its distance
    ///                     to _goal.
    /// @param[in] _goal The goal configuration to extend towards.
    /// @return The descriptor of the newly extended node and its distance to
    ///         _goal, or INVALID_VID if the extension failed.
    pair<VID,double> ExtendToGoal(const pair<VID, double>& _nearest,
        const CfgType& _goal) const;

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
RRTQuery() : QueryMethod<MPTraits>() {
  this->SetName("RRTQuery");
}


template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(XMLNode& _node) : QueryMethod<MPTraits>(_node) {
  this->SetName("RRTQuery");

  m_nfLabel = _node.Read("nfLabel", false, m_nfLabel, "Neighborhood finder "
      "method");
  m_exLabel = _node.Read("exLabel", true, m_exLabel, "Extender method");
  m_goalDist = _node.Read("goalDist", false, 0., 0.,
      numeric_limits<double>::max(), "Minimun Distance for valid query");
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
    cout << "Evaluating sub-query:" << endl
         << "\tfrom " << _start << endl
         << "\tto   " << _goal << endl;

  VID start = this->GetRoadmap()->GetGraph()->GetVID(_start);
  pair<VID, double> nearest;
  bool success = false;

  // Find the nearest node to _goal that is also connected to _start.
  nearest = FindNearestConnectedNeighbor(start, _goal);

  if(nearest.first == INVALID_VID)
    // If the nearest node is invalid, it means that the goal is in the map and
    // not connected to start. In this case, we can't connect.
    success = false;
  else if(nearest.second <= m_goalDist)
    // The nearest node is within the goal distance, so we are close enough.
    success = true;
  else {
    // The nearest node is too far and the goal isn't already connected. Try to
    // extend toward goal if we are within extender's delta range. If we can't
    // extend, we can't connect.
    nearest = ExtendToGoal(nearest, _goal);
    success = nearest.first != INVALID_VID && nearest.second <= m_goalDist;
  }

  if(success) {
    this->GeneratePath(start, nearest.first);
    if(this->m_debug)
      cout << "\tSuccess: found path from start to nearest node "
           << nearest.first << " at a distance of " << nearest.second
           << " from the goal." << endl;
  }
  else if(this->m_debug)
    cout << "\tFailed to connect (distance threshold is " << m_goalDist << ").\n";

  return success;
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

/*------------------------------- Helpers ------------------------------------*/

template <typename MPTraits>
pair<typename RRTQuery<MPTraits>::VID, double>
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
    return make_pair(INVALID_VID, numeric_limits<double>::max());
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
  vector<pair<VID, double>> neighbors;
  const bool wholeRoadmap = firstUnchecked == g->begin();
  this->GetNeighborhoodFinder(m_nfLabel)->FindNeighbors(this->GetRoadmap(),
      firstUnchecked, g->end(), wholeRoadmap, _goal,
      back_inserter(neighbors));
  m_highestCheckedVID = highestVID;

  if(this->m_debug)
    if(neighbors.empty())
      std::cout << "\t\t\tNo nearest neighbor found."
                << std::endl;
    else
      std::cout << "\t\t\tFound nearest node " << neighbors.back().first << " at "
                << "distance " << neighbors.back().second << "."
                << std::endl;

  if(neighbors.empty())
    return make_pair(INVALID_VID, numeric_limits<double>::max());
  else
    return neighbors.back();
}


template <typename MPTraits>
pair<typename RRTQuery<MPTraits>::VID, double>
RRTQuery<MPTraits>::
FindNearestConnectedNeighbor(const VID _start, const CfgType& _goal) {
  MethodTimer mt(this->GetStatClass(), "RRTQuery::FindNearestConnectedNeighbor");

  if(this->m_debug)
    cout << "\tSearching for the nearest connected neighbor..." << endl;

  auto g = this->GetRoadmap()->GetGraph();
  pair<VID, double> nearest;

  if(g->IsVertex(_goal)) {
    // The goal is already in the roadmap. It is it's own neighbor if it shares
    // a CC with _start and disconnected otherwise.
    VID goal = g->GetVID(_goal);
    if(this->SameCC(_start, goal))
      nearest = make_pair(goal, 0);
    else
      nearest = make_pair(INVALID_VID, numeric_limits<double>::max());
  }
  else
    // If _goal isn't already connected, find the nearest connected node.
    nearest = FindNearestNeighbor(_goal);

  if(this->m_debug) {
    if(nearest.first != INVALID_VID)
      cout << "\t\tClosest neighbor to goal is node " << nearest.first << " at "
           << "distance " << setprecision(4) << nearest.second << ".\n";
    else
      cout << "\t\tNo valid node was found." << endl;
  }
  return nearest;
}


template <typename MPTraits>
pair<typename RRTQuery<MPTraits>::VID, double>
RRTQuery<MPTraits>::
ExtendToGoal(const pair<VID, double>& _nearest, const CfgType& _goal) const {
  MethodTimer mt(this->GetStatClass(), "RRTQuery::ExtendToGoal");

  auto g = this->GetRoadmap()->GetGraph();
  auto e = this->GetExtender(m_exLabel);

  VID newVID = INVALID_VID;
  double distance = numeric_limits<double>::max();

  // If the nearest node is outside the extender's range, return invalid.
  if(_nearest.second > e->GetMaxDistance())
    return make_pair(newVID, distance);

  if(this->m_debug)
    cout << "\tTrying extension from node " << _nearest.first
         << " toward goal.\n";

  // Otherwise, try to extend from _nearest to _goal.
  CfgType qNew(this->GetTask()->GetRobot());
  LPOutput<MPTraits> lpOutput;
  if(e->Extend(g->GetVertex(_nearest.first), _goal, qNew, lpOutput)) {
    distance = lpOutput.m_edge.first.GetWeight();
    // If we went far enough, add the new node and edge.
    if(distance > e->GetMinDistance() && !g->IsVertex(qNew)) {
      newVID = g->AddVertex(qNew);
      qNew.SetStat("Parent", _nearest.first);
      g->AddEdge(_nearest.first, newVID, lpOutput.m_edge);
      // Using the NeighborhoodFinder's Distance metric for consistency.
      distance = this->GetNeighborhoodFinder(m_nfLabel)->GetDMMethod()->
          Distance(qNew, _goal);
      if(this->m_debug)
        cout << "\t\tExtension succeeded, created new node " << newVID << " at "
             << "distance " << distance << " from goal." << endl;
    }
    else if(this->m_debug)
      cout << "\t\tExtension failed." << endl;
  }

  return make_pair(newVID, distance);
}

/*----------------------------------------------------------------------------*/

#endif
