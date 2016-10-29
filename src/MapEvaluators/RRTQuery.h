#ifndef RRT_QUERY_H_
#define RRT_QUERY_H_

#include "QueryMethod.h"

#include "Utilities/MetricUtils.h"
#include <containers/sequential/graph/algorithms/astar.h>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief Evaluate a roadmap under construction to see if a query has been
///        satisfied.
/// @tparam MPTraits Motion planning universe
///
/// This query is specialized for RRT methods. The first query point is treated
/// as the start, and the remaining points are treated as sub-goals. This object
/// will attempt to find a path from the start through all sub-goals. If a
/// particular sub-goal is not already in the map, an extension towards it will
/// be attempted if the nearest node is within the provided extender's range.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RRTQuery : public QueryMethod<MPTraits> {

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
    ///\name Construction
    ///@{

    RRTQuery();
    RRTQuery(MPProblemType* _problem, XMLNode& _node);
    virtual ~RRTQuery() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const override;

    ///@}
    ///\name Query Interface
    ///@{

    const CfgType& GetRandomGoal() const {
      if(m_goals.empty())
        throw RunTimeException(WHERE, "Random goal requested, but none are "
            "available.");
      return m_goals[LRand() % m_goals.size()];
    }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Check whether a path connecting a given start and goal exists in
    ///        the roadmap.
    /// \param[in] _start The starting configuration to use.
    /// \param[in] _goal  The goal configuration to use.
    /// \return A bool indicating whether the path was found.
    virtual bool PerformSubQuery(const CfgType& _start, const CfgType& _goal)
        override;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Reset the path and list of undiscovered goals.
    /// \param[in] _r The roadmap to use.
    virtual void Initialize(RoadmapType* _r = nullptr) override;

    ///@}

  protected:

    ///\name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Find the nearest node to a goal configuration, assuming that the
    ///        goal is not already in the map.
    /// \param[in] _goal The goal node of interest.
    /// \return The descriptor of the map node that is nearest to _goal, or
    ///         INVALID_VID if we have already checked every node in the map. The
    ///         second item is the distance from said node to _goal.
    pair<VID, double> FindNearestNeighbor(const CfgType& _goal);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Find the nearest node to a goal configuration that is already
    ///        connected to the start node. The start is assumed to be in the map,
    ///        but the goal may or may not be.
    /// \param[in] _start The descriptor of the start node in the map.
    /// \param[in] _goal  The goal configuration, possibly in the map.
    /// \return If the goal isn't in the map, return the descriptor of the map
    ///         node that is nearest to it and their separation distance. If
    ///         the goal is already in the map, return its VID if it is connected
    ///         to the start and INVALID_VID otherwise.
    pair<VID, double> FindNearestConnectedNeighbor(const VID _start,
        const CfgType& _goal);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Extend towards a goal from the nearest node in the map.
    /// \param[in] _nearest The descriptor of the nearest node and its distance
    ///                     to _goal.
    /// \param[in] _goal The goal configuration to extend towards.
    /// \return The descriptor of the newly extended node and its distance to
    ///         _goal, or INVALID_VID if the extension failed.
    pair<VID,double> ExtendToGoal(const pair<VID, double>& _nearest,
        const CfgType& _goal) const;

    ///@}
    ///\name Query State
    ///@{

    double m_goalDist{0.};      ///< Getting at least this close = success.
    VID m_highestCheckedVID;    ///< The highest VID we have tried to connect to
                                ///< the goal.

    ///@}
    ///\name MP Object Labels
    ///@{

    string m_nfLabel{"Nearest"};       ///< Neighborhood finder label.
    string m_exLabel{"BERO"};          ///< Extender label.

    ///@}
    ///\name Unhide QueryMethod names.
    ///@{

    using QueryMethod<MPTraits>::m_goals;
    using QueryMethod<MPTraits>::m_path;

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
RRTQuery(MPProblemType* _problem, XMLNode& _node) :
    QueryMethod<MPTraits>(_problem, _node) {
  this->SetName("RRTQuery");
  ParseXML(_node);
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
RRTQuery<MPTraits>::
ParseXML(XMLNode& _node) {
  m_nfLabel = _node.Read("nfLabel", false, m_nfLabel, "Neighborhood finder "
      "method");
  m_exLabel = _node.Read("exLabel", true, m_exLabel, "Extender method");
  m_goalDist = _node.Read("goalDist", false, 0., 0.,
      numeric_limits<double>::max(), "Minimun Distance for valid query");
}


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
bool
RRTQuery<MPTraits>::
PerformSubQuery(const CfgType& _start, const CfgType& _goal) {
  if(this->m_debug)
    cout << "Evaluating sub-query:" << endl
         << "\tfrom " << _start << endl
         << "\tto   " << _goal << endl;

  VID start = m_path->GetRoadmap()->GetGraph()->GetVID(_start);
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
    cout << "\tFailed to connect (goal distance is " << m_goalDist << ").\n";

  return success;
}


template <typename MPTraits>
void
RRTQuery<MPTraits>::
Initialize(RoadmapType* _r) {
  QueryMethod<MPTraits>::Initialize(_r);
  m_highestCheckedVID = 0;
}

/*------------------------------- Helpers ------------------------------------*/


template <typename MPTraits>
pair<typename RRTQuery<MPTraits>::VID, double>
RRTQuery<MPTraits>::
FindNearestNeighbor(const CfgType& _goal) {
  if(this->m_debug)
    cout << "\t\tFinding the nearest node to the goal..." << endl;

  auto g = m_path->GetRoadmap()->GetGraph();

  VID highestVID = g->get_num_vertices() - 1;

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

  auto stats = this->GetStatClass();

  // If we haven't tried all vertexes, find the nearest untried vertex.
  stats->StartClock("RRTQuery::NeighborhoodFinding");
  vector<pair<VID, double>> neighbors;
  this->GetNeighborhoodFinder(m_nfLabel)->FindNeighbors(m_path->GetRoadmap(),
      ++g->find_vertex(m_highestCheckedVID), g->end(), true, _goal,
      back_inserter(neighbors));
  m_highestCheckedVID = highestVID;
  stats->StopClock("RRTQuery::NeighborhoodFinding");

  if(this->m_debug)
    cout << "\t\t\tFound nearest node " << neighbors.back().first << " at "
         << "distance " << neighbors.back().second << "." << endl;

  return neighbors.back();
}


template <typename MPTraits>
pair<typename RRTQuery<MPTraits>::VID, double>
RRTQuery<MPTraits>::
FindNearestConnectedNeighbor(const VID _start, const CfgType& _goal) {
  if(this->m_debug)
    cout << "\tSearching for the nearest connected neighbor..." << endl;

  auto g = m_path->GetRoadmap()->GetGraph();
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
  auto g = m_path->GetRoadmap()->GetGraph();
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
  CfgType qNew;
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
