#ifndef PMPL_RRT_QUERY_H_
#define PMPL_RRT_QUERY_H_

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

    virtual void Print(std::ostream& _os) const override;

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

    /// Try to connect any untried configurations that are within the extender's
    /// range of the goal.
    /// @param _goal The goal node of interest.
    /// @return The descriptor and goal-distance of the best newly extended node,
    ///         or INVALID_VID if the extensions all failed.
    Neighbor TryConnections(const CfgType& _goal);

    /// Extend towards a goal from a configuration in the map.
    /// @param _vid The descriptor of the configuration to extend from.
    /// @param _goal The goal configuration to extend towards.
    /// @return The descriptor of the newly extended node and its distance to
    ///         _goal, or INVALID_VID if the extension failed.
    Neighbor ExtendToGoal(const VID _vid, const CfgType& _goal) const;

    ///@}
    ///@name Query State
    ///@{

    double m_goalDistance{0.};  ///< Getting at least this close = success.
    VID m_highestCheckedVID;    ///< The highest VID we have tried to connect to
                                ///< the goal.

    ///@}
    ///@name MP Object Labels
    ///@{
    /// @todo m_dmLabel hides a name in the base class. Figure out a better
    ///       naming scheme that avoids collision.

    std::string m_dmLabel;      ///< Distance metric label.
    std::string m_exLabel;      ///< Extender label.

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

  m_dmLabel = _node.Read("dmLabel", false, m_dmLabel, "Distance metric method");
  m_exLabel = _node.Read("exLabel", true, m_exLabel, "Extender method");
  m_goalDistance = _node.Read("goalDist", false, 0., 0.,
      numeric_limits<double>::max(), "Minimum Distance for valid query");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
RRTQuery<MPTraits>::
Print(std::ostream& _os) const {
  QueryMethod<MPTraits>::Print(_os);
  _os << "\n\tDistance Metric: " << m_dmLabel
      << "\n\tExtender: " << m_exLabel
      << "\n\tGoal distance: " << m_goalDistance
      << std::endl;
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

  const VID start = this->GetRoadmap()->GetGraph()->GetVID(_start);

  // Try to connect each untried vertex which is within the extender's range.
  Neighbor best = TryConnections(_goal);
  bool connected = best.source != INVALID_VID
               and best.distance <= m_goalDistance;

  // If none of the connections succeeded, test if the goal is already
  // connected (do the connections first because there won't be many and it works
  // naturally with optimal planning).
  auto g = this->GetRoadmap()->GetGraph();
  if(!connected and g->IsVertex(_goal)) {
    const VID goal = g->GetVID(_goal);
    if(this->SameCC(start, goal))
      connected = true;
  }

  // If we have a successful connection through the roadmap, attempt to generate
  // the path.
  if(connected) {
    auto path = this->GeneratePath(start, best.source);

    if(!path.empty()) {
      *this->GetPath() += path;
      if(this->m_debug)
        std::cout << "\tSuccess: found path from start to nearest node "
                  << best.source << " at a distance of " << best.distance
                  << " from the goal."
                  << std::endl;
      return true;
    }
    else if(this->m_debug)
      std::cout << "\tStart and goal are connected, but path is not valid."
                << std::endl;
  }

  if(this->m_debug)
    std::cout << "\tFailed to connect (distance threshold is " << m_goalDistance
              << ")."
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
TryConnections(const CfgType& _goal) {
  MethodTimer mt(this->GetStatClass(), "RRTQuery::TryConnections");

  if(this->m_debug)
    std::cout << "\tTrying to connect unchecked configurations to the goal..."
              << std::endl;

  auto g = this->GetRoadmap()->GetGraph();

  const VID highestVID = (--g->end())->descriptor();

  // Quit if we have already tried with the current set of vertices.
  if(m_highestCheckedVID == highestVID) {
    if(this->m_debug)
      std::cout << "\t\tAll nodes have already been checked."
                << std::endl;
    return Neighbor();
  }
  else if(this->m_debug)
    std::cout << "\t\tNodes 0 through " << m_highestCheckedVID
              << " have already been checked.\n"
              << "\t\tSearching nodes " << m_highestCheckedVID + 1 << " through "
              << highestVID << "..."
              << std::endl;

  // If we haven't tried all vertexes, find the nearest untried vertex.
  // Start by getting the last checked node.
  auto lastChecked = g->find_vertex(m_highestCheckedVID);

  // If the node was deleted, scan backwards through the graph until we find the
  // first node that needs checked.
  if(lastChecked == g->end())
    do {
      --lastChecked;
    } while(lastChecked->descriptor() > m_highestCheckedVID);
  m_highestCheckedVID = highestVID;

  // Try to extend each untried node towards the goal. Keep track of the closest
  // result. Do not continue to retry on nodes that are created by this process.
  /// @todo This can potentially cause a seg-fault if the roadmap's vertex array
  ///       re-allocate as a result of ExtendToGoal. Correct it so that we are
  ///       not iterating through the graph while it is being expanded.
  Neighbor best;
  auto currentEnd = g->end();
  for(auto unchecked = lastChecked + 1; unchecked != currentEnd; ++unchecked) {
    Neighbor next = ExtendToGoal(unchecked->descriptor(), _goal);
    if(next.distance < best.distance)
      std::swap(next, best);
  }

  if(this->m_debug) {
    if(best.source == INVALID_VID)
      std::cout << "\t\tNo untested node within range found."
                << std::endl;
    else
      std::cout << "\t\tFound nearest node " << best.source << " at "
                << "distance " << std::setprecision(4) << best.distance << "."
                << std::endl;
  }

  return best;
}


template <typename MPTraits>
Neighbor
RRTQuery<MPTraits>::
ExtendToGoal(const VID _vid, const CfgType& _goal) const {
  auto g = this->GetRoadmap()->GetGraph();
  auto e = this->GetExtender(m_exLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);

  // Check distance.
  const CfgType& cfg = g->GetVertex(_vid);
  const double distance = dm->Distance(cfg, _goal);

  // If the nearest node is outside the extender's range, return invalid.
  if(distance > e->GetMaxDistance())
    return Neighbor();
  // If the distance within the goal threshold, we don't need to extend.
  else if(distance < m_goalDistance)
    return Neighbor(_vid, INVALID_VID, distance);

  // Try to extend from _nearest to _goal.
  if(this->m_debug)
    std::cout << "\tTrying extension toward goal from node " << _vid
              << " at distance " << std::setprecision(4) << distance
              << std::endl;

  CfgType qNew(this->GetTask()->GetRobot());
  LPOutput<MPTraits> lpOutput;
  const bool success = e->Extend(cfg, _goal, qNew, lpOutput);

  // If we fail, return an invalid result.
  if(!success) {
    if(this->m_debug)
      std::cout << "\t\tExtension failed." << std::endl;
    return Neighbor();
  }
  qNew.SetStat("Parent", _vid);

  // Try to add the extended node to the map. If already present, return an
  // invalid result.
  const size_t previousSize = g->Size();
  const VID newVID = g->AddVertex(qNew);
  const bool nodeAlreadyExists = g->Size() == previousSize;

  if(nodeAlreadyExists) {
    if(this->m_debug)
      std::cout << "\t\tExtension reproduced existing node " << newVID << "."
                << std::endl;
    return Neighbor();
  }

  // Add the edge to the map.
  g->AddEdge(_vid, newVID, lpOutput.m_edge);

  // Compute the new distance to the goal.
  const double newDistance = dm->Distance(qNew, _goal);
  if(this->m_debug)
    std::cout << "\t\tExtension succeeded, created new node " << newVID
              << " at distance " << newDistance << " from goal."
              << std::endl;

  return Neighbor(newVID, INVALID_VID, newDistance);
}

/*----------------------------------------------------------------------------*/

#endif
