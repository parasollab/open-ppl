#ifndef PMPL_GROUP_QUERY_H_
#define PMPL_GROUP_QUERY_H_

#include "QueryMethod.h"

#include "Utilities/MetricUtils.h"
#include "Utilities/SSSP.h"


////////////////////////////////////////////////////////////////////////////////
/// Evaluates the current group roadmap to determine whether a (group) path
/// exists which satisfies the current group task.
///
/// @todo There is a lot of copy-pasted code from QueryMethod because that
///       method calls several other objects which are not ready for groups yet.
///       We should be able to merge these two after a bit more flush-out of the
///       group support. I have marked it final to remind us that we should
///       not extend this as it will be merged.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupQuery final : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType       CfgType;
    typedef typename MPTraits::GroupRoadmapType   GroupRoadmapType;
    typedef typename MPTraits::GroupWeightType    WeightType;
    typedef typename GroupRoadmapType::VID        VID;
    typedef typename MPTraits::GoalTracker        GoalTracker;
    typedef typename GoalTracker::VIDSet          VIDSet;

    ///@}
    ///@name Construction
    ///@{

    GroupQuery();
    GroupQuery(XMLNode& _node);
    virtual ~GroupQuery() = default;

    ///@}
    ///@name MapEvaluator Overrides
    ///@{

    virtual bool operator()() override;

    ///@}
    ///@name Query Interface
    ///@{

    std::vector<VID> GeneratePath(const VID _start, const VIDSet& _end);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    void Reset(GroupRoadmapType* const _r);

    bool PerformSubQuery(const VID _start, const VIDSet& _goal);

    double StaticPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    double DynamicPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    ///@}
    ///@name Internal State
    ///@{

    GroupRoadmapType* m_roadmap{nullptr};

    size_t m_goalIndex{0};             ///< Index of next unreached goal.

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
GroupQuery<MPTraits>::
GroupQuery() : MapEvaluatorMethod<MPTraits>() {
  this->SetName("GroupQuery");
}


template <typename MPTraits>
GroupQuery<MPTraits>::
GroupQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("GroupQuery");
}

/*-------------------------- MapEvaluator Overrides --------------------------*/

template <typename MPTraits>
bool
GroupQuery<MPTraits>::
operator()() {
  auto goalTracker = this->GetGoalTracker();
  const std::vector<size_t> unreachedGoals = goalTracker->UnreachedGoalIndexes();
  auto task = this->GetGroupTask();
  const size_t numGoals = task->GetNumGoals();

  // If no goals remain, then this must be a refinement step (as in optimal
  // planning). In this case or the roadmap has changed, reinitialize and
  // rebuild the whole path.
  auto r = this->GetGroupRoadmap();
  if(unreachedGoals.empty() or r != m_roadmap)
    Reset(r);

  if(this->m_debug)
    std::cout << "Querying group roadmap for a path satisfying task '"
              << task->GetLabel()
              << "', " << unreachedGoals.size() << " / " << numGoals
              << " goals not reached."
              << "\n\tTrying to connect goal: " << m_goalIndex
              << "\n\tUnreached goals: " << unreachedGoals
              << std::endl;

  // Search for a sequential path through each task constraint in order.
  auto path = this->GetGroupPath();
  for(; m_goalIndex < numGoals; ++m_goalIndex) {
    // If this goal constraint is unreached, the query cannot succeed.
    auto iter = std::find(unreachedGoals.begin(), unreachedGoals.end(),
        m_goalIndex);
    const bool unreached = iter != unreachedGoals.end();
    if(unreached) {
      if(this->m_debug)
        std::cout << "\tGoal " << m_goalIndex << " has no satisfying VIDs."
                  << std::endl;
      return false;
    }

    // Get the start VID for this subquery.
    const VID start = path->Empty() ? *goalTracker->GetStartVIDs().begin()
                                    : path->VIDs().back();

    // Get the goal VIDs for this subquery.
    const VIDSet& goals = goalTracker->GetGoalVIDs(m_goalIndex);
    if(goals.empty())
      throw RunTimeException(WHERE) << "No VIDs located for reached goal "
                                    << m_goalIndex << ".";

    if(this->m_debug)
      std::cout << "\tEvaluating sub-query from " << start << " to " << goals
                << "." << std::endl;

    // Warn users if multiple goals are found.
    if(goals.size() > 1 and numGoals > 1)
      std::cerr << "\tWarning: subquery has " << goals.size() << " possible VIDs "
                << "for goal " << m_goalIndex << "/" << numGoals
                << ". The algorithm will try its best but isn't complete for "
                << "this case." << std::endl;

    // Perform this subquery. If it fails, there is no path.
    const bool success = PerformSubQuery(start, goals);
    if(!success)
      return false;
  }

  // We generated a path successfully: track the path length history.
  this->GetStatClass()->AddToHistory("pathlength", path->Length());

  if(this->m_debug)
    std::cout << "\tConnected all goals!" << std::endl;

  return true;
}

/*--------------------------- Query Interface --------------------------------*/

template <typename MPTraits>
std::vector<typename GroupQuery<MPTraits>::VID>
GroupQuery<MPTraits>::
GeneratePath(const VID _start, const VIDSet& _goals) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "GroupQuery::GeneratePath");

  if(this->m_debug)
    std::cout << "Generating path from " << _start << " to " << _goals << "."
              << std::endl;

  // Check for trivial path.
  if(_goals.count(_start))
    return {_start};

  stats->IncStat("Graph Search");

  // Set up the termination criterion to quit early if we find the _end node.
  SSSPTerminationCriterion<GroupRoadmapType> termination(
      [_goals](typename GroupRoadmapType::vertex_iterator& _vi,
             const SSSPOutput<GroupRoadmapType>& _sssp) {
        return _goals.count(_vi->descriptor()) ? SSSPTermination::EndSearch
                                               : SSSPTermination::Continue;
      }
  );

  // Set up the path weight function depending on whether we have any dynamic
  // obstacles.
  SSSPPathWeightFunction<GroupRoadmapType> weight;
  if(!this->GetMPProblem()->GetDynamicObstacles().empty()) {
    weight = [this](typename GroupRoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->DynamicPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }
  else {
    weight = [this](typename GroupRoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->StaticPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }

  // Run dijkstra's algorithm to find the path, if it exists.
  auto g = this->GetGroupRoadmap();
  const SSSPOutput<GroupRoadmapType> sssp = DijkstraSSSP(g, {_start}, weight,
      termination);

  // Find the last discovered node, which should be a goal if there is a valid
  // path.
  const VID last = sssp.ordering.back();
  if(!_goals.count(last))
    return {};

  // Extract the path.
  std::vector<VID> path;
  path.push_back(last);

  VID current = last;
  do {
    current = sssp.parent.at(current);
    path.push_back(current);
  } while(current != _start);
  std::reverse(path.begin(), path.end());

  return path;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
GroupQuery<MPTraits>::
Reset(GroupRoadmapType* const _r) {
  // Set the roadmap.
  m_roadmap = _r;

  // Reset the goal index.
  m_goalIndex = 0;

  // Reset the path.
  auto path = this->GetGroupPath();
  if(path)
    path->Clear();
}


template <typename MPTraits>
bool
GroupQuery<MPTraits>::
PerformSubQuery(const VID _start, const VIDSet& _goals) {
  // Try to generate a path from _start to _goal.
  auto path = this->GeneratePath(_start, _goals);

  // If the path isn't empty, we succeeded.
  if(!path.empty()) {
    *this->GetGroupPath() += path;
    const VID goalVID = path.back();

    if(this->m_debug)
      std::cout << "\tSuccess: reached goal node " << goalVID << "."
                << std::endl;

    return true;
  }
  else if(this->m_debug)
    std::cout << "\tFailed: could not find a path." << std::endl;

  return false;
}


template <typename MPTraits>
double
GroupQuery<MPTraits>::
StaticPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  const double edgeWeight  = _ei->property().GetWeight(),
               newDistance = _sourceDistance + edgeWeight;
  return newDistance;
}


template <typename MPTraits>
double
GroupQuery<MPTraits>::
DynamicPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  throw NotImplementedException(WHERE) << "This needs to be implemented to "
                                       << "consider planning for a group in the "
                                       << "presence of dynamic obstacles.";
}

/*----------------------------------------------------------------------------*/

#endif
