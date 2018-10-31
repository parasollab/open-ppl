#ifndef PMPL_GOAL_TRACKER_H_
#define PMPL_GOAL_TRACKER_H_

#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/MPTask.h"

#include <unordered_set>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Tracks the list of roadmap configurations which satisfy a task's
/// constraints.
///
/// Note that this object should NOT live in the MPSolution because the solution
/// is not tied to a particular task. However it does represent part of the
/// solution, so perhaps we will refactor the MPSolution to contain a roadmap
/// and set of task solutions (with goal tracker and path) for each robot.
////////////////////////////////////////////////////////////////////////////////
template <typename RoadmapType>
class GoalTrackerType final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename RoadmapType::GraphType GraphType;
    typedef typename GraphType::VI          VI;
    typedef typename GraphType::VID         VID;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::unordered_set<VID> VIDSet;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a goal tracker.
    /// @param _roadmap The roadmap.
    /// @param _task The task with goals to be tracked.
    GoalTrackerType(RoadmapType* const _roadmap, const MPTask* const _task);

    ~GoalTrackerType();

    ///@}
    ///@name Interface
    ///@{

    /// Find the VIDs which satisfy the start constraint.
    /// @return The set of VIDs which satisfy the start constraint.
    const VIDSet& GetStartVIDs() const;

    /// Find the VIDs which satisfy a particular goal constraint.
    /// @param _index The goal constraint index.
    /// @return The set of VIDs which satisfy goal constraint _index.
    const VIDSet& GetGoalVIDs(const size_t _index) const;

    /// Check the number of unreached goals.
    size_t UnreachedGoalCount() const;

    /// Get the indexes of the goal constraints which have not yet been
    /// satisfied.
    std::vector<size_t> UnreachedGoalIndexes() const;

    /// Reset this object for a new roadmap and task. Set both to null to just
    /// unhook it from the current roadmap.
    /// @param _roadmap The new roadmap.
    /// @param _task The new task.
    void Reset(RoadmapType* const _roadmap = nullptr,
        const MPTask* const _task = nullptr);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Compute and track the task satisfaction of a new roadmap configuration.
    /// @param _vertex A roadmap graph iterator to the newly added vertex.
    void TrackCfg(const VI _vertex);

    /// Release the task satisfaction info of a to-be-deleted roadmap
    /// configuration.
    /// @param _vertex A roadmap graph iterator to the to-be-deleted vertex.
    void UntrackCfg(const VI _vertex);

    ///@}
    ///@name Internal State
    ///@{

    RoadmapType* m_roadmap{nullptr};  ///< The roadmap being tracked.
    const MPTask* m_task{nullptr};    ///< The task defining start/goals.

    /// The VIDs which satisfy the start constraints.
    VIDSet m_startVIDs;

    /// The VIDs which satisfy each goal constraint.
    std::vector<VIDSet> m_goalVIDs;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename RoadmapType>
GoalTrackerType<RoadmapType>::
GoalTrackerType(RoadmapType* const _roadmap, const MPTask* const _task) {
  Reset(_roadmap, _task);
}


template <typename RoadmapType>
GoalTrackerType<RoadmapType>::
~GoalTrackerType() {
  Reset(nullptr, nullptr);
}

/*-------------------------------- Interface ---------------------------------*/

template <typename RoadmapType>
const typename GoalTrackerType<RoadmapType>::VIDSet&
GoalTrackerType<RoadmapType>::
GetStartVIDs() const {
  return m_startVIDs;
}


template <typename RoadmapType>
const typename GoalTrackerType<RoadmapType>::VIDSet&
GoalTrackerType<RoadmapType>::
GetGoalVIDs(const size_t _index) const {
  if(_index > m_goalVIDs.size())
    throw RunTimeException(WHERE) << "Requested goal VIDs for constraint "
                                  << _index << ", but only "
                                  << m_goalVIDs.size() << " constraints are "
                                  << "present.";
  return m_goalVIDs[_index];
}


template <typename RoadmapType>
size_t
GoalTrackerType<RoadmapType>::
UnreachedGoalCount() const {
  size_t count = 0;

  // Add up the number of goal constraints that have no satisfying vids.
  for(const auto& vids : m_goalVIDs)
    count += vids.empty();

  return count;
}


template <typename RoadmapType>
std::vector<size_t>
GoalTrackerType<RoadmapType>::
UnreachedGoalIndexes() const {
  std::vector<size_t> indexes;

  // Add up the number of goal constraints that have no satisfying vids.
  for(size_t i = 0; i < m_goalVIDs.size(); ++i)
    if(m_goalVIDs[i].empty())
      indexes.push_back(i);

  return indexes;
}


template <typename RoadmapType>
void
GoalTrackerType<RoadmapType>::
Reset(RoadmapType* const _roadmap, const MPTask* const _task) {
  using HT = typename GraphType::HookType;

  // Clear everything from the maps.
  m_startVIDs.clear();
  m_goalVIDs.clear();

  // Remove hooks from old roadmap if needed.
  if(m_roadmap) {
    auto g = m_roadmap->GetGraph();

    if(g->IsHook(HT::AddVertex, "GoalTracker"))
      g->RemoveHook(HT::AddVertex, "GoalTracker");
    if(g->IsHook(HT::DeleteVertex, "GoalTracker"))
      g->RemoveHook(HT::DeleteVertex, "GoalTracker");
  }

  // Set the new pointers. If either is null, save neither and quit.
  const bool null = !_roadmap or !_task;
  if(null) {
    m_roadmap = nullptr;
    m_task    = nullptr;
    return;
  }

  m_roadmap = _roadmap;
  m_task    = _task;

  // If we have no start or goal constraints this object is not needed and will
  // do nothing.
  if(!m_task->GetStartConstraint() and m_task->GetGoalConstraints().empty())
    return;

  // Otherwise, install hooks to track goal satsifaction.
  auto g = m_roadmap->GetGraph();
  g->InstallHook(HT::AddVertex, "GoalTracker",
      [this](const VI _vi){this->TrackCfg(_vi);});
  g->InstallHook(HT::DeleteVertex, "GoalTracker",
      [this](const VI _vi){this->UntrackCfg(_vi);});

  // Initialize the goalVIDs structure with the correct number of constraints.
  m_goalVIDs.resize(m_task->GetGoalConstraints().size());

  // Evaluate any existing configurations.
  for(auto vi = g->begin(); vi != g->end(); ++vi)
    TrackCfg(vi);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename RoadmapType>
void
GoalTrackerType<RoadmapType>::
TrackCfg(const VI _vertex) {
  // Get the VID and cfg info.
  const VID vid   = _vertex->descriptor();
  const auto& cfg = _vertex->property();

  // Check for satisfying the start constraint.
  if(m_task->GetStartConstraint()->Satisfied(cfg))
    m_startVIDs.insert(vid);

  // Check for satisfying each goal constraint.
  const auto& goals = m_task->GetGoalConstraints();
  for(size_t i = 0; i < goals.size(); ++i) {
    if(!goals[i]->Satisfied(cfg))
      continue;
    m_goalVIDs[i].insert(vid);
  }
}


template <typename RoadmapType>
void
GoalTrackerType<RoadmapType>::
UntrackCfg(const VI _vertex) {
  // Get the VID info.
  const VID vid = _vertex->descriptor();

  // Untrack satisfaction of the start constraint.
  if(m_startVIDs.count(vid))
    m_startVIDs.erase(vid);

  // Untrack satisfaction of the goal constraints.
  for(auto& vids : m_goalVIDs)
    if(vids.count(vid))
      vids.erase(vid);
}

/*----------------------------------------------------------------------------*/

#endif
