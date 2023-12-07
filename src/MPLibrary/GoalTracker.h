#ifndef PMPL_GOAL_TRACKER_H_
#define PMPL_GOAL_TRACKER_H_

#include "MPLibrary/MPBaseObject.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include <atomic>
#include <map>
#include <unordered_set>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Tracks the set of roadmap configurations which satisfy a task's constraints.
/// Works for both individual and group roadmaps.
///
/// Note that this object should NOT live in the MPSolution because the solution
/// is not tied to a particular task. However it does represent part of the
/// solution, so perhaps we will refactor the MPSolution to contain a roadmap
/// and set of task solutions (with goal tracker and path) for each robot.
////////////////////////////////////////////////////////////////////////////////
template <typename Roadmap, typename Task>
class GoalMap {

  public:

    ///@name Roadmap Types
    ///@{

    typedef typename Roadmap::VI  VI;
    typedef typename Roadmap::VID VID;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::unordered_set<VID> VIDSet;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a goal map for a roadmap/task pair.
    /// @param _r The roadmap to track.
    /// @param _t The task defining start/goal conditions.
    GoalMap(Roadmap* const _r, const Task* const _t);

    ~GoalMap();

    ///@}
    ///@name Interface
    ///@{

    /// Find the VIDs which satisfy the start constraint.
    /// @return The set of VIDs which satisfy the start constraint.
    const VIDSet& GetStartVIDs() const noexcept;

    /// Find the VIDs which satisfy a particular goal constraint.
    /// @param _index The goal constraint index.
    /// @return The set of VIDs which satisfy goal constraint _index.
    const VIDSet& GetGoalVIDs(const size_t _index) const noexcept;

    /// Check the number of unreached goals.
    size_t UnreachedGoalCount() const noexcept;

    /// Get the indexes of the goal constraints which have not yet been
    /// satisfied.
    std::vector<size_t> UnreachedGoalIndexes() const noexcept;

    ///@}
    ///@name Hooks
    ///@{
    /// Note that you must install the hooks *after* the object has settled in
    /// its final location in memory, or the this pointer will be corrupted and
    /// general mayhem will ensue.

    /// Install hooks in the roadmap for mapping new configurations.
    void InstallHooks() noexcept;

    /// Remove hooks from the roadmap.
    void RemoveHooks() noexcept;

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

    /// Get the hook label for this goal map.
    std::string GetHookLabel() const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    static std::atomic<size_t> s_index; ///< Global index for goal maps.

    const size_t m_index;      ///< The index of this map.
    Roadmap* const m_roadmap;  ///< The roadmap being tracked.
    const Task* const m_task;  ///< The task defining start/goals.

    /// The VIDs which satisfy the start constraints.
    VIDSet m_startVIDs;

    /// The VIDs which satisfy each goal constraint.
    std::vector<VIDSet> m_goalVIDs;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

// Initalize the global index.
template <typename Roadmap, typename Task>
std::atomic<size_t> GoalMap<Roadmap, Task>::s_index(0);


template <typename Roadmap, typename Task>
GoalMap<Roadmap, Task>::
GoalMap(Roadmap* const _roadmap, const Task* const _task)
    : m_index(++s_index), m_roadmap(_roadmap), m_task(_task) {
  // If either roadmap or task is null, we cannot build this object.
  if(!_roadmap)
    throw RunTimeException(WHERE) << "A non-null roadmap pointer is required.";
  if(!_task)
    throw RunTimeException(WHERE) << "A non-null task pointer is required.";

  // If we have no start or goal constraints, then this object is not needed and
  // will do nothing.
  if(m_task->Empty())
    return;

  // Initialize the goalVIDs structure with the correct number of constraints.
  m_goalVIDs.resize(m_task->GetNumGoals());

  // Evaluate any existing configurations.
  for(auto vi = m_roadmap->begin(); vi != m_roadmap->end(); ++vi)
    TrackCfg(vi);
}


template <typename Roadmap, typename Task>
GoalMap<Roadmap, Task>::
~GoalMap() {
}

/*-------------------------------- Interface ---------------------------------*/

template <typename Roadmap, typename Task>
const typename GoalMap<Roadmap, Task>::VIDSet&
GoalMap<Roadmap, Task>::
GetStartVIDs() const noexcept {
  return m_startVIDs;
}


template <typename Roadmap, typename Task>
const typename GoalMap<Roadmap, Task>::VIDSet&
GoalMap<Roadmap, Task>::
GetGoalVIDs(const size_t _index) const noexcept {
  if(_index > m_goalVIDs.size())
    throw RunTimeException(WHERE) << "Requested goal VIDs for constraint "
                                  << _index << ", but only "
                                  << m_goalVIDs.size() << " constraints are "
                                  << "present in task '" << m_task->GetLabel()
                                  << "'.";
  return m_goalVIDs[_index];
}


template <typename Roadmap, typename Task>
size_t
GoalMap<Roadmap, Task>::
UnreachedGoalCount() const noexcept {
  size_t count = 0;

  // Add up the number of goal constraints that have no satisfying vids.
  for(const auto& vids : m_goalVIDs)
    count += vids.empty();

  return count;
}


template <typename Roadmap, typename Task>
std::vector<size_t>
GoalMap<Roadmap, Task>::
UnreachedGoalIndexes() const noexcept {
  std::vector<size_t> indexes;

  // Add up the number of goal constraints that have no satisfying vids.
  for(size_t i = 0; i < m_goalVIDs.size(); ++i)
    if(m_goalVIDs[i].empty())
      indexes.push_back(i);

  return indexes;
}

/*---------------------------------- Hooks -----------------------------------*/

template <typename Roadmap, typename Task>
void
GoalMap<Roadmap, Task>::
InstallHooks() noexcept {
  using HT = typename Roadmap::HookType;

  const std::string& label = this->GetHookLabel();

  m_roadmap->InstallHook(HT::AddVertex, label,
      [this](const VI _vi){this->TrackCfg(_vi);});

  m_roadmap->InstallHook(HT::DeleteVertex, label,
      [this](const VI _vi){this->UntrackCfg(_vi);});
}


template <typename Roadmap, typename Task>
void
GoalMap<Roadmap, Task>::
RemoveHooks() noexcept {
  using HT = typename Roadmap::HookType;

  const std::string& label = this->GetHookLabel();

  if(m_roadmap->IsHook(HT::AddVertex, label))
    m_roadmap->RemoveHook(HT::AddVertex, label);

  if(m_roadmap->IsHook(HT::DeleteVertex, label))
    m_roadmap->RemoveHook(HT::DeleteVertex, label);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename Roadmap, typename Task>
void
GoalMap<Roadmap, Task>::
TrackCfg(const VI _vertex) {
  // Get the VID and cfg info.
  const VID vid   = _vertex->descriptor();
  const auto& cfg = _vertex->property();

  // Check for satisfying the start constraint.
  if(m_task->EvaluateStartConstraints(cfg))
    m_startVIDs.insert(vid);

  // Check for satisfying each goal constraint.
  const size_t numGoals = m_task->GetNumGoals();
  for(size_t i = 0; i < numGoals; ++i)
    if(m_task->EvaluateGoalConstraints(cfg, i))
      m_goalVIDs[i].insert(vid);
}


template <typename Roadmap, typename Task>
void
GoalMap<Roadmap, Task>::
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


template <typename Roadmap, typename Task>
std::string
GoalMap<Roadmap, Task>::
GetHookLabel() const noexcept {
  return "GoalMap" + std::to_string(m_index);
}

/*----------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
/// Maintains a set of GoalMap for multiple roadmap/task pairs.
////////////////////////////////////////////////////////////////////////////////
class GoalTrackerType final : MPBaseObject {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType       RoadmapType;
    typedef typename MPBaseObject::GroupRoadmapType  GroupRoadmapType;
    typedef typename RoadmapType::VID                VID;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::unordered_set<VID> VIDSet;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a goal tracker, which must be owned by a specific MPLibrary.
    /// @param _library The owning library.
    GoalTrackerType(MPLibrary* const _library);

    ~GoalTrackerType();

    ///@}
    ///@name Interface
    ///@{

    /// Find the VIDs which satisfy the start constraint for a roadmap/task pair.
    /// @param _roadmap The roadmap.
    /// @param _task The task.
    /// @return The set of VIDs from _roadmap which satisfy the start
    ///         constraints for _task.
    template <typename Roadmap, typename Task>
    const VIDSet& GetStartVIDs(Roadmap* const _roadmap, const Task* const _task)
        const;

    /// This version uses the current roadmap/task.
    /// @overload
    /// @throw If both a group and individual 'current' task are set.
    const VIDSet& GetStartVIDs() const;

    /// Find the VIDs which satisfy a particular goal constraint for a
    /// roadmap/task pair.
    /// @param _roadmap The roadmap.
    /// @param _task The task.
    /// @param _index The goal constraint index.
    /// @return The set of VIDs from _roadmap which satisfy goal constraint
    ///         _index from task _task.
    template <typename Roadmap, typename Task>
    const VIDSet& GetGoalVIDs(Roadmap* const _roadmap, const Task* const _task,
        const size_t _index) const;

    /// This version uses the current roadmap/task.
    /// @overload
    /// @throw If both a group and individual 'current' task are set.
    const VIDSet& GetGoalVIDs(const size_t _index) const;

    /// Check the number of unreached goals for a roadmap/task.
    /// @param _roadmap The roadmap.
    /// @param _task The task.
    /// @return The number of unreached goals.
    template <typename Roadmap, typename Task>
    size_t UnreachedGoalCount(Roadmap* const _roadmap, const Task* const _task)
        const;

    /// This version uses the current roadmap/task.
    /// @overload
    /// @throw If both a group and individual 'current' task are set.
    size_t UnreachedGoalCount() const;

    /// Get the indexes of the goal constraints which have not yet been
    /// satisfied.
    /// @param _roadmap The roadmap.
    /// @param _task The task.
    /// @return The unreached goal indexes.
    template <typename Roadmap, typename Task>
    std::vector<size_t> UnreachedGoalIndexes(Roadmap* const _roadmap,
        const Task* const _task) const;

    /// This version uses the current roadmap/task.
    /// @overload
    /// @throw If both a group and individual 'current' task are set.
    std::vector<size_t> UnreachedGoalIndexes() const;

    /// Add a goal map for a roadmap/task pair.
    /// @param _roadmap The roadmap.
    /// @param _task The task.
    void AddMap(RoadmapType* const _roadmap, const MPTask* const _task);

    /// Add a goal map for a group roadmap/task pair.
    /// @param _roadmap The group roadmap.
    /// @param _task The task.
    void AddMap(GroupRoadmapType* const _roadmap, const GroupTask* const _task);

    /// Check for a goal map for a roadmap/task pair.
    /// @param _roadmap The roadmap.
    /// @param _task The task.
    bool IsMap(RoadmapType* const _roadmap, const MPTask* const _task) const;

    /// Check for a goal map for a group roadmap/task pair.
    /// @param _roadmap The group roadmap.
    /// @param _task The task.
    bool IsMap(GroupRoadmapType* const _roadmap, const GroupTask* const _task)
        const;

    /// Clear all goal maps.
    void Clear();

    ///@}

  private:

    ///@name Internal Types
    ///@{

    typedef std::pair<RoadmapType*, const MPTask*>         IndividualKey;
    typedef std::pair<GroupRoadmapType*, const GroupTask*> GroupKey;

    typedef GoalMap<RoadmapType, MPTask>                   IndividualMap;
    typedef GoalMap<GroupRoadmapType, GroupTask>           GroupMap;

    ///@}
    ///@name Helpers
    ///@{

    const IndividualMap& GetMap(RoadmapType* const _roadmap,
        const MPTask* const _task) const;

    const GroupMap& GetMap(GroupRoadmapType* const _roadmap,
        const GroupTask* const _task) const;

    ///@}
    ///@name Internal State
    ///@{
    /// @note These must be non-reallocating containers for the hook functions
    ///       to work properly.

    std::map<IndividualKey, IndividualMap> m_individualMaps;
    std::map<GroupKey, GroupMap> m_groupMaps;

    ///@}

};

#endif
