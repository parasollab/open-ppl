#include "GoalTracker.h"

#include <atomic>
#include <map>
#include <unordered_set>
#include <vector>

/*------------------------------- Construction -------------------------------*/

GoalTrackerType::
GoalTrackerType(MPLibrary* const _library) {
  this->SetName("GoalTracker");
  this->SetMPLibrary(_library);
}


GoalTrackerType::
~GoalTrackerType() {
  Clear();
}

/*-------------------------------- Interface ---------------------------------*/

template <typename Roadmap, typename Task>
const typename GoalTrackerType::VIDSet&
GoalTrackerType::
GetStartVIDs(Roadmap* const _roadmap, const Task* const _task) const {
  return GetMap(_roadmap, _task).GetStartVIDs();
}


const typename GoalTrackerType::VIDSet&
GoalTrackerType::
GetStartVIDs() const {
  auto task = this->GetTask();
  auto groupTask = this->GetGroupTask();

  if(task and groupTask)
    throw RunTimeException(WHERE) << "Cannot determine the current task since "
                                  << "both group (" << groupTask->GetLabel()
                                  << ") and individual (" << task->GetLabel()
                                  << ") tasks are set.";
  else if(task)
    return GetStartVIDs(this->GetRoadmap(), task);
  else if(groupTask)
    return GetStartVIDs(this->GetGroupRoadmap(), groupTask);
  else
    throw RunTimeException(WHERE) << "No current task is set.";
}


template <typename Roadmap, typename Task>
const typename GoalTrackerType::VIDSet&
GoalTrackerType::
GetGoalVIDs(Roadmap* const _roadmap, const Task* const _task, const size_t _index)
    const {
  return GetMap(_roadmap, _task).GetGoalVIDs(_index);
}


const typename GoalTrackerType::VIDSet&
GoalTrackerType::
GetGoalVIDs(const size_t _index) const {
  auto task = this->GetTask();
  auto groupTask = this->GetGroupTask();

  if(task and groupTask)
    throw RunTimeException(WHERE) << "Cannot determine the current task since "
                                  << "both group (" << groupTask->GetLabel()
                                  << ") and individual (" << task->GetLabel()
                                  << ") tasks are set.";
  else if(task)
    return GetGoalVIDs(this->GetRoadmap(), task, _index);
  else if(groupTask)
    return GetGoalVIDs(this->GetGroupRoadmap(), groupTask, _index);
  else
    throw RunTimeException(WHERE) << "No current task is set.";
}


template <typename Roadmap, typename Task>
size_t
GoalTrackerType::
UnreachedGoalCount(Roadmap* const _roadmap, const Task* const _task) const {
  return GetMap(_roadmap, _task).UnreachedGoalCount();
}


size_t
GoalTrackerType::
UnreachedGoalCount() const {
  auto task = this->GetTask();
  auto groupTask = this->GetGroupTask();

  if(task and groupTask)
    throw RunTimeException(WHERE) << "Cannot determine the current task since "
                                  << "both group (" << groupTask->GetLabel()
                                  << ") and individual (" << task->GetLabel()
                                  << ") tasks are set.";
  else if(task)
    return UnreachedGoalCount(this->GetRoadmap(), task);
  else if(groupTask)
    return UnreachedGoalCount(this->GetGroupRoadmap(), groupTask);
  else
    throw RunTimeException(WHERE) << "No current task is set.";
}


template <typename Roadmap, typename Task>
std::vector<size_t>
GoalTrackerType::
UnreachedGoalIndexes(Roadmap* const _roadmap, const Task* const _task) const {
  return GetMap(_roadmap, _task).UnreachedGoalIndexes();
}


std::vector<size_t>
GoalTrackerType::
UnreachedGoalIndexes() const {
  auto task = this->GetTask();
  auto groupTask = this->GetGroupTask();

  if(task and groupTask)
    throw RunTimeException(WHERE) << "Cannot determine the current task since "
                                  << "both group (" << groupTask->GetLabel()
                                  << ") and individual (" << task->GetLabel()
                                  << ") tasks are set.";
  else if(task)
    return UnreachedGoalIndexes(this->GetRoadmap(), task);
  else if(groupTask)
    return UnreachedGoalIndexes(this->GetGroupRoadmap(), groupTask);
  else
    throw RunTimeException(WHERE) << "No current task is set.";
}


void
GoalTrackerType::
AddMap(RoadmapType* const _roadmap, const MPTask* const _task) {
  IndividualKey key{_roadmap, _task};

  // Ensure that we haven't already added this map.
  if(m_individualMaps.count(key)) {
    auto robot = _roadmap->GetRobot();
    throw RunTimeException(WHERE) << "Map for individual robot '"
                                  << robot->GetLabel() << "' (" << robot << "), "
                                  << "task '" << _task->GetLabel() << "' ("
                                  << _task << ") already exists.";
  }

  auto iter = m_individualMaps.emplace(key,
      GoalMap<RoadmapType, MPTask>(_roadmap, _task)).first;
  iter->second.InstallHooks();
}


void
GoalTrackerType::
AddMap(GroupRoadmapType* const _roadmap, const GroupTask* const _task) {
  GroupKey key{_roadmap, _task};

  // Ensure that we haven't already added this map.
  if(m_groupMaps.count(key)) {
    auto group = _roadmap->GetGroup();
    throw RunTimeException(WHERE) << "Map for robot group '"
                                  << group->GetLabel() << "' (" << group << "), "
                                  << "task '" << _task->GetLabel() << "' ("
                                  << _task << ") already exists.";
  }

  auto iter = m_groupMaps.emplace(key,
      GoalMap<GroupRoadmapType, GroupTask>(_roadmap, _task)).first;
  iter->second.InstallHooks();

  // Create maps for individual robots if needed.
  for(auto& task : *_task) {
    auto roadmap = this->GetRoadmap(task.GetRobot());
    if(!IsMap(roadmap, &task))
      AddMap(roadmap, &task);
  }
}


bool
GoalTrackerType::
IsMap(RoadmapType* const _roadmap, const MPTask* const _task) const {
  IndividualKey key{_roadmap, _task};
  return m_individualMaps.count(key);
}


bool
GoalTrackerType::
IsMap(GroupRoadmapType* const _roadmap, const GroupTask* const _task) const {
  GroupKey key{_roadmap, _task};
  return m_groupMaps.count(key);
}


void
GoalTrackerType::
Clear() {
  // Remove hooks.
  for(auto& keyValue : m_individualMaps)
    keyValue.second.RemoveHooks();
  for(auto& keyValue : m_groupMaps)
    keyValue.second.RemoveHooks();

  // Clear maps.
  m_individualMaps.clear();
  m_groupMaps.clear();
}

/*--------------------------------- Helpers ----------------------------------*/

const typename GoalTrackerType::IndividualMap&
GoalTrackerType::
GetMap(RoadmapType* const _roadmap, const MPTask* const _task) const {
  // Ensure neither argument is null.
  if(!_roadmap)
    throw RunTimeException(WHERE) << "A non-null roadmap pointer is required.";
  if(!_task)
    throw RunTimeException(WHERE) << "A non-null task pointer is required.";

  IndividualKey key{_roadmap, _task};
  try {
    return m_individualMaps.at(key);
  }
  catch(const std::out_of_range&) {
    auto robot = _roadmap->GetRobot();
    throw RunTimeException(WHERE) << "Map for individual robot '"
                                  << robot->GetLabel() << "' (" << robot << "), "
                                  << "task '" << _task->GetLabel() << "' ("
                                  << _task << ") does not exist.";
  }
}


const typename GoalTrackerType::GroupMap&
GoalTrackerType::
GetMap(GroupRoadmapType* const _roadmap, const GroupTask* const _task) const {
  // Ensure neither argument is null.
  if(!_roadmap)
    throw RunTimeException(WHERE) << "A non-null roadmap pointer is required.";
  if(!_task)
    throw RunTimeException(WHERE) << "A non-null task pointer is required.";

  GroupKey key{_roadmap, _task};
  try {
    return m_groupMaps.at(key);
  }
  catch(const std::out_of_range&) {
    auto group = _roadmap->GetGroup();
    throw RunTimeException(WHERE) << "Map for robot group '"
                                  << group->GetLabel() << "' (" << group << "), "
                                  << "task '" << _task->GetLabel() << "' ("
                                  << _task << ") does not exist.";
  }
}

/*----------------------------------------------------------------------------*/
