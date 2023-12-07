#include "MPSolution.h"

#include <map>
#include <memory>
#include <unordered_map>

/*------------------------------ Construction --------------------------------*/

MPSolutionType::
RobotSolution::
RobotSolution(Robot* const _robot, StatClass* const _stats)
  : freeMap(new RoadmapType(_robot)),
    obstMap(new RoadmapType(_robot)),
    path   (new Path(freeMap.get())),
    lom    (new LocalObstacleMap(_stats)) {
  freeMap->SetCCTracker(_stats);
  obstMap->SetCCTracker(_stats);
}


MPSolutionType::
GroupSolution::
GroupSolution(RobotGroup* const _group, MPSolutionType* const _solution)
  : freeMap(new GroupRoadmapType(_group, _solution)),
    obstMap(new GroupRoadmapType(_group, _solution)),
    path   (new GroupPath(freeMap.get())) { }


MPSolutionType::
MPSolutionType(Robot* const _r)
  : m_robot(_r), m_stats(new StatClass()) {
  m_individualSolutions[m_robot] = RobotSolution(m_robot, m_stats.get());
}


MPSolutionType::
MPSolutionType(RobotGroup* const _g)
  : m_group(_g), m_stats(new StatClass()) {
  // Initialize a solution for each robot in the group.
  for(auto robot : *m_group)
    m_individualSolutions[robot] = RobotSolution(robot, m_stats.get());

  // Initialize a group solution. Must happen after solutions are populated so
  // that the group map can access the individual robot maps.
  m_groupSolutions[m_group] = GroupSolution(_g, this);
}

/*-------------------------------- Modifiers ---------------------------------*/

void
MPSolutionType::
AddRobot(Robot* const _r) noexcept {
  m_robot = _r;

  auto iter = m_individualSolutions.find(_r);
  if(iter != m_individualSolutions.end()) {
    std::cout << "Robot "
              << _r->GetLabel()
              << " is already in solution."
              << std::endl;
    return;
  }
    
  m_individualSolutions[_r] = RobotSolution(_r,m_stats.get());
}


void
MPSolutionType::
SetRobot(Robot* const _r) noexcept {
  if(m_robot == _r)
    return;

  // Move m_robot's solution to match the new pointer.
  auto iter = m_individualSolutions.find(m_robot);
  m_individualSolutions[_r] = std::move(iter->second);
  m_individualSolutions.erase(iter);

  m_robot = _r;

  m_individualSolutions[_r].freeMap->SetRobot(_r);
  m_individualSolutions[_r].obstMap->SetRobot(_r);
  m_individualSolutions[_r].path->FlushCache();
}


void
MPSolutionType::
SetRoadmap(Robot* const _r, RoadmapType* _roadmap) noexcept {
  auto robotSolution = GetRobotSolution(_r);
  if(!robotSolution)
    throw RunTimeException(WHERE) << "Cannot set roadmap for robot that does not "
                                  << "have a solution.";

  robotSolution->freeMap.reset(_roadmap);
  robotSolution->path.reset(new Path(_roadmap));
}


void
MPSolutionType::
SetPath(Robot* const _r, Path* _path) noexcept {
  auto robotSolution = GetRobotSolution(_r);
  if(!robotSolution)
    throw RunTimeException(WHERE) << "Cannot set path for robot that does not "
                                  << "have a solution.";

  robotSolution->path.release();
  robotSolution->path.reset(_path);
}


void
MPSolutionType::
AddRobotGroup(RobotGroup* const _g) noexcept {
  m_group = _g;

  auto iter = m_groupSolutions.find(_g);
  if(iter != m_groupSolutions.end()) {
    std::cout << "Group :"
              << _g->GetLabel()
              << " already in solution." 
              << std::endl;
    return;
  }

  m_groupSolutions[_g] = GroupSolution(_g,this);
}


void
MPSolutionType::
SetGroupRoadmap(RobotGroup* const _g, GroupRoadmapType* _roadmap) noexcept {
  auto groupSolution = GetGroupSolution(_g);
  if(!groupSolution)
    throw RunTimeException(WHERE) << "Cannot set roadmap for group that does not "
                                  << "have a solution.";

  groupSolution->freeMap.reset(_roadmap);
  groupSolution->path.reset(new GroupPath(_roadmap));
}


void
MPSolutionType::
SetGroupPath(RobotGroup* const _g, GroupPath* _path) noexcept {
  auto groupSolution = GetGroupSolution(_g);
  if(!groupSolution)
    throw RunTimeException(WHERE) << "Cannot set path for group that does not "
                                  << "have a solution.";

  groupSolution->path.release();
  groupSolution->path.reset(_path);
}

/*---------------------------- Roadmap Accessors -----------------------------*/

typename MPSolutionType::RoadmapType*
MPSolutionType::
GetRoadmap(Robot* const _r) const noexcept {
  auto s = GetRobotSolution(_r);
  return s ? s->freeMap.get() : nullptr;
}


typename MPSolutionType::RoadmapType*
MPSolutionType::
GetBlockRoadmap(Robot* const _r) const noexcept {
  auto s = GetRobotSolution(_r);
  return s ? s->obstMap.get() : nullptr;
}


Path*
MPSolutionType::
GetPath(Robot* const _r) const noexcept {
  auto s = GetRobotSolution(_r);
  return s ? s->path.get() : nullptr;
}


LocalObstacleMap*
MPSolutionType::
GetLocalObstacleMap(Robot* const _r) const noexcept {
  auto s = GetRobotSolution(_r);
  return s ? s->lom.get() : nullptr;
}


typename MPSolutionType::GroupRoadmapType*
MPSolutionType::
GetGroupRoadmap(RobotGroup* const _g) const noexcept {
  auto s = GetGroupSolution(_g);
  return s ? s->freeMap.get() : nullptr;
}


GroupPath*
MPSolutionType::
GetGroupPath(RobotGroup* const _g) const noexcept {
  auto s = GetGroupSolution(_g);
  return s ? s->path.get() : nullptr;
}


StatClass*
MPSolutionType::
GetStatClass() const noexcept {
  return m_stats.get();
}


Robot*
MPSolutionType::
GetRobot() const noexcept {
  return m_robot;
}


RobotGroup*
MPSolutionType::
GetRobotGroup() const noexcept {
  return m_group;
}

/*--------------------------------- Helpers ----------------------------------*/

const typename MPSolutionType::RobotSolution*
MPSolutionType::
GetRobotSolution(Robot* _r) const noexcept {
  if(!_r)
    _r = m_robot;

  auto iter = m_individualSolutions.find(_r);
  const bool found = iter != m_individualSolutions.end();
  return found ? &iter->second : nullptr;
}


typename MPSolutionType::RobotSolution*
MPSolutionType::
GetRobotSolution(Robot* _r) noexcept {
  if(!_r)
    _r = m_robot;

  auto iter = m_individualSolutions.find(_r);
  const bool found = iter != m_individualSolutions.end();
  return found ? &iter->second : nullptr;
}


const typename MPSolutionType::GroupSolution*
MPSolutionType::
GetGroupSolution(RobotGroup* _g) const noexcept {
  if(!_g)
    _g = m_group;

  auto iter = m_groupSolutions.find(_g);
  const bool found = iter != m_groupSolutions.end();
  return found ? &iter->second : nullptr;
}


typename MPSolutionType::GroupSolution*
MPSolutionType::
GetGroupSolution(RobotGroup* _g) noexcept {
  if(!_g)
    _g = m_group;

  auto iter = m_groupSolutions.find(_g);
  const bool found = iter != m_groupSolutions.end();
  return found ? &iter->second : nullptr;
}
