#include "RobotGroup.h"

#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include <algorithm>


/*------------------------------- Construction -------------------------------*/

RobotGroup::
RobotGroup(MPProblem* const _problem, const std::string& _label,
    const std::vector<Robot*> _robots)
  : m_problem(_problem), m_label(_label), m_robots(_robots) {
  for(size_t i = 0; i < m_robots.size(); ++i)
    m_indexes[m_robots[i]] = i;
}


RobotGroup::
RobotGroup(MPProblem* const _problem, XMLNode& _node) : m_problem(_problem) {
  // A robot group node should have a label and a set of child robots.
  m_label = _node.Read("label", true, "", "The group label.");

  // Parse each child node.
  for(auto& child : _node) {
    std::string name = child.Name();
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);

    // Currently only "Member" children are supported.
    if(name != "member")
      continue;

    const std::string label = child.Read("label", true, "", "The member robot "
        "label.");

    auto memberRobot = m_problem->GetRobot(label);
    m_robots.push_back(memberRobot);
  }

  for(size_t i = 0; i < m_robots.size(); ++i)
    m_indexes[m_robots[i]] = i;
}

/*----------------------------------- Accessors ------------------------------*/

std::string
RobotGroup::
GetLabel() const noexcept {
  return m_label;
}


Robot*
RobotGroup::
GetRobot(const size_t _index) const noexcept {
  if(_index > m_robots.size())
    throw RunTimeException(WHERE, "Request for robot " + std::to_string(_index)
        + " in a group with " + std::to_string(m_robots.size()) + " robots.");

  return m_robots[_index];
}


Robot*
RobotGroup::
GetRobot(const std::string& _label) const noexcept {
  for(auto robot : m_robots)
    if(robot->GetLabel() == _label)
      return robot;
  return nullptr;
}


size_t
RobotGroup::
GetGroupIndex(Robot* const _robot) const noexcept {
  try {
    return m_indexes.at(_robot);
  }
  catch(const std::runtime_error& _e) {
    std::ostringstream oss;
    oss << "Request for index of robot " << _robot
        << " which is not in the group.";
    throw RunTimeException(WHERE, oss.str());
  }
}


size_t
RobotGroup::
Size() const noexcept {
  return m_robots.size();
}

/*-------------------------------- Iteration ---------------------------------*/

RobotGroup::iterator
RobotGroup::
begin() noexcept {
  return m_robots.begin();
}


RobotGroup::iterator
RobotGroup::
end() noexcept {
  return m_robots.end();
}


RobotGroup::const_iterator
RobotGroup::
begin() const noexcept {
  return m_robots.begin();
}


RobotGroup::const_iterator
RobotGroup::
end() const noexcept {
  return m_robots.end();
}

/*----------------------------------- Debug ----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const RobotGroup& _g) {
  _os << "Group " << _g.GetLabel() << " has " << _g.Size() << " robots.";
  for(const auto robot : _g)
    _os << "\n\t" << robot->GetLabel();
  return _os << std::endl;
}

/*----------------------------------------------------------------------------*/
