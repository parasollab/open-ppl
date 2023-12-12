#include "MPBaseObject.h"

#include "MPLibrary/MPLibrary.h"

#include <iostream>
#include <string>

/*-------------------------------- Construction ------------------------------*/

MPBaseObject::
MPBaseObject(const std::string& _label, const std::string& _name, bool _debug) :
    m_debug(_debug), m_name(_name), m_label(_label) { }


MPBaseObject::
MPBaseObject(XMLNode& _node) {
  m_label = _node.Read("label", true, "", "Label Identifier");
  m_debug = _node.Read("debug", false, false, "Show run-time debug info?");
}

MPBaseObject::
~MPBaseObject() { }

/*------------------------------------ I/O -----------------------------------*/

void
MPBaseObject::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

/*-------------------------- Name and Label Accessors ------------------------*/

const std::string&
MPBaseObject::
GetName() const {
  return m_name;
}


const std::string&
MPBaseObject::
GetLabel() const {
  return m_label;
}


std::string
MPBaseObject::
GetNameAndLabel() const {
  return m_name + "::" + m_label;
}


void
MPBaseObject::
SetLabel(const std::string& _s) {
  m_label = _s;
}

/*----------------------------- MPLibrary Accessors --------------------------*/

void
MPBaseObject::
SetMPLibrary(MPLibrary* _l) noexcept {
  m_library = _l;
}


MPLibrary*
MPBaseObject::
GetMPLibrary() const noexcept {
  return m_library;
}


bool
MPBaseObject::
IsRunning() const noexcept {
  return m_library->IsRunning();
}

/*------------------------------ Problem Accessors ---------------------------*/

MPProblem*
MPBaseObject::
GetMPProblem() const noexcept {
  return m_library->GetMPProblem();
}


Environment*
MPBaseObject::
GetEnvironment() const noexcept {
  return GetMPProblem()->GetEnvironment();
}


MPTask*
MPBaseObject::
GetTask() const noexcept {
  return m_library->GetTask();
}


GroupTask*
MPBaseObject::
GetGroupTask() const noexcept {
  return m_library->GetGroupTask();
}

/*--------------------------- Solution Accessors -----------------------------*/

MPSolutionType*
MPBaseObject::
GetMPSolution() const noexcept {
  return m_library->GetMPSolution();
}


typename MPBaseObject::RoadmapType*
MPBaseObject::
GetRoadmap(Robot* const _r) const noexcept {
  return m_library->GetRoadmap(_r);
}


typename MPBaseObject::GroupRoadmapType*
MPBaseObject::
GetGroupRoadmap(RobotGroup* const _g) const noexcept {
  return m_library->GetGroupRoadmap(_g);
}


typename MPBaseObject::RoadmapType*
MPBaseObject::
GetBlockRoadmap(Robot* const _r) const noexcept {
  return m_library->GetBlockRoadmap(_r);
}


Path*
MPBaseObject::
GetPath(Robot* const _r) const noexcept {
  return m_library->GetPath(_r);
}


GroupPath*
MPBaseObject::
GetGroupPath(RobotGroup* const _g) const noexcept {
  return m_library->GetGroupPath(_g);
}


StatClass*
MPBaseObject::
GetStatClass() const noexcept {
  return m_library->GetStatClass();
}


LocalObstacleMap*
MPBaseObject::
GetLocalObstacleMap() const noexcept {
  return m_library->GetLocalObstacleMap();
}


const std::string&
MPBaseObject::
GetBaseFilename() const {
  return GetMPProblem()->GetBaseFilename();
}
