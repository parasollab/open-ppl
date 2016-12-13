#include "MPProblem.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/MPUtils.h"
#include "Utilities/PMPLExceptions.h"

using namespace std;


/*---------------------------- Construction ----------------------------------*/

MPProblem::
MPProblem() = default;


MPProblem::
MPProblem(const string& _filename) {
  ReadXMLFile(_filename);
}


MPProblem::
~MPProblem() {
  delete m_environment;
  m_environment = nullptr;

  for(auto& robot : m_robots)
    delete robot;
  m_robots.clear();
}

/*---------------------------- XML Helpers -----------------------------------*/

const std::string&
MPProblem::
GetXMLFilename() const {
  return m_xmlFilename;
}


void
MPProblem::
ReadXMLFile(const string& _filename) {
  const bool envIsSet = m_environment;

  size_t sl = _filename.rfind("/");
  m_filePath = _filename.substr(0, sl == string::npos ? 0 : sl + 1);
  m_xmlFilename = _filename;

  // Open the XML and get the root and input nodes.
  XMLNode mpNode(_filename, "MotionPlanning");
  XMLNode input(_filename, "Problem");

  // Parse the input node to set the environment, robot(s), and query.
  if(!envIsSet)
    for(auto& child : input)
      ParseChild(child);

  // Print XML details if requested.
  const bool print = mpNode.Read("print", false, false, "Print all XML input");
  if(print)
    Print(cout);

  // Handle XML warnings/errors.
  const bool warnings = mpNode.Read("warnings", false, false, "Report warnings");
  if(warnings) {
    const bool warningsAsErrors = mpNode.Read("warningsAsErrors", false, false,
        "XML warnings considered errors");
    if(!envIsSet)
      input.WarnAll(warningsAsErrors);
  }

  // Make sure there is an environment and a robot.
  if(!m_environment)
    throw ParseException(input.Where(), "No environment specified in the "
        "problem node.");
  if(m_robots.empty())
    throw ParseException(input.Where(), "No robots specified in the problem "
        "node.");

  // Initialize the Cfg robot pointers.
  /// @TODO Remove the need for this nonsense.
  Cfg::m_robots.clear();
  Cfg::SetSize(m_robots.size());
  for(size_t i = 0; i < m_robots.size(); ++i)
    Cfg::InitRobots(m_robots[i]->GetMultiBody(), i);

  // Compute the environment resolution.
  GetEnvironment()->ComputeResolution(GetRobots());
}


bool
MPProblem::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "Environment") {
    // Ignore this node if we already have an environment.
    if(!m_environment)
      m_environment = new Environment(_node);
    return true;
  }
  else if(_node.Name() == "Robot") {
    /// @TODO We currently assume that the environment is parsed first. Need to
    ///       make sure this always happens regardless of the XML file ordering.
    /// @TODO Move the DOF parsing into MultiBody's read so that we don't have
    ///       to pass the boundary with the robot constructor.
    auto robot = new Robot(this, _node, GetEnvironment()->GetBoundary().get());
    m_robots.push_back(robot);
    return true;
  }
  else if(_node.Name() == "Query") {
    // Ignore this node if we already have a query file.
    if(m_queryFilename.empty())
      m_queryFilename = _node.Read("filename", true, "", "Query file name");
    return true;
  }
  else
    return false;
}

/*----------------------------- Environment Accessors ------------------------*/

Environment*
MPProblem::
GetEnvironment() {
  return m_environment;
}


void
MPProblem::
SetEnvironment(Environment* _e) {
  m_environment = _e;
}

/*------------------------------ Robot Accessors -----------------------------*/

size_t
MPProblem::
NumRobots() const {
  return m_robots.size();
}


Robot*
MPProblem::
GetNewRobot(size_t _index) const {
  if(_index >= m_robots.size())
    throw RunTimeException(WHERE, "Requested Robot " + std::to_string(_index) +
        ", but only " + std::to_string(m_robots.size()) +
        " robots are available.");
  return m_robots[_index];
}


Robot*
MPProblem::
GetNewRobot(const std::string& _label) {
  for(auto robot : m_robots)
    if(robot->GetLabel() == _label)
      return robot;

  throw RunTimeException(WHERE, "Requested Robot with label '" + _label + "', "
      "but no such robot was found.");
  return nullptr;
}


ActiveMultiBody*
MPProblem::
GetRobot(size_t _index) const {
  return GetNewRobot(_index)->GetMultiBody();
}


vector<ActiveMultiBody*>
MPProblem::
GetRobots() const {
  vector<ActiveMultiBody*> robots;
  robots.reserve(m_robots.size());
  for(const auto& ptr : m_robots)
    robots.push_back(ptr->GetMultiBody());
  return robots;
}

/*----------------------------- Task Accessors -------------------------------*/

const std::vector<MPTask*>&
MPProblem::
GetTasks() const noexcept {
  return m_tasks;
}

/*-------------------------------- Debugging ---------------------------------*/

void
MPProblem::
Print(ostream& _os) const {
  _os << "MPProblem" << endl;
  m_environment->Print(_os);
}

/*-------------------------- File Path Accessors -----------------------------*/

const std::string&
MPProblem::
GetBaseFilename() const {
  return m_baseFilename;
}


void
MPProblem::
SetBaseFilename(const std::string& _s) {
  m_baseFilename = _s;
}


string MPProblem::m_filePath = "";


string
MPProblem::
GetPath(const string& _filename) {
  if(_filename[0] != '/')
    return m_filePath + _filename;
  else
    return _filename;
}


void
MPProblem::
SetPath(const string& _filename) {
  m_filePath = _filename;
}

/*----------------------------------------------------------------------------*/
