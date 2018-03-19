#include "MPProblem.h"

#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/DynamicObstacle.h"
#include "Utilities/MPUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

using namespace std;


/*---------------------------- Construction ----------------------------------*/

MPProblem::
MPProblem() = default;


MPProblem::
MPProblem(const string& _filename) {
  ReadXMLFile(_filename);
}


MPProblem::
MPProblem(const MPProblem& _other) {
  *this = _other;
}


MPProblem::
MPProblem(MPProblem&& _other) = default;


MPProblem::
~MPProblem() = default;

/*-------------------------------- Assignment --------------------------------*/

MPProblem&
MPProblem::
operator=(const MPProblem& _other) {
  // Guard against self-assignment.
  if(this == &_other)
    return *this;

  // Copy environment.
  m_environment = std::unique_ptr<Environment>(
      new Environment(*_other.m_environment)
  );

  // Copy robots.
  m_robots.clear();
  for(const auto& robot : _other.m_robots)
    m_robots.emplace_back(new Robot(this, *robot));

  // Copy tasks.
  m_taskMap.clear();
  for(const auto& robotTasks : _other.m_taskMap) {
    Robot* const robot = robotTasks.first;
    const auto& tasks = robotTasks.second;

    for(const auto& task : tasks) {
      m_taskMap[robot].emplace_back(new MPTask(*task));
      m_taskMap[robot].back()->SetRobot(robot);
    }
  }

  m_xmlFilename = _other.m_xmlFilename;
  m_baseFilename = _other.m_baseFilename;
  m_filePath = _other.m_filePath;

  return *this;
}


MPProblem&
MPProblem::
operator=(MPProblem&& _other) = default;

/*---------------------------- XML Helpers -----------------------------------*/

const std::string&
MPProblem::
GetXMLFilename() const {
  return m_xmlFilename;
}


void
MPProblem::
ReadXMLFile(const string& _filename) {
  const bool envIsSet = m_environment.get();

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

  // If no tasks were specified, assume we want an unconstrained plan for the
  // first robot.
  if(m_taskMap.empty()) {
    if(m_robots.size() > 1)
      throw ParseException(input.Where(), "No task was specified in the problem "
          "node, but multiple robots are specified. Taskless execution only "
          "supports single robot problems.");

    auto robot = m_robots.front().get();
    std::cout << "No task specified, assuming we want an unconstrained plan for "
              << "the first robot, labeled \'" << robot->GetLabel() << "\'."
              << std::endl;

    m_taskMap[robot].emplace_back(new MPTask(robot));
  }

  // Compute the environment resolution.
  GetEnvironment()->ComputeResolution(GetRobots());
}


void
MPProblem::
AddTask(std::unique_ptr<MPTask>&& _task) {
  auto robot = _task->GetRobot();
  m_taskMap[robot].push_back(std::move(_task));
}


void
MPProblem::
ReassignTask(MPTask* const _task, Robot* const _newOwner) {
  // Find an iterator to the existing entry in the task map.
  auto oldOwner = _task->GetRobot();
  auto& oldTasks = m_taskMap.at(oldOwner);
  auto iter = oldTasks.begin();
  for(; iter != oldTasks.end(); ++iter)
    if(iter->get() == _task)
      break;

  // If we hit the end, the task was not found.
  if(iter == oldTasks.end())
    throw RunTimeException(WHERE, "Requested task was not found.");

  // Set the task's robot.
  (*iter)->SetRobot(_newOwner);

  // Move the task to the new owner's map.
  auto& newTasks = m_taskMap.at(_newOwner);
  newTasks.emplace_back(std::move(*iter));
  oldTasks.erase(iter);
}

/*----------------------------- Environment Accessors ------------------------*/

Environment*
MPProblem::
GetEnvironment() {
  return m_environment.get();
}


void
MPProblem::
SetEnvironment(std::unique_ptr<Environment>&& _e) {
  m_environment = std::move(_e);

  // Reset the robot DOF limits based on the new environment.
  MakePointRobot();
  for(auto& robot : m_robots)
    robot->InitializePlanningSpaces();
}

/*------------------------------ Robot Accessors -----------------------------*/

size_t
MPProblem::
NumRobots() const noexcept {
  return m_robots.size();
}


Robot*
MPProblem::
GetRobot(size_t _index) const {
  if(_index >= m_robots.size())
    throw RunTimeException(WHERE, "Requested Robot " + std::to_string(_index) +
        ", but only " + std::to_string(m_robots.size()) +
        " robots are available.");
  return m_robots[_index].get();
}


Robot*
MPProblem::
GetRobot(const std::string& _label) const {
  if(_label == "point")
    return m_pointRobot.get();

  for(auto& robot : m_robots)
    if(robot->GetLabel() == _label)
      return robot.get();

  throw RunTimeException(WHERE, "Requested Robot with label '" + _label + "', "
      "but no such robot was found.");
  return nullptr;
}


const std::vector<std::unique_ptr<Robot>>&
MPProblem::
GetRobots() const noexcept {
  return m_robots;
}

/*----------------------------- Task Accessors -------------------------------*/

std::vector<MPTask*>
MPProblem::
GetTasks(Robot* const _robot) const noexcept {
  const auto& tasks = m_taskMap.at(_robot);

  std::vector<MPTask*> output;

  for(const auto& task : tasks)
    if(!task->IsCompleted())
      output.push_back(task.get());

  return output;
}


const std::vector<std::unique_ptr<DynamicObstacle>>&
MPProblem::
GetDynamicObstacles() const noexcept {
  return m_dynamicObstacles;
}

/*-------------------------------- Debugging ---------------------------------*/

void
MPProblem::
Print(ostream& _os) const {
  _os << "MPProblem"
      << std::endl;
  m_environment->Print(_os);
  /// @TODO Print robot and task information.
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


string
MPProblem::
GetPath(const string& _filename) {
  if(!_filename.empty() and _filename[0] != '/')
    return m_filePath + _filename;
  else
    return _filename;
}


void
MPProblem::
SetPath(const string& _filename) {
  m_filePath = _filename;
}

/*---------------------------- Construction Helpers --------------------------*/

bool
MPProblem::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "Environment") {
    // Ignore this node if we already have an environment.
    if(!m_environment)
      m_environment = std::unique_ptr<Environment>(new Environment(_node));
    MakePointRobot();
    return true;
  }
  else if(_node.Name() == "Robot") {
    /// @TODO We currently assume that the environment is parsed first. Need to
    ///       make sure this always happens regardless of the XML file ordering.
    m_robots.emplace_back(new Robot(this, _node));
    return true;
  }
  else if(_node.Name() == "DynamicObstacle") {
    // If this is a dynamic obstacle, get the path file name and make sure it exists.

    std::unique_ptr<Robot> robot(new Robot(this, _node));

    const std::string filePath = GetPathName(_node.Filename());
    const std::string obstacleFile = _node.Read("pathfile", true, "",
        "DynamicObstacle path file name");
    const std::string obstacleFilename = filePath + obstacleFile;

    vector<Cfg> path = LoadPath(obstacleFilename, robot.get());
    m_dynamicObstacles.emplace_back(new DynamicObstacle(std::move(robot), path));
    return true;
  }
  else if(_node.Name() == "Task") {
    const std::string label = _node.Read("robot", true, "", "Label for the robot "
        " assigned to this task.");
    m_taskMap[this->GetRobot(label)].emplace_back(new MPTask(this, _node));
    return true;
  }
  else
    return false;
}


void
MPProblem::
MakePointRobot() {
  // Make robot's multibody.
  std::unique_ptr<MultiBody> point(new MultiBody(MultiBody::Type::Active));

  const bool is2d = GetEnvironment()->GetBoundary()->GetDimension() == 2;

  Body body(point.get());
  body.SetBodyType(is2d ? Body::Type::Planar : Body::Type::Volumetric);
  body.SetMovementType(Body::MovementType::Translational);

  // Create body geometry. Use a single, open triangle.
  GMSPolyhedron poly;
  poly.GetVertexList() = vector<Vector3d>{{1e-8, 0, 0},
      {0, 0, 1e-8}, {-1e-8, 0, 0}};
  poly.GetPolygonList() = vector<GMSPolygon>{GMSPolygon(0, 1, 2,
      poly.GetVertexList())};
  body.SetPolyhedron(std::move(poly));

  // Add body geometry to multibody.
  const size_t index = point->AddBody(std::move(body));
  point->SetBaseBody(index);

  // Make sure we didn't accidentally call another robot 'point'.
  if(GetRobot("point"))
    throw RunTimeException(WHERE, "A robot in the problem is named 'point'. "
        "This name is reserved for the internal point robot.");

  // Create the robot object.
  m_pointRobot = std::unique_ptr<Robot>(
      new Robot(this, std::move(point), "point")
  );
  m_pointRobot->SetVirtual(true);
}

/*----------------------------------------------------------------------------*/
