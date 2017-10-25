#include "PathFollowingChildAgent.h"

#include <limits>
#include <unordered_map>
#include <algorithm>
#include "ConfigurationSpace/Cfg.h"
#include "Behaviors/Agents/AgentGroup.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"
#include "utils/tcp_socket.h"

vector<Cfg> PathFollowingChildAgent::m_AllRoadmapPoints;
vector<std::string> PathFollowingChildAgent::m_chargingLocations;

int iidx = 0;
vector<std::string> Square;
/*------------------------------ Construction --------------------------------*/

PathFollowingChildAgent::
PathFollowingChildAgent(Robot* const _r) : Agent(_r) { }

PathFollowingChildAgent::
~PathFollowingChildAgent() {
  // Ensure agent is properly torn down.
  PathFollowingChildAgent::Uninitialize();
}

void
PathFollowingChildAgent::
SetMPRoadmap(RoadmapType* _rMap) {
  m_solution->SetRoadmap(_rMap);
}

void PathFollowingChildAgent::
SetTask(MPTask* _task) {
  m_task = _task;
}

void
PathFollowingChildAgent::
InitializeMpSolution(MPSolution* _s) {
  // Create a new solution object to hold a plan for this agent.
  m_solution = new MPSolution(_s->GetRobot());
  m_solution->SetRoadmap(_s->GetRoadmap());
}

/*------------------------------ Helpers -------------------------------------*/

/// Connect to the netbook and get coordinate information
/*vector<double>
PathFollowingChildAgent::
GetCoordinatesFromMarker() {
  cout << "Trying to get coordinates " << endl;
  utils::tcp_socket client(m_robot->GetIPAddress(), "4002");
  cout << "Connection successful " << endl;
  size_t count;
  cout << "Sending packet (1,2,3)..." << endl;
  char c;
  client >> c;
  count = c;
  cout << "Size of c " << count << endl;
  double x = 0.0;
  double y = 0.0;
  double angle = 0.0;
  for(size_t i=0;i<count;i++) {
    packet p1;
    client >> p1;
    x += p1.x/10000.0;
    y += p1.y/10000.0;
    angle += p1.angle/10000.0;
  }
  // Now let's average the values
  double totalMarkers = (double)count;
  x = x/totalMarkers;
  y = y/totalMarkers;
  angle = angle/totalMarkers;
  cout << "Odometry of Robot: " << x << ", " << y << ", " << (angle)*(180/M_PI) << endl;
  client.disconnect();
  vector<double> coordinates = {};
  return coordinates;
}*/


/// very basic collision detection
//TODO: improve this to handle collision better. Currently it only handles the
//case where the robot is going to collide with other robot's side. Need to take
//care of deadlock scenarios. Also figure out what path to take to avoid
//collision.
bool
PathFollowingChildAgent::
InCollision() {
  auto dm = m_library->GetDistanceMetric("euclidean");

  auto problem = m_robot->GetMPProblem();

  bool inCollision = false;
  for(auto& robot : problem->GetRobots()) {
    if(robot != m_robot) {
      auto robotPosition = robot->GetDynamicsModel()->GetSimulatedState();
      auto myPosition = m_robot->GetDynamicsModel()->GetSimulatedState();
      double distance = dm->Distance(robotPosition, myPosition);
      // If the other robot has higher priority, halt this robot.
      if(distance < 3){
        inCollision = true;
        if(m_robot->GetAgent()->m_priority < robot->GetAgent()->m_priority){
          m_shouldHalt = true;
        }
      }
    }
  }
  return inCollision;
}


Cfg
PathFollowingChildAgent::
GetRandomRoadmapPoint() {
  if(m_AllRoadmapPoints.empty())
    m_done = true;

  auto point = m_AllRoadmapPoints.back();
  m_AllRoadmapPoints.pop_back();
  return point;
}

//Go to a random point in the roadmap
MPTask*
PathFollowingChildAgent::
GetNewTask() {
  //use the parent robot
  auto task = new MPTask(m_parentRobot);
  auto currentPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto newGoal = GetRandomRoadmapPoint();
  //if(iidx > 3)
    //iidx = 0;
  //auto newGoal = Square[iidx];
  //iidx++;
  //use the parent robot
  auto start = new CSpaceConstraint(m_parentRobot, currentPos);
  //use the parent robot
  auto goal = new CSpaceConstraint(m_parentRobot, newGoal);
  task->AddStartConstraint(start);
  task->AddGoalConstraint(goal);
  return task;
}

///TODO: Get helper that is closest to the worker
//currently we just get a random helper and assign it to the worker.
bool
PathFollowingChildAgent::
CallForHelp() {
  //std::unordered_map<Robot*, Cfg>::iterator item;

  //item = m_parentAgent->GetHelpers().begin();
  //TODO: change this to get closest helper
  if(m_parentAgent->GetHelpers().empty())
    return false;
  auto helper = m_parentAgent->GetHelpers().front();
  cout << "HELPER: " << helper->GetLabel() << " M_ROBOT: " << m_robot->GetLabel() << endl;
  //if(item != m_parentAgent->GetHelpers().end()) {
  //int random_index = rand() % m_parentAgent->GetHelpers().size();
  //std::advance(item, random_index);
  if(m_waitForHelp) {
    //Set new task for helper to get to worker position
    auto workerPos = m_robot->GetDynamicsModel()->GetSimulatedState();
    cout << "worker Robot position in call for help " << workerPos << endl;
    auto helperPos = helper->GetDynamicsModel()->GetSimulatedState();
    cout << "helper Robot position in call for help " << helperPos << endl;
    //auto helperTask = CreateNewTask(helperPos, workerPos, "label");
    auto helperTask = new MPTask(m_parentRobot);
    //auto helperPos = m_myHelper->GetDynamicsModel()->GetSimulatedState();
    auto start = new CSpaceConstraint(m_parentRobot, helperPos);
    auto goal = new CSpaceConstraint(m_parentRobot, workerPos);
    //auto goal = m_robot()->GetAgent()->GetTask()->GetGoalConstraint();

    helperTask->AddStartConstraint(start);
    helperTask->AddGoalConstraint(goal);
    helperTask->SetLabel("GoingToHelp");

    cout << "Setting this task for helper " << start << " ----- " << goal << endl;
    m_pathIndex = 0;
    m_path.clear();
    helper->GetAgent()->SetCurrentTask(helperTask);
    m_myHelper = helper;
    cout << "SIZE BEFORE: " << m_parentAgent->GetHelpers().size() << endl;
    m_parentAgent->GetHelpers().erase(m_parentAgent->GetHelpers().begin()+0);
    cout << "SIZE AFTER: " << m_parentAgent->GetHelpers().size() << endl;
    //let helper get close enough to worker
    //assign worker's old goal to new worker
  }
  else{
    helper->SetLabel("worker");
    //TODO: Maybe need to swap goals / priorities here.
  }
  return true;
  //}
  //return false;
}

bool
PathFollowingChildAgent::
IsAtChargingStation() {
  auto currentPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto dm = m_library->GetDistanceMetric("euclidean");
  const double threshold = .05;
  for(const auto& x : m_chargingLocations) {
    Cfg point(m_robot);
    std::istringstream pointStream(x);
    point.Read(pointStream);
    double distance = dm->Distance(currentPos, point);
    if(distance <= threshold)
      return true;
  }
  return false;
}

void
PathFollowingChildAgent::
InitializePointsVector() {
  for(typename GraphType::VI i = m_solution->GetRoadmap()->GetGraph()->begin();
      i!=m_solution->GetRoadmap()->GetGraph()->end(); i++) {
    Cfg cfg =  m_solution->GetRoadmap()->GetGraph()->GetVertex(i);
    if (!(std::find(m_AllRoadmapPoints.begin(), m_AllRoadmapPoints.end(),cfg)!=m_AllRoadmapPoints.end()))
      m_AllRoadmapPoints.push_back(cfg);
  }
  std::random_shuffle ( m_AllRoadmapPoints.begin(), m_AllRoadmapPoints.end() );
}

void
PathFollowingChildAgent::
FindNearestChargingLocation() {
  if(m_task->GetLabel() == "GettingToChargingLocation")
    return;
  cout << "Going to charging location " << endl;
  m_task = new MPTask(m_parentRobot);
  auto currentPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto randN = rand() % m_chargingLocations.size();
  auto chargingLocation = m_chargingLocations[randN];
  //cout << m_robot->GetLabel() << " found this charging location: " << chargingLocation << endl;
  auto start = new CSpaceConstraint(m_parentRobot, currentPos);
  auto goal = new CSpaceConstraint(m_parentRobot, chargingLocation);
  m_task->AddStartConstraint(start);
  m_task->AddGoalConstraint(goal);
  m_task->SetLabel("GettingToChargingLocation");
  auto problem = m_robot->GetMPProblem();

  m_pathIndex = 0;
  m_path.clear();
  // Use the planning library to find a path.
  m_library->Solve(problem, m_task, m_solution);

  m_path = m_solution->GetPath()->Cfgs();
}

/*------------------------------ Functions for ICreate ------------------------*/

vector<double>
PathFollowingChildAgent::
GetRotationAndTranslationAmt(const Cfg& _current, const Cfg& _goal) {

  vector<double> curPoints = _current.GetData();
  vector<double> goalPoints = _goal.GetData();

  double xDist = (goalPoints[0] - curPoints[0]);
  double yDist = (goalPoints[1] - curPoints[1]);
  double translateAmt = sqrt(pow(xDist,2) + pow(yDist,2));
  cout << "m_odometry[2] " << m_odometry[2] << endl;
  double rotAmt = atan2(yDist,xDist) - m_odometry[2];

  //Normalize rotAmt
  if (rotAmt > PI)
    rotAmt -= (2.0 * PI);
  else if (rotAmt < -PI)
    rotAmt += (2.0 * PI);

  cout << "cur point " << curPoints[0] <<", " << curPoints[1] << " goal: " << goalPoints[0] << ", " << goalPoints[1]\
    << " \nx dist " << xDist << "\ny dist " << yDist << "\nRoation amount: " << rotAmt << "\nTranslation amount: " << translateAmt << endl;
  vector<double> RotationAndTranslation;
  RotationAndTranslation.push_back(rotAmt);
  RotationAndTranslation.push_back(translateAmt);

  return RotationAndTranslation;
}

void
PathFollowingChildAgent::
UpdateOdometry(const double& _x, const double& _y, const double& _angle) {
  m_odometry[0] = _x;
  m_odometry[1] = _y;
  m_odometry[2] += _angle;

  //If angle is greater than 2pi then subtract 2pi from the angle.
  if(m_odometry[2] >= 2*M_PI)
    m_odometry[2] = m_odometry[2] - 2*M_PI;
  //If angle is less than 2pi then add 2pi to the angle.
  else if(m_odometry[2] <= -2*M_PI)
    m_odometry[2] = m_odometry[2] + 2*M_PI;
}

/*------------------------------ Agent Interface -----------------------------*/

void
PathFollowingChildAgent::
Initialize() {
  if(m_initialized)
    return;
  m_initialized = true;

  m_battery = new Battery();

  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();

  m_task = problem->GetTasks(m_robot).front();
  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  // Use the planning library to find a path.
  m_library->Solve(problem, m_task, m_solution);

  // Extract the path from the solution.
  m_path = m_solution->GetPath()->Cfgs();

  InitializePointsVector();

  std::string charginL = "3 0 0 0 0 0";
  m_chargingLocations.push_back(charginL);

  Square.push_back("4 0 0 0 0 0");
  Square.push_back("4 1 0 0 0 0");
  Square.push_back("5 1 0 0 0 0");
  Square.push_back("5 0 0 0 0 0");

}

void
PathFollowingChildAgent::
ExecuteTask(double _dt) {
  // Do nothing if there are no unvisited points left.
  if(m_pathIndex >= m_path.size()) {
    return;
  }

  if(m_debug)
    std::cout << "Approaching waypoint " << m_pathIndex << " / "
              << m_path.size() - 1 << ".\n";
  // Get the current configuration.
  const Cfg current = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto data = current.GetData();

  ////=============================================================================////

  // Ensure we have a create controller.
  /*if(m_robot->GetLabel() == "worker" && m_robot->GetIPAddress() != "") {
    static RobotController createController(m_robot->GetIPAddress());
    createController.StartCommandQueue(); // Does nothing after first call.

    vector<double> currentControls;
    if(m_goalTaken.empty() && current != m_path[m_pathIndex]) {
      cout << "current point " << current << " goal " << m_path[m_pathIndex] << endl;
      currentControls = GetRotationAndTranslationAmt(current, m_path[m_pathIndex]);
      UpdateOdometry(0.0,0.0,currentControls[0]);
      createController.EnqueueCommand(currentControls[1], currentControls[0], 0);
      m_goalTaken.push_back(m_path[m_pathIndex]);
    }
  }*/
  ////=============================================================================////

  // We consider the robot to have reached the next subgoal if it is within a
  // threshold distance. Advance the path index until the next subgoal is
  // at least one threshold away.
  auto dm = m_library->GetDistanceMetric("euclidean");
  const double threshold = .05;

  double distance = dm->Distance(current, m_path[m_pathIndex]);
  //cout << m_robot->GetLabel() << " current point here " << current << " goal here " << m_path[m_pathIndex] << endl;

  if(m_debug)
    std::cout << "\tDistance from current configuration: "
              << distance << "/" << threshold
              << std::endl;

  while(distance < threshold and m_pathIndex < m_path.size()) {
    if(m_debug)
      std::cout << "\tReached waypoint " << m_pathIndex << " at "
                << distance << "/" << threshold << std::endl
                << "Waypoint = " << m_path[m_pathIndex] << std::endl;

    cout << m_robot->GetLabel() << " This is the current goal " << m_path[m_pathIndex] << endl;
    // Move to next cfg in path since the distance is within the threshold.
    ++m_pathIndex;
    m_goalTaken.clear();

    // Break if we try to go beyond the path's end. Necessary, as calculating
    // the distance on an undefined cfg will crash some systems.
    if(m_pathIndex >= m_path.size())
      break;

    distance = dm->Distance(current, m_path[m_pathIndex]);
  }

  // If we hit the end, return.
  if(m_pathIndex >= m_path.size()) {
    //if(m_debug)
      std::cout << m_robot->GetLabel() << " : Reached the end of the path." << std::endl;

    // Warning: Halt() doesn't respect the dynamics of the simulation and is
    // only to be used for visual verification of the path in the simulator.
    m_task->SetCompleted(true);
    this->Halt();
    return;
  }

  // Otherwise, execute the control that is nearest to the desired force.
  auto bestControl = m_robot->GetController()->operator()(current,
      m_path[m_pathIndex], _dt);
  bestControl.Execute();

}


void
PathFollowingChildAgent::
Step(const double _dt) {
  if(m_done) {
    cout << "All tasks done. Let's go home. " << endl;
    return;
  }
  Initialize();
  //cout << "I am a " << m_robot->GetLabel() << " my current task is " << m_task->GetLabel() << endl;
  //cout << "value of started " << m_task->Started() << endl;
  if(!m_task->Started()) {
    auto problem = m_robot->GetMPProblem();
    m_library->Solve(problem, m_task, m_solution);

    m_path = m_solution->GetPath()->Cfgs();
    m_task->SetStarted(true);
  }
  if(!InCollision()){
    m_shouldHalt = false;
    ExecuteTask(_dt);
  }
  else{
    if(m_shouldHalt)
      this->Halt();
    else{
      AvoidCollision();
    }
  }

  if(m_robot->GetLabel() == "worker")
    WorkerStep(_dt);
  else
    HelperStep(_dt);
}

void
PathFollowingChildAgent::
WorkerStep(const double _dt) {
  //cout << "Worker Doing Stuff" << endl;
  m_battery->UpdateValue(0.01);
  if(m_task->IsCompleted()) {
    m_pathIndex = 0;
    m_path.clear();
    m_task = GetNewTask();

    // Use the planning library to find a path.
  }
  if((m_battery->GetCurLevel() < 0.2*m_battery->GetMaxLevel())) {
    // if you don't have a helper yet, call for help
    if(!m_myHelper) {
      cout << "Calling for Help " << endl;
      m_pathIndex = 0;
      m_path.clear();
      this->Halt();
      CallForHelp();
    }
    // else check if the helper is close enough to you to take over
    else if(m_myHelper) {
      auto helperPos = m_myHelper->GetDynamicsModel()->GetSimulatedState();
      auto myPos = m_robot->GetDynamicsModel()->GetSimulatedState();
      auto dm = m_library->GetDistanceMetric("euclidean");

      //cout << m_robot->GetLabel() << " path index " << m_pathIndex << " size " << m_path.size() << endl;
      cout << m_robot->GetLabel() << " my position: " << myPos << endl;
      cout << m_myHelper->GetLabel() << " my position: " << helperPos << endl;

      double distance = dm->Distance(myPos, helperPos);
      cout << "distance: " << distance << endl;
      if(distance < 3.0) {
        cout << "Changing with helper " << endl;
        // Get this robot's task and assign it to worker. However, we need to
        // change the start constraints, so get the goal constraint from the
        // worker's task
        // TODO: Figure out what m_parentRobot is
        auto newTask = new MPTask(m_parentRobot);
        auto helperPos = m_myHelper->GetDynamicsModel()->GetSimulatedState();
        auto start = new CSpaceConstraint(m_parentRobot, helperPos);
        auto goal = m_robot->GetAgent()->GetCurrentTask()->GetGoalConstraints().front();

        m_myHelper->SetLabel("worker");
        newTask->AddStartConstraint(start);
        newTask->AddGoalConstraint(goal);
        newTask->SetLabel("WorkerTask");

        m_myHelper->GetAgent()->SetCurrentTask(newTask);

        // Swap the helper's and the worker's priority.
        int temp = m_myHelper->GetAgent()->m_priority;
        m_myHelper->GetAgent()->m_priority = m_robot->GetAgent()->m_priority;
        m_robot->GetAgent()->m_priority = temp;

        cout << m_robot << " Priority: " << m_robot->GetAgent()->m_priority << endl;
        cout << m_myHelper << " Priority: " << m_myHelper->GetAgent()->m_priority << endl;
        m_pathIndex = 0;
        m_path.clear();
        m_myHelper = nullptr;
        // Use the planning library to find a path.
        m_robot->SetLabel("helper");
      }
      else
        return;
    }
    else
      return;
  }
}


void
PathFollowingChildAgent::
HelperStep(const double _dt) {
  //cout << "Helper Doing Stuff" << endl;
  if(m_robot->GetLabel() == "helper" && !IsAtChargingStation() && m_task->GetLabel() != "GoingToHelp") {
    FindNearestChargingLocation();
  }
  else if(m_robot->GetLabel() == "helper" && IsAtChargingStation()) {
    m_path.clear();
    m_pathIndex = 0;
    //if battery is low, charge
    m_battery->Charge(5);
    //if battery is full (more than 90%) , available to help
    if(m_battery->GetCurLevel() >= 0.9*m_battery->GetMaxLevel()){
      auto curPos = m_robot->GetDynamicsModel()->GetSimulatedState();
      //TODO: change this so that you don't keep pushing back the same robot
      //maybe use an unordered set?
      auto helpers = m_parentAgent->GetHelpers();
      // If m_robot is not on the helpers list, push it back
      if(find(helpers.begin(), helpers.end(), m_robot) == helpers.end() && m_task->GetLabel() != "GoingToHelp"){
        m_parentAgent->GetHelpers().push_back(m_robot);
      }
      //cout << "Done charging, available to help " << endl;
    }
  }
}


void
PathFollowingChildAgent::
AvoidCollision() {
  auto dm = m_library->GetDistanceMetric("euclidean");

  auto problem = m_robot->GetMPProblem();
  for(auto& robot : problem->GetRobots()) {
    if(robot != m_robot && (m_pathIndex < m_path.size()) ) {
      auto current = m_robot->GetDynamicsModel()->GetSimulatedState();
      auto robotPos = robot->GetDynamicsModel()->GetSimulatedState();
      double distanceToSubgoal = dm->Distance(current, m_path[m_pathIndex]);
      double distance1 = dm->Distance(current, robotPos);
      double distance2 = dm->Distance(m_path[m_pathIndex], robotPos);
      //TODO: Pick more accurate threshold than 3
      //TODO: Change the task of the robot with higher priority
      if(((distance1 + distance2) - distanceToSubgoal) < 3) {
        //cout << "TODO: Avoid Collision" << endl;
        //m_path.clear();
        //m_pathIndex = 0;
        //SetTask(GetNewTask());
      }
    }
  }
}


void
PathFollowingChildAgent::
Uninitialize() {
  if(!m_initialized)
    return;
  m_initialized = false;

  m_path.clear();
  m_pathIndex = 0;
  delete m_library;
  delete m_task;
  delete m_battery;
}

/*----------------------------------------------------------------------------*/
