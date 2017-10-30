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

/*------------------------------ Construction --------------------------------*/

PathFollowingChildAgent::
PathFollowingChildAgent(Robot* const _r) : Agent(_r) {
}

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

/// very basic collision detection
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
      double distance = EuclideanDistance(robotPosition, myPosition);
      // If the other robot has higher priority, halt this robot.
      if(distance < 3){
        inCollision = true;
        if(m_robot->GetAgent()->m_priority < robot->GetAgent()->m_priority){
          m_shouldHalt = true;
          return inCollision;
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
  //use the parent robot
  auto start = new CSpaceConstraint(m_parentRobot, currentPos);
  //use the parent robot
  auto goal = new CSpaceConstraint(m_parentRobot, newGoal);
  task->AddStartConstraint(start);
  task->AddGoalConstraint(goal);
  return task;
}

bool
PathFollowingChildAgent::
CallForHelp() {

  if(m_parentAgent->GetHelpers().empty())
    return false;
  //TODO: Change this to get closest helper
  auto helper = m_parentAgent->GetHelpers().front();
  if(m_waitForHelp) {
    //Set new task for helper to get to worker position
    auto workerPos = m_robot->GetDynamicsModel()->GetSimulatedState();
    cout << "worker Robot position in call for help " << workerPos << endl;
    auto helperPos = helper->GetDynamicsModel()->GetSimulatedState();
    cout << "helper Robot position in call for help " << helperPos << endl;

    auto helperTask = new MPTask(m_parentRobot);
    auto start = new CSpaceConstraint(m_parentRobot, helperPos);
    auto goal = new CSpaceConstraint(m_parentRobot, workerPos);

    //let helper get close enough to worker
    //assign worker's old goal to new worker
    helperTask->AddStartConstraint(start);
    helperTask->AddGoalConstraint(goal);
    helperTask->SetLabel("GoingToHelp");

    cout << "Setting this task for helper " << start << " ----- " << goal << endl;
    m_pathIndex = 0;
    m_path.clear();
    helper->GetAgent()->SetCurrentTask(helperTask);
    m_myHelper = helper;
    //TODO Find a better way to assign priorities
    m_myHelper->GetAgent()->m_priority = m_robot->GetAgent()->m_priority-100;
    m_parentAgent->GetHelpers().erase(m_parentAgent->GetHelpers().begin());

    // Set the helper's charging station to open
    ClearChargingStation();
  }
  else{
    //helper->SetLabel("worker");
  }
  return true;
}

bool
PathFollowingChildAgent::
IsAtChargingStation() {
  auto currentPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  const double threshold = .5;
  for(const auto& chargingLocation : m_parentAgent->GetChargingLocations()) {
    double distance = EuclideanDistance(currentPos, chargingLocation.first);
    if(distance <= threshold){
      return true;
    }
  }
  return false;
}

void
PathFollowingChildAgent::
ClearChargingStation(){
  // Clear the charging station that m_myHelper was on
  auto currentPos = m_myHelper->GetDynamicsModel()->GetSimulatedState();
  const double threshold = .5;
  for(auto& chargingLocation : m_parentAgent->GetChargingLocations()) {
    double distance = EuclideanDistance(currentPos, chargingLocation.first);
    if(distance <= threshold){
      chargingLocation.second = false;
      cout << "No More Robot On Charging Location: " << chargingLocation.first << endl;
      return;
    }
  }
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
  auto task = new MPTask(m_parentRobot);
  auto problem = m_robot->GetMPProblem();

  auto currentPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto start = new CSpaceConstraint(m_parentRobot, currentPos);
  vector<Cfg> tempPath;
  Cfg chargingLocation;
  double pathLength = 0;
  // Find the closest charging location to m_robot
  // TODO: Is solving frequently breaking things?
  for(auto tempLocation : m_parentAgent->GetChargingLocations()){
    auto tempGoal = new CSpaceConstraint(m_parentRobot, tempLocation.first);
    auto tempTask = new MPTask(m_parentRobot);
    // Create a temporary task and solve it to get path length
    tempTask->AddStartConstraint(start);
    tempTask->AddGoalConstraint(tempGoal);
    m_library->Solve(problem, tempTask, m_solution);
    tempPath = m_solution->GetPath()->Cfgs();
    // If the temp task has the shortest path length, store the task
    double tempLength = GetPathLength(tempPath);
    cout << "Charging Location: " << tempLocation.first << " Length: " << tempLength << endl;
    if((tempLength < pathLength || pathLength == 0) && tempLocation.second == false){
      pathLength = tempLength;
      task = tempTask;
      chargingLocation = tempLocation.first;
    }
  }

  task->SetLabel("GettingToChargingLocation");

  this->SetTask(task);

  m_pathIndex = 0;
  m_path.clear();
  // Use the planning library to find a path.
  m_library->Solve(problem, m_task, m_solution);

  m_path = m_solution->GetPath()->Cfgs();

  cout << "Going To Charging Location: " << chargingLocation << endl;
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

  // We consider the robot to have reached the next subgoal if it is within a
  // threshold distance. Advance the path index until the next subgoal is
  // at least one threshold away.
  auto dm = m_library->GetDistanceMetric("euclidean");
  const double threshold = .05;

  double distance = EuclideanDistance(current, m_path[m_pathIndex]);

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

    distance = EuclideanDistance(current, m_path[m_pathIndex]);
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

  cout << m_robot << " Priority: " << m_robot->GetAgent()->m_priority << endl;

  if(!m_task->Started()) {
    auto problem = m_robot->GetMPProblem();
    m_library->Solve(problem, m_task, m_solution);

    m_path = m_solution->GetPath()->Cfgs();
    m_task->SetStarted(true);
  }
  if(!InCollision()){
    m_shouldHalt = false;
    ExecuteTask(_dt);
    m_avoidCollisionHalt = 0;
  }
  else{
    if(m_shouldHalt)
      this->Halt();
    else{
      AvoidCollision();
      m_avoidCollisionHalt++;
      ExecuteTask(_dt);
    }
  }

  if(m_robot->GetLabel().compare(0, 6, "worker") == 0)
    WorkerStep(_dt);
  else
    HelperStep(_dt);
}

void
PathFollowingChildAgent::
WorkerStep(const double _dt) {
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

      cout << m_robot->GetLabel() << " my position: " << myPos << endl;
      cout << m_myHelper->GetLabel() << " my position: " << helperPos << endl;

      double distance = EuclideanDistance(myPos, helperPos);
      cout << "distance: " << distance << endl;
      if(distance < 3.0) {
        cout << "Changing with helper " << endl;
        // Get this robot's task and assign it to worker. However, we need to
        // change the start constraints, so get the goal constraint from the
        // worker's task
        auto newTask = new MPTask(m_parentRobot);
        auto helperPos = m_myHelper->GetDynamicsModel()->GetSimulatedState();
        auto start = new CSpaceConstraint(m_parentRobot, helperPos);
        auto goal = m_robot->GetAgent()->GetCurrentTask()->GetGoalConstraints().front();

        // Swap the helper's and the worker's labels.
        string tempLabel = m_myHelper->GetLabel();
        m_myHelper->SetLabel(m_robot->GetLabel());
        m_robot->SetLabel(tempLabel);

        newTask->AddStartConstraint(start);
        newTask->AddGoalConstraint(goal);
        newTask->SetLabel("WorkerTask");

        m_myHelper->GetAgent()->SetCurrentTask(newTask);

        // Swap the helper's and the worker's priority.
        int temp = m_myHelper->GetAgent()->m_priority;
        m_myHelper->GetAgent()->m_priority = m_robot->GetAgent()->m_priority;
        m_robot->GetAgent()->m_priority = temp;

        m_pathIndex = 0;
        m_path.clear();
        m_myHelper = nullptr;
        // Use the planning library to find a path.

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
  if(m_robot->GetLabel().compare(0,6,"helper") == 0 && !IsAtChargingStation() && m_task->GetLabel() != "GoingToHelp") {
    FindNearestChargingLocation();
  }
  else if(m_robot->GetLabel().compare(0,6,"helper") == 0 && IsAtChargingStation() && m_task->GetLabel() != "GoingToHelp") {
    m_path.clear();
    this->Halt();
    m_pathIndex = 0;
    //Reset robot priority to 0 while it charges
    m_robot->GetAgent()->m_priority = 0;
    //if battery is low, charge
    m_battery->Charge(5);
    //if battery is full (more than 90%) , available to help
    if(m_battery->GetCurLevel() >= 0.9*m_battery->GetMaxLevel()){
      auto curPos = m_robot->GetDynamicsModel()->GetSimulatedState();
      //TODO: change this so that you don't keep pushing back the same robot
      //maybe use an unordered set?
      auto helpers = m_parentAgent->GetHelpers();
      // If m_robot is not on the helpers list, push it back
      if(find(helpers.begin(), helpers.end(), m_robot) == helpers.end()){
        m_parentAgent->GetHelpers().push_back(m_robot);
      }
      //cout << "Done charging, available to help " << endl;
    }
  }
}


bool
PathFollowingChildAgent::
AvoidCollision() {
  bool valid = false;
  auto dm = m_library->GetDistanceMetric("euclidean");
  auto problem = m_robot->GetMPProblem();
  for(auto& robot : problem->GetRobots()) {
    if(robot != m_robot && (m_pathIndex < m_path.size())
        && robot->GetLabel() != "coordinator") {
      auto current = m_robot->GetDynamicsModel()->GetSimulatedState();
      auto robotPos = robot->GetDynamicsModel()->GetSimulatedState();
      double distanceToSubgoal = EuclideanDistance(current, m_path[m_pathIndex]);
      double distance1 = EuclideanDistance(current, robotPos);
      double distance2 = EuclideanDistance(m_path[m_pathIndex], robotPos);
      //TODO: Pick more accurate threshold than 3
      if(((distance1 + distance2) - distanceToSubgoal) < 3) {
        if(m_avoidCollisionHalt == 0) this->Halt();

        CfgType last = m_path[m_pathIndex-1];
        CfgType next = m_path[m_pathIndex];
        CfgType newCfg = next;

        Vector3d myV = current.GetLinearPosition();
        Vector3d nextV = next.GetLinearPosition();
        Vector3d otherV = robotPos.GetLinearPosition();


        if(abs(myV[0]-otherV[0]) < 2){
          nextV[0] = otherV[0]+4;
        }
        if(abs(myV[1]-otherV[1]) < 2){
          nextV[1] = otherV[1]+4;
        }
        newCfg.SetLinearPosition(nextV);

        if(newCfg.InBounds(m_robot->GetMPProblem()->GetEnvironment())) {
          //cout << m_robot->GetLabel() << " Cfg is in free space " << endl;
          valid = true;
          m_path.insert(m_path.begin()+m_pathIndex,newCfg);
        }
        else {
          valid = false;
        }

        
      }
    }
  }
  return valid;
}

double
PathFollowingChildAgent::
GetPathLength(vector<Cfg>& path){
  double distance = 0;
  for(size_t i = 0; i < path.size()-1; ++i){
    distance += EuclideanDistance(path[i], path[i+1]);
  }
  return distance;
}

double
PathFollowingChildAgent::
EuclideanDistance(Cfg point1, Cfg point2){
  double x_dist = pow((point1[0]-point2[0]),2);
  double y_dist = pow((point1[1]-point2[1]),2);
  return sqrt(x_dist + y_dist);
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
