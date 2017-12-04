#include "PathFollowingChildAgent.h"

#include <limits>
#include <unordered_map>
#include <algorithm>
#include "ConfigurationSpace/Cfg.h"
#include "Behaviors/Agents/BatteryConstrainedGroup.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include "MPProblem/Robot/HardwareInterfaces/ArucoDetectorInterface.h"
#include "MPProblem/Robot/HardwareInterfaces/QueuedHardwareInterface.h"

/*------------------------------ Construction --------------------------------*/

PathFollowingChildAgent::
PathFollowingChildAgent(Robot* const _r) : Agent(_r) {
}


PathFollowingChildAgent::
~PathFollowingChildAgent() {
  // Ensure agent is properly torn down.
  Uninitialize();
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

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  m_robotPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  
  // Copy this robot's first task so that it uses the parent robot pointer.
  auto firstTask = problem->GetTasks(m_robot).front();

  m_task = new MPTask(m_parentRobot);
  for(auto& constraint : firstTask->GetStartConstraints())
    m_task->AddStartConstraint(constraint);
  for(auto& constraint : firstTask->GetPathConstraints())
    m_task->AddPathConstraint(constraint);
  for(auto& constraint : firstTask->GetGoalConstraints())
    m_task->AddGoalConstraint(constraint);

}

const bool
PathFollowingChildAgent::
IsHeadOnCollision() {
  if(m_pathIndex >= m_path.size())
    return false;
  auto problem = m_robot->GetMPProblem();

  // Define the threshold for avoiding collisions. Plan to go around if the
  // robot's centers would come closer than this amount.
  /// @TODO This could be made a parameter if you want to dynamically adjust how
  ///       close the robots can get.
  const double threshold = 2 * m_robot->GetMultiBody()->GetBoundingSphereRadius();

  // Check this robot against all others to see if it is about to hit something.
  const Cfg myCfg   = m_robot->GetDynamicsModel()->GetSimulatedState(),
            subGoal = m_path[m_pathIndex];

  const Vector3d myPoint = myCfg.GetPoint(),
                 toGoal  = subGoal.GetPoint() - myPoint;

  for(auto& robot : problem->GetRobots()) {
    // Skip self and coordinator.
    if(robot == m_robot or robot->GetLabel() == "coordinator")
      continue;

    const Cfg theirCfg = robot->GetDynamicsModel()->GetSimulatedState();

    const Vector3d toThem = theirCfg.GetPoint() - myPoint;

    const double clearance = toThem.orth(toGoal).norm();

    auto robotPosition = robot->GetDynamicsModel()->GetSimulatedState();
    auto myPosition = m_robot->GetDynamicsModel()->GetSimulatedState();
    double distance = EuclideanDistance(robotPosition, myPosition);

    if(clearance < threshold && distance < 3*threshold) {
      //cout << m_robot->GetLabel() << " is in head on collision with " << robot->GetLabel() << endl;
      if(m_headOnCollidingRobot != robot) {
        this->Halt();
        PauseHardwareAgent(m_dt);
        cout << "Goal: " << m_path[m_path.size()-1] << endl;
        cout << "Position of other robots: " << endl;
        for (auto& robot : problem->GetRobots()) {
          if(robot == m_robot or robot->GetLabel() == "coordinator")
            continue;
          cout << robot->GetLabel() << ": " <<
            robot->GetDynamicsModel()->GetSimulatedState() << endl;
        }
        cout << "\n\n" << m_robot->GetLabel() << " at position " << m_robot->GetDynamicsModel()->GetSimulatedState() << endl;
        cout << m_robot->GetLabel() << " in head on collision ******************************************************************** " << endl;
        cout << "with robot at position " << robot->GetDynamicsModel()->GetSimulatedState() << endl;
        cout << endl;
        cout << endl;
        AvoidCollision();
        m_headOnCollidingRobot = robot;
        return true;
      }
    }
    else {
      m_headOnCollidingRobot = nullptr;
    }
  }
  return false;
}


void
PathFollowingChildAgent::
Step(const double _dt) {

  Initialize();
  
  //Skip till you match the hardware time.
  auto hardware = m_robot->GetHardwareInterface("base");
  if(hardware) {
    const double hardwareTime = hardware->GetCommunicationTime();

    m_dt += _dt;
    if(m_dt < hardwareTime)
      return;
  }
  else
    m_dt = _dt;
  
  //If we have moved 1.0 meters, localize. The error seems high just after 1 m, so
  //we would need to localize often.
  if(m_distance > 1.0) {
    Localize(m_dt);
    LocalizeAngle(m_dt);
    m_dt = 0;
    //if(!m_finishedLocalizing)
    return;
  }

  if(m_parentAgent->IsHelper(m_robot))
    HelperStep(m_dt);
  else
    WorkerStep(m_dt);

  // If not in collision, keep going with current task.
  const bool collision = InCollision();
  bool inHeadonCollision = false;
  if(!collision) {
    m_shouldHalt = false;
    m_headOnCollidingRobot = nullptr;
  }
  // Otherwise, check if it should stop/plan around.
  else if(m_shouldHalt) {
    this->Halt();
    //halt and return, don't call Execute task
    //PauseSimulatedAgent(m_dt);
    PauseHardwareAgent(m_dt);
    m_dt = 0;
    return;
  }
  else 
    inHeadonCollision = IsHeadOnCollision();

  //If the tasked is assigned but not started
  if(m_task && !m_task->IsStarted()) {
    //If in headon collision then skip this step and don't plan the task.
    //Lazy PRM will tak care of it in the next time step.
    if(!inHeadonCollision)
      GetNextPath(m_task);
    else {
      m_dt = 0;
      return;
    }
  }
  
  ExecuteTask(m_dt);
  m_dt = 0;
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

/*------------------------------ Helpers -------------------------------------*/

void
PathFollowingChildAgent::
GetNextPath(MPTask* const _task, const bool _collisionAvoidance) {
  // Ask the coordinator to give us the next path.
  cout << m_robot->GetLabel() << "planning for task " << _task->GetLabel() << endl;
  cout << "Original path " <<  endl;
  for(auto i : m_path)
    cout << i;
  cout << endl;
  m_robot->SetVirtual(true);
  m_path = m_parentAgent->MakeNextPlan(_task, _collisionAvoidance);
  m_robot->SetVirtual(false);
  m_robot->SynchronizeModels();
  cout << "\nNew path " <<  endl;
  for(auto i : m_path)
    cout << i;
  cout << endl;
  m_pathIndex = 0;
  //this->SetCurrentTask(_task);
  m_task->SetStarted();
}


const bool
PathFollowingChildAgent::
InCollision() {
  auto dm = m_library->GetDistanceMetric("euclidean");
  auto problem = m_robot->GetMPProblem();

  bool coll = false;

  for(auto& robot : problem->GetRobots()) {
    if(robot->GetLabel() == "coordinator" or robot == m_robot)
      continue;

    auto robotPosition = robot->GetDynamicsModel()->GetSimulatedState();
    auto myPosition = m_robot->GetDynamicsModel()->GetSimulatedState();
    double distance = EuclideanDistance(robotPosition, myPosition);

    // If the other robot has higher priority, halt this robot.
    if(distance < 8. * m_robot->GetMultiBody()->GetBoundingSphereRadius()) {

      if(m_robot->GetAgent()->m_priority < robot->GetAgent()->m_priority){
        m_shouldHalt = true;
        return true;
      }
      else{
        m_shouldHalt = false;
        coll = true;
      }
    }
  }
  return coll;
}


MPTask*
PathFollowingChildAgent::
GetNewTask() {
  //use the parent robot
  auto task = new MPTask(m_parentRobot);
  auto currentPos = m_robot->GetDynamicsModel()->GetSimulatedState();

  // If the new goal has no robot pointer, that means there wasn't one.
  const Cfg newGoal = m_parentAgent->GetRandomRoadmapPoint();
  if(newGoal.GetRobot() == nullptr)
  {
    m_done = true;
    return nullptr;
  }

  auto start = new CSpaceConstraint(m_parentRobot, currentPos);
  auto goal = new CSpaceConstraint(m_parentRobot, newGoal);
  task->AddStartConstraint(start);
  task->AddGoalConstraint(goal);

  return task;
}


const bool
PathFollowingChildAgent::
CallForHelp() {
  // If there are no helpers available, the call fails.
  if(m_parentAgent->GetHelpers().empty())
    return false;

  int nearestIndex = GetNearestHelper();
  auto helper = m_parentAgent->GetHelpers().at(nearestIndex);
  //Set new task for helper to get to worker position
  auto workerPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto helperPos = helper->GetDynamicsModel()->GetSimulatedState();

  auto helperTask = new MPTask(m_parentRobot);
  auto start = new CSpaceConstraint(m_parentRobot, helperPos);
  auto goal = new CSpaceConstraint(m_parentRobot, workerPos);

  //let helper get close enough to worker
  //assign worker's old goal to new worker
  helperTask->AddStartConstraint(start);
  helperTask->AddGoalConstraint(goal);
  helperTask->SetLabel("GoingToHelp");

  m_pathIndex = 0;
  m_path.clear();
  helper->GetAgent()->SetCurrentTask(helperTask);
  m_myHelper = helper;
  //TODO Find a better way to assign priorities
  m_myHelper->GetAgent()->m_priority = m_robot->GetAgent()->m_priority-100;

  m_parentAgent->GetHelpers().erase(
      m_parentAgent->GetHelpers().begin() + nearestIndex);

  // Set the helper's charging station to open
  ClearChargingStation();
  return true;
}


const int
PathFollowingChildAgent::
GetNearestHelper() {
  int count = 0, index = 0;
  const Cfg workerPos = m_robot->GetDynamicsModel()->GetSimulatedState();

  double nearestDistance = std::numeric_limits<double>::max();

  for(auto helper : m_parentAgent->GetHelpers()) {
    const double distance = EuclideanDistance(workerPos,
        helper->GetDynamicsModel()->GetSimulatedState());

    if(distance < nearestDistance) {
      nearestDistance = distance;
      index = count;
    }
    count++;
  }
  return index;
}


const bool
PathFollowingChildAgent::
IsAtChargingStation() {
  const Cfg currentPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  const double threshold = .4;

  for(const auto& chargingLocation :
      m_parentAgent->GetChargingLocations() ) {

    const double distance = EuclideanDistance(
        currentPos, chargingLocation.first);

    if(distance <= threshold){
      return true;
    }
  }

  return false;
}


void
PathFollowingChildAgent::
ClearChargingStation() {
  // Clear the charging station that m_myHelper was on
  const Cfg currentPos = m_myHelper->GetDynamicsModel()->GetSimulatedState();
  const double threshold = .5;

  for(auto& chargingLocation : m_parentAgent->GetChargingLocations()) {
    const double distance = EuclideanDistance(currentPos,
        chargingLocation.first);

    if(distance <= threshold) {
      chargingLocation.second = false;
      cout << "No More Robot On Charging Location: " << chargingLocation.first << endl;
      return;
    }
  }
}


void
PathFollowingChildAgent::
FindNearestChargingLocation() {
  if(m_task->GetLabel() == "GettingToChargingLocation")
    return;

  auto task = new MPTask(m_parentRobot);
  auto currentPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto start = new CSpaceConstraint(m_parentRobot, currentPos);

  vector<Cfg> tempPath;
  Cfg chargingLocation;
  double pathLength = 0;
  pair<Cfg, bool>* chosenLocation = nullptr;
  // Find the closest charging location to m_robot
  for(auto tempLocation : m_parentAgent->GetChargingLocations()) {
    auto tempGoal = new CSpaceConstraint(m_parentRobot, tempLocation.first);
    auto tempTask = new MPTask(m_parentRobot);
    // Create a temporary task and solve it to get path length
    tempTask->AddStartConstraint(start);
    tempTask->AddGoalConstraint(tempGoal);
    GetNextPath(tempTask);
    tempPath = m_path;

    // If the temp task has the shortest path length and it is free, store the task
    /// @TODO Switch planning to weighted euclidean distance which ignores
    ///       rotations, then we can use the Path::Length function.
    double tempLength = GetPathLength(tempPath);

    if((tempLength < pathLength || pathLength == 0) and
        tempLocation.second == false) {

      pathLength = tempLength;
      task = tempTask;
      chargingLocation = tempLocation.first;
      chosenLocation = &tempLocation;
    }
  }

  chosenLocation->second = true;
  task->SetLabel("GettingToChargingLocation");
  this->SetCurrentTask(task);
}


void
PathFollowingChildAgent::
Rotate(double& _angle, double _dt) {

  Cfg point(m_robot);
  std::istringstream pointStream("0 0 0 0 0 0");
  point.Read(pointStream);

  //TODO: Hacky solution to make it rotate. Could be done in a better way,
  //maybe?
  Cfg point2(m_robot);
  std::istringstream pointStream2("0 0 "+to_string(_angle/M_PI)+" 0 0 0");
  point2.Read(pointStream2);

  auto bestControl = m_robot->GetController()->operator()
    (point2, point, _dt);

  bestControl.Execute();
  auto hardwareInterface = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base")); //TODO Magic string

  if(hardwareInterface) {
    hardwareInterface->EnqueueCommand({bestControl}, _dt);
    vector<double> temp = bestControl.GetForce();
    _angle -= abs(temp[2]*_dt);
  }
}


void
PathFollowingChildAgent::
Localize(double _dt) {

  if(m_ignoreLocalization)
    return;
  m_ignoreAngleLocalization = true;
  m_finishedLocalizing = false;

  auto hardwareInterface = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base")); //TODO Magic string

  // Rotate for about 90 degrees
  if(abs(m_localizingAngle) > 0.2)
    Rotate(m_localizingAngle, _dt);
  else {
    //PauseSimulatedAgent(_dt);
    this->Halt();
    if(hardwareInterface->IsIdle()) {
      //cout << "Coordinates in the simulator: " <<
        //m_robot->GetDynamicsModel()->GetSimulatedState() << endl;

      ArucoDetectorInterface* netbook = static_cast<ArucoDetectorInterface*>
        (m_robot->GetHardwareInterface("netbook")); //TODO Magic string

      vector<double> coordinates;
      for(int i = 0; i< 2; i++)
        coordinates = netbook->GetCoordinatesFromMarker();

      if(!coordinates.empty()) {
        /*cout << "Coordinates from markers: " << endl;
        for(auto info : coordinates)
          cout << info << ", ";
        cout << endl;*/
        coordinates[2] = coordinates[2]/(180); //Get it in the range [-1, 1]
        m_coordinates.push_back(coordinates);
      }

      if(m_totalRotations >= 4) {
        m_totalRotations = 0;

        double x = 0, y = 0, ang = 0;
        for(auto vec : m_coordinates) {
          x += vec[0];
          y += vec[1];
          ang += vec[2];
        }

        x = x/m_coordinates.size();
        y = y/m_coordinates.size();
        ang = ang/m_coordinates.size();

        m_coordinates.clear();
        vector<double> finalPos{x,y,ang};

        Cfg robotPos(m_robot);
        m_robotPos = robotPos;
        m_robotPos.SetData(finalPos);

        m_ignoreAngleLocalization = false;

        //cout << "Finished with positional localization, "
          //"angle will be calculated next. " << endl;
        return;
      }
      else {
        cout << "Changing angle " << endl;
        // reset this value to 90 degrees; TODO: don't use magic numbers
        m_localizingAngle = 1.57;
        m_totalRotations++;
        //cout << netbook->GetNumMarkersSeen() <<
          //" Markers found. Rotating more." << endl;
      }
    }
  }
  return;
}


void
PathFollowingChildAgent::
LocalizeAngle(double _dt) {
  if(m_ignoreAngleLocalization)
    return;
  m_ignoreLocalization = true;

  auto hardwareInterface = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base")); //TODO Magic string

  // Rotate till angle is almost 0
  if(abs(m_localizingAngle) > 0.2)
    Rotate(m_localizingAngle, _dt);
  else {
    this->Halt();
    //PauseSimulatedAgent(_dt);

    if(hardwareInterface->IsIdle()) {
      //cout << "Calculating angle now " << endl;
      //cout << "Coordinates in the simulator: " <<
        //m_robot->GetDynamicsModel()->GetSimulatedState() << endl;

      ArucoDetectorInterface* netbook = static_cast<ArucoDetectorInterface*>
        (m_robot->GetHardwareInterface("netbook")); //TODO Magic string

      vector<double> coordinates;
      for(int i = 0; i< 2; i++)
        coordinates = netbook->GetCoordinatesFromMarker();

      if(!coordinates.empty()) {
        coordinates[2] = coordinates[2]/(180); //Get it in the range [-1, 1]
        Cfg tempPos(m_robot);
        tempPos.SetData(coordinates);

        //auto dist = EuclideanDistance(m_robotPos, tempPos);
        /*cout << "Position calculated after all markers: " << m_robotPos << endl;
        cout << "temporary position in after 360 rotation: " << tempPos << endl;
        cout << "Distance between the physical robot and temppos: " << dist << endl;*/

        if(EuclideanDistance(m_robotPos, tempPos) < 0.4) {
          m_distance = 0;
          m_ignoreLocalization = false;
          m_finishedLocalizing = true;

          Cfg actualPhysicalPos(m_robot);
          actualPhysicalPos.SetData({m_robotPos[0],m_robotPos[1], coordinates[2]});
          m_robot->GetDynamicsModel()->SetSimulatedState(actualPhysicalPos);
          Replan();
          return;
        }
        else {
          m_localizingAngle = 1.57;
          return;
        }
      }
      else {
        // reset this value to 60 degrees; TODO: don't use magic numbers
        m_localizingAngle = 1.57;
        //cout << netbook->GetNumMarkersSeen() << " Markers found. Rotating more." << endl;
      }
    }
  }
  return;
}


void
PathFollowingChildAgent::
ExecuteTask(double _dt) {
  /*if(m_parentAgent->IsHelper(m_robot)) {
    cout << endl;
    cout << endl;
    cout << "IN EXECUTE TASK FUNCTION" << endl;
    cout << m_robot->GetLabel() << " executing task " << endl;
    cout << m_path.size() << ", " << m_finishedLocalizing << ", " << m_pathIndex << endl;
  }*/
  if(!m_finishedLocalizing)
    return;
  // Do nothing if there are no unvisited points left 
  auto hardwareInterface = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base")); //TODO Magic string

  if(m_pathIndex >= m_path.size() or m_path.empty()) {
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

  while(distance < threshold and m_pathIndex < m_path.size() ) {
    if(m_debug)
      std::cout << "\tReached waypoint " << m_pathIndex << " at "
        << distance << "/" << threshold << std::endl
        << "Waypoint = " << m_path[m_pathIndex] << std::endl;

    // Move to next cfg in path since the distance is within the threshold.
    ++m_pathIndex;

    // Break if we try to go beyond the path's end. Necessary, as calculating
    // the distance on an undefined cfg will crash some systems.
    if(m_pathIndex >= m_path.size())
      break;

    distance = EuclideanDistance(current, m_path[m_pathIndex]);
  }

  // If we hit the end return.
  if(m_pathIndex >= m_path.size()) {
    //if(m_debug)
    std::cout << m_robot->GetLabel() << " : Reached the end of the path." << std::endl;

    cout << m_robot->GetDynamicsModel()->GetSimulatedState() << endl;
    this->Halt();
    //PauseSimulatedAgent(_dt);
    m_task->SetCompleted();
    m_path.clear();
    m_pathIndex = 0;
    //Reset values for certain variables;
    m_headOnCollidingRobot = nullptr;
    return;
  }

  // Otherwise, execute the control that is nearest to the desired force.
  auto bestControl = m_robot->GetController()->operator()(current,
      m_path[m_pathIndex], _dt);
  bestControl.Execute();

  //cout << "this is the control " << bestControl << endl;
  //cout << endl;
  //cout << endl;
  if(hardwareInterface) {
    //Keep track of the distance the the robot has moved so far. We would
    //localize after every few meters (or whatever distance we think is needed)
    m_distance += _dt*bestControl.GetForce()[0];
    hardwareInterface->EnqueueCommand({bestControl}, _dt);
  }
}


void
PathFollowingChildAgent::
PauseSimulatedAgent(double _dt) {

  Cfg point(m_robot);
  std::istringstream pointStream("0 0 0 0 0 0");
  point.Read(pointStream);

  auto emptyControl = m_robot->GetController()->operator()(point, point, _dt);

  emptyControl.Execute();
}


void
PathFollowingChildAgent::
PauseHardwareAgent(double _dt) {

  Cfg point(m_robot);
  std::istringstream pointStream("0 0 0 0 0 0");
  point.Read(pointStream);

  auto emptyControl = m_robot->GetController()->operator()(point, point, _dt);
  auto hardwareInterface = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base")); //TODO Magic string
  if(hardwareInterface)
    hardwareInterface->EnqueueCommand({emptyControl}, _dt);
}


void
PathFollowingChildAgent::
Replan() {
  if(m_pathIndex >= m_path.size() or m_path.empty())
    return;
  // Create a new task with the same goal as before
  // using current position as the start
  auto task = new MPTask(m_parentRobot);

  auto curPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto start = new CSpaceConstraint(m_parentRobot, curPos);

  //Get the same goal from the current path
  auto goalPos = m_path.back();
  auto goal = new CSpaceConstraint(m_parentRobot, goalPos);

  task->AddStartConstraint(start);
  task->AddGoalConstraint(goal);
  task->SetLabel(m_task->GetLabel());

  m_pathIndex = 0;
  m_path.clear();

  this->SetCurrentTask(task);
}


void
PathFollowingChildAgent::
WorkerStep(double _dt) {
  if(m_done)
    return;
  
  auto hardwareInterface = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base")); //TODO Magic string

  if(hardwareInterface)
    m_battery->UpdateValue(0.25);
  else
    m_battery->UpdateValue(.05);
  
  if(m_task->IsCompleted()) {
    m_pathIndex = 0;
    m_path.clear();
    this->SetCurrentTask(GetNewTask());
  }
  if((m_battery->GetCurLevel() < 0.2*m_battery->GetMaxLevel())) {
    if(!m_myHelper) {
      // if you don't have a helper yet, call for help
      if(!CallForHelp())
        return;
      if(m_path.size() < m_pathIndex) 
        m_currentGoal = m_path.back();
      else
        m_currentGoal = m_parentAgent->GetRandomRoadmapPoint();
      m_pathIndex = 0;
      m_path.clear();
      this->Halt();
      //PauseSimulatedAgent(_dt);
      PauseHardwareAgent(_dt);
      //this->Halt();
      m_distance = 1.2;
      cout << "Called for help at position " << m_robot->GetDynamicsModel()->GetSimulatedState() << endl;
    }
    // else check if the helper is close enough to you to take over
    else if(m_myHelper) {
      auto helperPos = m_myHelper->GetDynamicsModel()->GetSimulatedState();
      auto myPos = m_robot->GetDynamicsModel()->GetSimulatedState();
      double distance = EuclideanDistance(myPos, helperPos);

      //If the helper is close enough, initiate the behavior swap
      //TODO: Change the threshold
      if(distance < 9. * m_robot->GetMultiBody()->GetBoundingSphereRadius()) {
        // Get this robot's task and assign it to worker. Get the goal constraint from the
        // worker's task

        // Swap the helper's and the worker's labels.
        string tempLabel = m_myHelper->GetLabel();
        m_myHelper->SetLabel(m_robot->GetLabel());
        m_robot->SetLabel(tempLabel);

        // Swap the helper's and the worker's priority.
        int temp = m_myHelper->GetAgent()->m_priority;
        m_myHelper->GetAgent()->m_priority = m_robot->GetAgent()->m_priority;
        m_robot->GetAgent()->m_priority = temp;

        //Create a new task for the new worker. The new worker get the same goal
        //as the previous worker
        auto newTask = new MPTask(m_parentRobot);
        auto helperPos = m_myHelper->GetDynamicsModel()->GetSimulatedState();
        auto start = new CSpaceConstraint(m_parentRobot, helperPos);
        newTask->AddStartConstraint(start);
       
        auto goalPos = m_currentGoal;
        cout << "Seg faulting here: " << goalPos << endl;
        auto goal = new CSpaceConstraint(m_parentRobot, goalPos);

        newTask->AddGoalConstraint(goal);
        
        newTask->SetLabel("WorkerTask");

        m_myHelper->GetAgent()->SetCurrentTask(newTask);

        m_pathIndex = 0;
        m_path.clear();
        m_myHelper = nullptr;
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
HelperStep(double _dt) {
  if(m_task->GetLabel() == "GoingToHelp")
    return;
  if(!IsAtChargingStation()) 
    FindNearestChargingLocation();
  else {
    //Clear the current path and halt the robot
    m_path.clear();
    m_pathIndex = 0;
    this->Halt();
    PauseHardwareAgent(_dt);
    //Reset robot priority to 0 while it charges
    m_robot->GetAgent()->m_priority = 0;
    //if battery is low, charge
    m_battery->Charge(5);
    //if battery is full (more than 90%) , available to help
    if(m_battery->GetCurLevel() >= 0.9*m_battery->GetMaxLevel()){
      auto curPos = m_robot->GetDynamicsModel()->GetSimulatedState();
      auto helpers = m_parentAgent->GetHelpers();
      // If m_robot is not on the helpers list, push it back
      if(find(helpers.begin(), helpers.end(), m_robot) == helpers.end())
        m_parentAgent->GetHelpers().push_back(m_robot);
    }
  }
}


void
PathFollowingChildAgent::
AvoidCollision() {
  //TODO Make new path using different node than intitial path planning.
  CfgType currentGoal = m_path.back();
  
  //Checks that goal is not in collision with halted agent
  //Would be better if LazyQuery could return that it didn't find a path
  for (auto& robot: m_robot->GetMPProblem()->GetRobots()){
    if(robot->GetLabel() == "coordinator" or robot == m_robot)
      continue;
    auto robotPosition = robot->GetDynamicsModel()->GetSimulatedState();
    auto myPosition = m_robot->GetDynamicsModel()->GetSimulatedState();
    double distance = EuclideanDistance(robotPosition, myPosition);
    double goalDistance = EuclideanDistance(robotPosition, currentGoal);
    while(distance < 8. * m_robot->GetMultiBody()->GetBoundingSphereRadius()
        && goalDistance <
        2. * m_robot->GetMultiBody()->GetBoundingSphereRadius()) {
      currentGoal = m_parentAgent->GetRandomRoadmapPoint();
      goalDistance = EuclideanDistance(robotPosition, currentGoal);
      cout << "Goal distance is: " << goalDistance << endl;
    }
  }

  CfgType currentPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto goal = new CSpaceConstraint(m_parentRobot, currentGoal);
  auto start = new CSpaceConstraint(m_parentRobot, currentPos);
  auto task = new MPTask(m_parentRobot);

  task->AddStartConstraint(start);
  task->AddGoalConstraint(goal);
  task->SetLabel("CollisionAvoidanceTask");

  //Use lazy PRM to find a path
  GetNextPath(task, true);
}


const double
PathFollowingChildAgent::
GetPathLength(const vector<Cfg>& _path) const {
  double distance = 0;
  for(size_t i = 0; i < _path.size() - 1; ++i)
    distance += EuclideanDistance(_path[i], _path[i + 1]);
  return distance;
}


const double
PathFollowingChildAgent::
EuclideanDistance(const Cfg& _point1, const Cfg& _point2) const {
  const double x = pow(_point1[0] - _point2[0], 2),
               y = pow(_point1[1] - _point2[1], 2);
  return std::sqrt(x + y);
}
/*----------------------------------------------------------------------------*/
