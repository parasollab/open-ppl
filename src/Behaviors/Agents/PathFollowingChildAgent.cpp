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

#include "nonstd/numerics.h"
#include "nonstd/timer.h"

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

  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  m_robotPos = m_robot->GetDynamicsModel()->GetSimulatedState();

  // Copy this robot's first task so that it uses the parent robot pointer
  // (because this is a shared roadmap method).
  auto firstTask = problem->GetTasks(m_robot).front();
  firstTask->SetRobot(m_parentRobot);
  SetTask(firstTask.get());
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

  for(auto& robotPtr : problem->GetRobots()) {
    auto robot = robotPtr.get();
    // Skip self and coordinator.
    if(robot == m_robot or robot->IsVirtual())
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

        if(m_debug){
          cout << "Goal: " << m_path[m_path.size()-1] << endl;
          cout << "Position of other robots: " << endl;
          for (auto& robot : problem->GetRobots()) {
            if(robot == m_robot or robot->IsVirtual())
              continue;
            cout << robot->GetLabel() << ": " <<
              robot->GetDynamicsModel()->GetSimulatedState() << endl;
          }
          cout << "\n\n" << m_robot->GetLabel() << " at position " << m_robot->GetDynamicsModel()->GetSimulatedState() << endl;
          cout << m_robot->GetLabel() << " in head on collision ******************************************************************** " << endl;
          cout << "with robot at position " << robot->GetDynamicsModel()->GetSimulatedState() << endl;
          cout << endl;
          cout << endl;
        }

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
  nonstd::timer clock;

  clock.restart();

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
  if(m_distance > 2.0) {
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
  InCollision();
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
  m_totalRunTime += clock.elapsed()/1e9;
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
}

/*------------------------------ Helpers -------------------------------------*/

void
PathFollowingChildAgent::
GetNextPath(MPTask* const _task, const bool _collisionAvoidance) {
  // Ask the coordinator to give us the next path.
  m_robot->SetVirtual(true);
  m_path = m_parentAgent->MakeNextPlan(_task, _collisionAvoidance);
  m_robot->SetVirtual(false);
  m_robot->SynchronizeModels();
  m_pathIndex = 0;
  //this->SetCurrentTask(_task);
  m_task->SetStarted();
}


void
PathFollowingChildAgent::
InCollision() {
  const double distanceThreshold = 6. * m_robot->GetMultiBody()->GetBoundingSphereRadius();
  auto nearbyRobots = this->ProximityCheck(distanceThreshold);

  // If we found nearby robots, then there is a potential collision.
  if(!nearbyRobots.empty()) {
    nearbyRobots.push_back(m_robot);
    m_parentAgent->ArbitrateCollision(nearbyRobots);
  }
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
  std::unique_ptr<CSpaceConstraint> start(new CSpaceConstraint(m_parentRobot, helperPos)),
                                    goal(new CSpaceConstraint(m_parentRobot, workerPos));

  //let helper get close enough to worker
  //assign worker's old goal to new worker
  helperTask->AddStartConstraint(start);
  helperTask->AddGoalConstraint(goal);
  helperTask->SetLabel("GoingToHelp");

  helper->GetAgent()->SetCurrentTask(helperTask);

  m_parentAgent->GetHelpers().erase(
      m_parentAgent->GetHelpers().begin() + nearestIndex);

  // Set the helper's charging station to open
  //ClearChargingStation();
  return true;
}


void
PathFollowingChildAgent::
Rotate(double& _angle, double _dt) {

  Cfg point(m_robot);
  point.SetData(std::vector<double>{0, 0, 0, 0, 0, 0});

  //TODO: Hacky solution to make it rotate. Could be done in a better way,
  //maybe?
  Cfg point2(m_robot);
  point2.SetData(std::vector<double>{0, 0, _angle / M_PI, 0, 0, 0});

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

      if(!coordinates.empty() and netbook->GetNumMarkersSeen() >= 2) {
        /*cout << "Coordinates from markers: " << endl;
        for(auto info : coordinates)
          cout << info << ", ";
        cout << endl;*/
        coordinates[2] = coordinates[2]/(180); //Get it in the range [-1, 1]
        m_coordinates.push_back(coordinates);
      }

      if(m_totalRotations >= 4 or netbook->GetNumMarkersSeen() >= 3) {
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
        // reset this value to 90 degrees
        m_localizingAngle = 1.57;
        m_totalRotations++;
        //cout << "Changing angle " << endl;
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

        if(EuclideanDistance(m_robotPos, tempPos) < 0.5) {
          m_distance = 0;
          m_ignoreLocalization = false;
          m_finishedLocalizing = true;

          Cfg actualPhysicalPos(m_robot);
          actualPhysicalPos.SetData({m_robotPos[0],m_robotPos[1], coordinates[2]});
          m_robot->GetDynamicsModel()->SetSimulatedState(actualPhysicalPos);
          if(m_goToSameGoal)
            Replan();
          else{
            GetNextPath(m_task, true);
            m_goToSameGoal = true;
          }
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
    //PauseHardwareAgent(_dt);
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

  auto hardwareInterface = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base")); //TODO Magic string

  // Derate battery faster when we are using the iCreates so that we don't need
  // to wait forever to see the switch.
  if(hardwareInterface)
    m_robot->GetBattery()->UpdateValue(0.36 );
  else
    m_robot->GetBattery()->UpdateValue(.04);

  if(m_task->IsCompleted()) {
    m_pathIndex = 0;
    m_path.clear();
    auto task = GetNewTask();
    cout << "New task " << task << endl;
    if(!task)
      return;
    this->SetCurrentTask(task);
    GetNextPath(m_task);
    m_task->SetStarted();
  }

  if((m_robot->GetBattery()->GetCurLevel() < 0.2*m_robot->GetBattery()->GetMaxLevel())) {

    if(!m_myHelper) {
      // if you don't have a helper yet, call for help
      if(!CallForHelp())
        return;
      if(m_path.size() < m_pathIndex)
        m_currentGoal = m_path.back();
      else {
        m_currentGoal = m_parentAgent->GetRandomRoadmapPoint(m_robot->GetLabel());
        if(!m_currentGoal.GetRobot())
          return;
      }
      m_pathIndex = 0;
      m_path.clear();
      this->Halt();
      //PauseSimulatedAgent(_dt);
      PauseHardwareAgent(_dt);
      //this->Halt();
      if(hardwareInterface)
        m_distance = 2.2;
      cout << "Called for help at position " << m_robot->GetDynamicsModel()->GetSimulatedState() << endl;
    }
    // else check if the helper is close enough to you to take over
    else {
      auto helperPos = m_myHelper->GetDynamicsModel()->GetSimulatedState();
      auto myPos = m_robot->GetDynamicsModel()->GetSimulatedState();
      double distance = EuclideanDistance(myPos, helperPos);

      //If the helper is close enough, initiate the behavior swap
      //TODO: Change the threshold
      if(distance < 7. * m_robot->GetMultiBody()->GetBoundingSphereRadius()) {
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
  }
}


void
PathFollowingChildAgent::
HelperStep(double _dt) {
  if(m_task->GetLabel() == "GoingToHelp")
    return;
//  if(!IsAtChargingStation())
//    FindNearestChargingLocation();
  else {
    //Clear the current path and halt the robot
    m_path.clear();
    m_pathIndex = 0;
    this->Halt();
    PauseHardwareAgent(_dt);
    //Reset robot priority to 0 while it charges
    m_robot->GetAgent()->m_priority = 0;
    //if battery is low, charge
    m_robot->GetBattery()->Charge(1);
    //if battery is full (more than 90%) , available to help
    if(m_robot->GetBattery()->GetCurLevel() >= 0.9*m_robot->GetBattery()->GetMaxLevel()) {
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
    if(robot->IsVirtual() or robot == m_robot)
      continue;
    auto robotPosition = robot->GetDynamicsModel()->GetSimulatedState();
    auto myPosition = m_robot->GetDynamicsModel()->GetSimulatedState();
    double distance = EuclideanDistance(robotPosition, myPosition);
    double goalDistance = EuclideanDistance(robotPosition, currentGoal);
    while(distance < 6. * m_robot->GetMultiBody()->GetBoundingSphereRadius()
        && goalDistance <
        2. * m_robot->GetMultiBody()->GetBoundingSphereRadius()) {
      currentGoal = m_parentAgent->GetRandomRoadmapPoint(m_robot->GetLabel());
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

 // auto hardwareInterface = static_cast<QueuedHardwareInterface*>
   // (m_robot->GetHardwareInterface("base")); //TODO Magic string

  // Set m_distance > 1 so that the robot will localize again
  /*if(hardwareInterface) {
    //m_distance = 2.2;
    /-*for(auto robot : m_robot->GetMPProblem()->GetRobots()){
      if(robot->GetLabel() != "coordiator")
        cout << robot->GetLabel() << " position before localizing " << robot->GetDynamicsModel()->GetSimulatedState() << endl;
    }*-/
    m_goToSameGoal = false;
    this->SetCurrentTask(task);
  }*/
  //else
    // Use lazy PRM to find a path
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
