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
PathFollowingChildAgent(Robot* const _r, XMLNode& _node) {
  // Parse XML parameters.
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
      if(m_headOnCollidingRobot != robot) {
        this->Halt();
        PauseAgent(m_dt);

        if(m_debug){
          for (auto& robot : problem->GetRobots()) {
            if(robot == m_robot or robot->IsVirtual())
              continue;
            cout << robot->GetLabel() << ": " <<
              robot->GetDynamicsModel()->GetSimulatedState() << endl;
          }
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
  Initialize();

  // Skip till you match the hardware time.
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
    PauseAgent(m_dt);
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
}

/*----------------------------- Child Interface ------------------------------*/

bool
PathFollowingChildAgent::
IsBatteryLow() {
  auto battery = m_robot->GetBattery();
  const double threshold = .2 * battery->GetMaxLevel();
  return battery->GetCurLevel() < threshold;
}


bool
PathFollowingChildAgent::
IsBatteryHigh() {
  auto battery = m_robot->GetBattery();
  const double threshold = .9 * battery->GetMaxLevel();
  return battery->GetCurLevel() >= threshold;
}

/*------------------------------ Helpers -------------------------------------*/

void
PathFollowingChildAgent::
ExecuteControl(const Control _c, const double _dt) {
  // Execute the control on the simulated robot.
  _c.Execute();

  // If we are controlling a hardware robot, execute the control on hardware as
  // well.
  auto hardwareInterface = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base")); //TODO Magic string

  if(hardwareInterface)
    hardwareInterface->EnqueueCommand({_c}, _dt);

  // Update odometry tracking.
  /// @TODO This assumes we are using a velocity-based iCreate controller. Make
  ///       this explicitly required or generalize.
  m_distance += _dt * _c.GetForce()[0];

  // Derate the emulated battery.
  // Derate faster when we are using the iCreates so that we don't need
  // to wait forever to see the switch.
  const double depletionRate = hardwareInterface ? .36 : .04;
  m_robot->GetBattery()->UpdateValue(_dt * depletionRate);
}


void
PathFollowingChildAgent::
InCollision() {
  const double distanceThreshold = 6. *
      m_robot->GetMultiBody()->GetBoundingSphereRadius();
  auto nearbyRobots = this->ProximityCheck(distanceThreshold);

  // If we found nearby robots, then there is a potential collision.
  if(!nearbyRobots.empty()) {
    nearbyRobots.push_back(m_robot);
    m_parentAgent->ArbitrateCollision(nearbyRobots);
  }
}


void
PathFollowingChildAgent::
GeneratePlan() {
  // If we have no task, do nothing.
  if(!GetTask())
    return;

  /// TODO
  // Halt agent
  // Synchronize models
  // Copy parent's problem
  // Solve in separate thread
  // Update parent's roadmap
  //m_library->Solve(m_robot->GetMPProblem(), GetTask(), m_solution,
  //    "LazyPRM", LRand(), "LazyCollisionAvoidance");

  return m_solution->GetPath()->Cfgs();
}

/*----------------------------------------------------------------------------*/

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

  // Execute the control.
  ExecuteControl(bestControl, _dt);
  vector<double> temp = bestControl.GetForce();
  _angle -= abs(temp[2]*_dt);
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
    //PauseAgent(_dt);
    this->Halt();
    if(hardwareInterface->IsIdle()) {
      //cout << "Coordinates in the simulator: " <<
        //m_robot->GetDynamicsModel()->GetSimulatedState() << endl;

      ArucoDetectorInterface* netbook = static_cast<ArucoDetectorInterface*>
        (m_robot->GetHardwareInterface("netbook")); //TODO Magic string

      vector< vector<double> > allCoordinates;

      vector<double> coordinates;
      for(int i = 0; i< 2; i++)
        coordinates = netbook->GetCoordinatesFromMarker();

      if(!coordinates.empty() and netbook->GetNumMarkersSeen() >= 2) {
        /*cout << "Coordinates from markers: " << endl;
        for(auto info : coordinates)
          cout << info << ", ";
        cout << endl;*/
        coordinates[2] = coordinates[2]/(180); //Get it in the range [-1, 1]
        allCoordinates.push_back(coordinates);
      }

      if(m_totalRotations >= 4 or netbook->GetNumMarkersSeen() >= 3) {
        m_totalRotations = 0;

        double x = 0, y = 0, ang = 0;
        for(auto vec : allCoordinates) {
          x += vec[0];
          y += vec[1];
          ang += vec[2];
        }

        x = x/allCoordinates.size();
        y = y/allCoordinates.size();
        ang = ang/allCoordinates.size();

        vector<double> finalPos{x,y,ang};

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
    //PauseAgent(_dt);

    if(hardwareInterface->IsIdle()) {

      ArucoDetectorInterface* netbook = static_cast<ArucoDetectorInterface*>
        (m_robot->GetHardwareInterface("netbook")); //TODO Magic string

      vector<double> coordinates;
      for(int i = 0; i< 2; i++)
        coordinates = netbook->GetCoordinatesFromMarker();

      if(!coordinates.empty()) {
        coordinates[2] = coordinates[2]/(180); //Get it in the range [-1, 1]
        Cfg tempPos(m_robot);
        tempPos.SetData(coordinates);

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
          {
            // Replan(); This function just recreated the same task and did not
            // replan.
          }
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
  if(m_pathIndex >= m_path.size() or m_path.empty())
    return;

  // Get the current configuration.
  const Cfg current = m_robot->GetDynamicsModel()->GetSimulatedState();

  // We consider the robot to have reached the next subgoal if it is within a
  // threshold distance. Advance the path index until the next subgoal is
  // at least one threshold away.
  auto dm = m_library->GetDistanceMetric("euclidean");

  const double threshold = .05;
  double distance = dm->Distance(current, m_path[m_pathIndex]);
  while(distance < threshold and m_pathIndex < m_path.size() ) {
    // Move to next cfg in path since the distance is within the threshold.
    ++m_pathIndex;

    // Break if we try to go beyond the path's end. Necessary, as calculating
    // the distance on an undefined cfg will crash some systems.
    if(m_pathIndex >= m_path.size())
      break;

    distance = dm->Distance(current, m_path[m_pathIndex]);
  }

  // If we hit the end return.
  if(m_pathIndex >= m_path.size()) {
    this->Halt();
    m_task->SetCompleted();
    m_path.clear();
    m_pathIndex = 0;

    m_headOnCollidingRobot = nullptr;
    return;
  }

  // Otherwise, execute the control that is nearest to the desired force.
  auto bestControl = m_robot->GetController()->operator()(current,
      m_path[m_pathIndex], _dt);

  ExecuteControl(bestControl, _dt);
}


void
PathFollowingChildAgent::
PauseAgent(double _dt) {

  Cfg point(m_robot);
  std::istringstream pointStream("0 0 0 0 0 0");
  point.Read(pointStream);

  auto emptyControl = m_robot->GetController()->operator()(point, point, _dt);

  ExecuteControl(emptyControl, _dt);
}


void
PathFollowingChildAgent::
WorkerStep(double _dt) {
  if(IsBatteryLow()) {
    // if no helper
    //   halt
    //   call for help
    // else if helper is close enough to take over
    //   give task to helper
    //   swap priorities
    //   send worker back to charger
  }

  if(m_task->IsCompleted()) {
    // Get a new task from coordinator
  }
}


void
PathFollowingChildAgent::
HelperStep(double _dt) {
  if(m_task->GetLabel() == "GoingToHelp") {
    // Keep going
  }
  else if(/* not at charging station */)
    // Find nearest charging station
  else {
    // At charging station:
    //Clear the current path and halt the robot
    m_path.clear();
    m_pathIndex = 0;
    this->Halt();
    PauseAgent(_dt);
    // Reset robot priority to 0 while it charges
    // If battery is low, charge
    // If battery is full (more than 90%) , available to help
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

/*----------------------------------------------------------------------------*/
