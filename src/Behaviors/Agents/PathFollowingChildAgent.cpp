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

  // We cannot send commands faster than the hardware can accept them, so skip
  // until the hardware is ready.
  if(WaitForHardware(_dt))
    return;

  // If the current task is completed, get a new task from coordinator.
  if(m_task->IsCompleted()) {
  }

  // If we have a task that is not started, make a plan for it now.
  if(m_task && !m_task->IsStarted()) {
  }

  /* Helper ------------------------------------------------------------------*/

  if(m_task->GetLabel() == "GoingToHelp") {
    // Keep going
  }
  else if(/* not at charging station */)
    // Find nearest charging station
  else {
    // At charging station:
    // Clear the current path and halt the robot
    // Reset robot priority to 0 while it charges
    // If battery is low, charge
    // If battery is full (more than 90%) , available to help
  }

  /* Worker ------------------------------------------------------------------*/

  if(IsBatteryLow()) {
    // if no helper
    //   halt
    //   call for help
    // else if helper is close enough to take over
    //   give task to helper
    //   swap priorities
    //   send worker back to charger
  }

  /*--------------------------------------------------------------------------*/

  ExecuteTask(m_dt);
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
  this->Agent::ExecuteControl(_c, _dt);

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
  auto myTask = GetTask();
  if(!myTask)
    return;

  /// TODO
  // Halt agent
  // Copy problem so that we can plan concurrently with the main simulation.
  MPProblem problem = m_robot->GetMPProblem();

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
ExecuteTask(double _dt) {
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
