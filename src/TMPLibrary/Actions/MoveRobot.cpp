#include "MoveRobot.h"

#include "Behaviors/Agents/PathFollowingAgent.h"

#include "Geometry/Boundaries/CSpaceBoundingSphere.h"

#include "MPLibrary/MPSolution.h"

#include "MPProblem/Constraints/BoundaryConstraint.h"

//MoveRobot::countChecked = 0;

MoveRobot::
MoveRobot(Robot* _robot, const Boundary* _start, const Boundary* _goal, bool _hasObject,
          RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _roadmap, MPLibrary* _library, bool _manipulator) {
  m_robot = _robot;
  m_start = _start;
  m_goal = _goal;
  m_hasObject = _hasObject;
  if(_roadmap) {
    m_roadmapGraph = _roadmap;
    m_providedRoadmap = true;
  }
  else {
    m_roadmapGraph = new RoadmapGraph<Cfg,DefaultWeight<Cfg>>(m_robot);
    m_providedRoadmap = false;
  }
  m_library = _library;
  m_manipulator = _manipulator;

  m_startState.m_robotLocations[_robot] = _start;
  m_resultState.m_robotLocations[_robot] = _goal;

  if(_hasObject) {
    m_startState.m_taskOwners.push_back(_robot);
    m_startState.m_objectLocations.push_back(_start);

    m_resultState.m_taskOwners.push_back(_robot);
    m_resultState.m_objectLocations.push_back(_goal);
  }
}

MoveRobot::
~MoveRobot() = default;

bool
MoveRobot::
CheckPreConditions(const FactLayer* _factLayer) {
  //Check that robot is in the start location
  //MoveRobot::countChecked++;
  auto possibleLocationsIter = _factLayer->m_possibleRobotLocations.find(m_robot);
  auto possibleLocations = possibleLocationsIter->second;
  //auto it = std::find(possibleLocations.begin(), possibleLocations.end(), m_start);
  bool match = false;
  for(auto location : possibleLocations) {
    if(location->GetCenter() == m_start->GetCenter()) {
      match = true;
    }
  }
  //if(it == possibleLocations.end())
  if(!match)
    return false;

  MPTask* originalTask = m_library->GetTask();
  //Plan between the two using existing nodes in the roadmap
  MPTask* task = new MPTask(m_robot);

  //Checks if the robot has the object/task to start the move
  if(m_hasObject) {
    auto objectLocations = _factLayer->m_possibleObjectLocations;
    auto obIt = std::find(objectLocations.begin(), objectLocations.end(), m_start);
    if(obIt == objectLocations.end()) {
      //std::cout << "Does not have the object " << m_robot->GetLabel() << std::endl;
      return false;
    }
    auto objectOwners = _factLayer->m_possibleRobotPayloads;
    auto owIt = std::find(objectOwners.begin(), objectOwners.end(), m_robot);
    if(owIt == objectOwners.end()) {
      //std::cout << "Does not have the object " << m_robot->GetLabel() << std::endl;
      return false;
    }
  }

  if(m_manipulator) {
    auto startBox = m_start->Clone();
    std::cout << "Start Type: " << startBox->Name() << std::endl;

    auto startConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(m_robot, std::move(startBox)));

    auto goalBox = m_goal->Clone();
    std::cout << "Goal Type: " << goalBox->Name() << std::endl;

    auto goalConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(m_robot, std::move(goalBox)));

    task->SetStartConstraint(std::move(startConstraint));
    task->AddGoalConstraint(std::move(goalConstraint));
  }
  else {
    auto radius = 1.2 * (m_robot->GetMultiBody()->GetBoundingSphereRadius());

    std::unique_ptr<CSpaceBoundingSphere> boundingSphere(
        new CSpaceBoundingSphere(m_start->GetCenter(), radius));
    auto startConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(m_robot, std::move(boundingSphere)));

    std::unique_ptr<CSpaceBoundingSphere> boundingSphere2(
        new CSpaceBoundingSphere(m_goal->GetCenter(), radius));

    auto goalBoundary = boundingSphere2->Clone();

    auto goalConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(m_robot, std::move(boundingSphere2)));

    task->SetStartConstraint(std::move(startConstraint));
    task->AddGoalConstraint(std::move(goalConstraint));
  }

  if(m_debug) {
    std::cout << "Checking Preconditions for : " << this->PrintAction() << std::endl;
  }
  //if(!CanBeInLocation(m_robot, goalBoundary.get()))
  //  return false;

  if(m_debug) {
    std::cout << "Planning in MOVEROBOT for " << m_robot->GetLabel() <<  std::endl;
  }

  m_library->SetTask(task);

  PathFollowingAgent* agent = static_cast<PathFollowingAgent*>(m_robot->GetAgent());
  auto solution = agent->GetMPSolution();
  solution->SetRoadmap(agent->GetRobot(),m_roadmapGraph);
  try {
    m_roadmapGraph->SetRobot(m_robot);

    //for(auto vit = m_roadmapGraph->begin(); vit != m_roadmapGraph->end(); vit++){
    //  std::cout << vit->property().PrettyPrint() << " : " << vit->property().GetRobot() << std::endl;
    //  auto start = task->GetStartConstraint();
    //  std::cout << "Satisfied: " << start->Satisfied(vit->property()) << std::endl;
    //}

    if(m_providedRoadmap) {
      m_library->Solve(m_robot->GetMPProblem(), task, solution, "EvaluateMapStrategy",
          LRand(), "CheckPreCondition-Provided");
    }
    else {
      m_library->Solve(m_robot->GetMPProblem(), task, solution, "LazyQuery",
          LRand(), "CheckPreCondition-Generating");
    }
    if(solution->GetPath()->Cfgs().empty()) {
      //if(m_manipulator and !m_hasObject)
        m_used = true;
      return false;
    }
    if(m_manipulator) {
      auto goalCfg = solution->GetPath()->Cfgs()[solution->GetPath()->Cfgs().size() -1];
      auto currentCfg = m_robot->GetSimulationModel()->GetState();
      const size_t numDof = m_robot->GetMultiBody()->PosDOF() +
                            m_robot->GetMultiBody()->OrientationDOF();
      for(size_t i = 0; i < numDof; i++) {
        if(currentCfg[i] != goalCfg[i]) {
          std::cout << "Move requires mobile base" << std::endl;
          m_used = true;
          return false;
        }
      }
    }
    //if(m_hasObject)
      m_cost = solution->GetPath()->Length() + 100;
    std::cout << "Finished planning in MOVEROBOT" << std::endl;
  }
  // Usually indicates that the precheck for the goal constraint didn't work
  // as a result of some bug likely in terrain validity checker
  catch(...) {
    if(m_debug) {
      std::cout << "Caught error in MOVEROBOT for " << m_robot->GetLabel() << std::endl;
    }
    m_used = true;
    m_library->SetTask(originalTask);
    delete task;
    return false;
  }
  //Return true if path exists and false if not
  if(m_debug) {
    std::cout << "Adding MOVEROBOT for "
              << m_robot->GetLabel()
              << "from ";
    for(auto d : m_start->GetCenter()) {
      std::cout << d << " : ";
    }
    std::cout << " to ";
    for(auto d : m_goal->GetCenter()) {
      std::cout << d << " : ";
    }
    std::cout << std::endl
              << "WITH TASK: "
              << m_hasObject
              << std::endl
              << std::endl;
  }
  m_library->SetTask(originalTask);
  delete task;
  return true;
}

std::vector<Robot*>
MoveRobot::
GetRobots() {
  return {m_robot};
}

std::string
MoveRobot::
PrintAction() {
  std::string ret = "MOVEROBOT " + m_robot->GetLabel() + " from ";
  for(auto d : m_start->GetCenter()) {
    ret += std::to_string(d) + " : ";
  }
  ret += " to ";
  for(auto d : m_goal->GetCenter()) {
    ret += std::to_string(d) + " : ";
  }
  if(m_hasObject) {
    ret += "WITH object/task";
  }
  else {
    ret += "withOUT object/task";
  }
  return ret;
}
