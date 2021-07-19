#ifndef PPL_LAZY_QUERY_H_
#define PPL_LAZY_QUERY_H_

#include "MPLibrary/MapEvaluators/LazyQuery.h"
#include "Testing/MPLibrary/MapEvaluators/MapEvaluatorMethodTest.h"
#include "Utilities/MPUtils.h"
#include "MPLibrary/MPSolution.h"

#include "Vector.h"

namespace mathtool {
  class EulerAngle;
  class Transformation;
}

template <typename MPTraits>
class LazyQueryTest : virtual public LazyQuery<MPTraits>,
                      public MapEvaluatorMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::CfgType          CfgType;

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    LazyQueryTest();

    LazyQueryTest(XMLNode& _node);

    ~LazyQueryTest();

    ///@}

  private:

    ///@name Interface Test Function
    ///@{

    virtual TestResult MainFunctionTest() override;

    ///@}

    ///@name Helper Functions
    ///@{
    
    /// Construct a roadmap to test that edges and nodes get 
    /// invalidated properly for a single robot
    void SingleSetup();

    /// Construct a roadmap to test that edges and nodes get 
    /// invalidated properly for a group or robots
    // void GroupSetup();

    ///@}

    ///@name Helper Members
    ///@{

    /// Single helper members
    RoadmapType* m_singleRoadmap;
    MPTask*      m_singleTask;

    /// Group helper members
    // GroupRoadmapType* m_groupRoadmap;
    // GroupTask*        m_groupTask;

    ///@}
};

/*--------------------------- Construction ---------------------------*/
template <typename MPTraits>
LazyQueryTest<MPTraits>::
LazyQueryTest() : QueryMethod<MPTraits>(), LazyQuery<MPTraits>() {}

template <typename MPTraits>
LazyQueryTest<MPTraits>::
LazyQueryTest(XMLNode& _node) : QueryMethod<MPTraits>(), LazyQuery<MPTraits>(_node) {}

template <typename MPTraits>
LazyQueryTest<MPTraits>::
~LazyQueryTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename LazyQueryTest<MPTraits>::TestResult
LazyQueryTest<MPTraits>::
MainFunctionTest() {

  bool passed = true;
  std::string message = "";

  // Test Individual Robot Functionality
  SingleSetup();

  auto singleResult = this->IndividualRobotMainFunction(m_singleRoadmap,
                                                        m_singleTask);

  if(!singleResult) {
    passed = false;
    message = message + "\n\tIndividual robot functionality testing failed "
                "TODO: Be more specific.\n";
  }

  if(!passed) {
    message = "MainFunctionTest::FAILED :(\n" + message;
    return std::make_pair(passed, message);
  }

  // // Test Group Robot Functionality
  // std::pair<typename MPTraits::GroupRoadmapType*, typename MPTraits::GroupTask*> groupDetails = GroupSetup();

  // auto groupResult = this->GroupRobotMainFunction(groupDetails.first,
  //                                                 groupDetails.second);

  // if (!groupResult) {
  //   passed = false;
  //   message = message + "\n\tGroup robot functionality testing failed "
  //               "TODO: Be more specific.\n";
  // }

  // if(passed) {
  //   message = "MainFunctionTest::PASSED!\n";
  // } else {
  //   message = "MainFunctionTest::FAILED :(\n" + message;
  // }
  return std::make_pair(passed, message);
}

/*--------------------------Helper Functions -------------------------*/

template <typename MPTraits>
void
LazyQueryTest<MPTraits>::
SingleSetup() {
  // Get Robot
  auto robot = this->GetMPProblem()->GetRobots()[0].get();

  /// Roadmap Construction
  auto roadmap = RoadmapType(robot);

  // Construct vertices and add to roadmap

  // Start & Goal: Valid robot configurations.
  vector<double> startP { 24.0, -24.0, 0 };
  auto start = roadmap->AddVertex(CfgType(startP, robot));

  vector<double> goalP { 24.0, 24.0, 0 };
  auto goal = roadmap->AddVertex(CfgType(goalP, robot));

  // Vertex 1: Out of boundary vertex.
  vector<double> v1P { 30, 0, 0 };
  auto v1 = roadmap->AddVertex(CfgType(v1P, robot));

  // Vertex 2: In boundary vertex.
  vector<double> v2P { 20, 0, 0 };
  auto v2 = roadmap->AddVertex(CfgType(v2P, robot));

  // Add edges to roadmap with respective weights

  // Edge 1: Passes through object in enviornment
  // Tests to ensure edges are invalidated properly
  roadmap->AddEdge(start, goal, 3.0);

  // Edge 2 & 3: Passes through vertex out of environment
  // boundary. Tests to ensure vertices are invalidated
  // properly.
  roadmap->AddEdge(start, v1, 1.0);
  roadmap->AddEdge(v1, goal, 1.0);

  // Edge 4 & 5: Valid path to goal. Tests to ensure query
  // method can find path after failing first to searches.
  roadmap->AddEdge(start, v2, 4.0);
  roadmap->AddEdge(v2, goal, 4.0);

  /// Task Construction
  MPTask* task(robot);
  // Set robot start and goal constraints
  auto startConstraint = std::unique_ptr<CSpaceConstraint>(
    new CSpaceConstraint(robot, CfgType(startP, robot)));

  auto goalConstraint  = std::unique_ptr<CSpaceConstraint>(
    new CSpaceConstraint(robot, CfgType(goalP, robot)));

  task->SetStartConstraint(std::move(startConstraint));
  task->AddGoalConstraint(std::move(goalConstraint));

  // // Set path constraint
  // auto boundary = this->GetMPProblem()->GetEnviornment()->GetBoundary();

  // std::unique_ptr<BoundaryConstraint> pathConstraint =
  //       std::unique_ptr<BoundaryConstraint>(new BoundaryConstraint(robot, boundary));

  // task->AddPathConstraint(pathConstraint);

  m_singleRoadmap = roadmap;
  m_singleTask    = task;
  // return std::make_pair(roadmap, task);
}

// template <typename MPTraits>
// void
// LazyQueryTest<MPTraits>::
// GroupSetup() {

//   // Get Robot group
//   auto robotGroup = this->GetMPProblem()->GetRobotGroup(0);

//   // Robot 1 Setup
//   auto robot1 = robotGroup[0];
//   auto roadmap1 = RoadmapType(robot1);

//   // Construct vertices and add to roadmap for robot 1
//   // Start & Goal: Valid robot 1 configurations.
//   vector<double> start1P { 24.0, -24.0, 0 };
//   auto start1 = roadmap1->AddVertex(CfgType(start1P, robot1));

//   vector<double> goal1P { 24.0, 24.0, 0 };
//   auto goal1 = roadmap1->AddVertex(CfgType(goal1P, robot1));

//   // Vertex 1: Out of boundary vertex for robot 1 roadmap.
//   vector<double> v1P { 30, 0, 0 };
//   auto v1 = roadmap1->AddVertex(CfgType(v1P, robot1));

//   // Vertex 2: In boundary vertex for robot 1 roadmap.
//   vector<double> v2P { 20, 0, 0 };
//   auto v2 = roadmap1->AddVertex(CfgType(v2P, robot1));

//   // Add edges to roadmap 1 with respective weights

//   // Edge 1: Passes through object in enviornment
//   // Tests to ensure edges are invalidated properly
//   roadmap1->AddEdge(start1, goal1, 3.0);

//   // Edge 2 & 3: Passes through vertex out of environment
//   // boundary. Tests to ensure vertices are invalidated
//   // properly.
//   roadmap1->AddEdge(start1, v1, 1.0);
//   roadmap1->AddEdge(v1, goal1, 1.0);

//   // Edge 4 & 5: Valid path to goal. Tests to ensure query
//   // method can find path after failing first search attempts.
//   roadmap1->AddEdge(start1, v2, 4.0);
//   roadmap1->AddEdge(v2, goal1, 4.0);

//   // Robot 2 setup
//   auto robot2 = robotGroup[0];
//   auto roadmap2 = RoadmapType(robot2);

//   // Construct vertices amd add to roadmap for robot 2
//   vector<double> start2P { -24.0, -24.0, 0.0 };
//   auto start2 = roadmap2->AddVertex(CfgType(start2P, robot2));

//   vector<double> goal2P { -24.0, 24.0, 0.0 };
//   auto goal2 = roadmap2->AddVertex(CfgType(goal2P, robot2));

//   // Vertex 3: Out of bounds vertex for robot 2 roadmap.
//   vector<double> v3P { -30.0, 0.0, 0.0 };
//   auto v3 = roadmap2->AddVertex(CfgType(v3P, robot2));

//   // Add edges to roadmap 2 with respective weights

//   // Edge 6: Valid path to goal for robot 2. Tests to ensure
//   // query method can find path after failing search attempts.
//   roadmap2->AddEdge(start2, goal2, 8.0);

//   // Edge 7 & 8: Passes through vertex outside of environment
//   // boundary. Tests to ensure vertices are invalidated properly
//   // for robot 2.
//   roadmap2->AddEdge(start2, v3, 1.0);
//   roadmap2->AddEdge(v3, goal2, 1.0);

//   // Setup group solution for making the Group Roadmap
//   auto solutionType = this->GetMPLibrary()->GetMPSolution();
//   auto solution = 
//   solution->AddRobot(robot1);
//   solution->SetRoadmap(robot1, roadmap1);

//   solution->AddRobot(robot2);
//   solution->SetRoadmap(robot2, roadmap2);

//   // Construct group roadmap
//   GroupRoadmap<GroupSolution<MPTraits>> groupRoadmap(robotGroup, solution);

//   /// Setup Group Task
//   // Setup robot 1 task
//   MPTask* task1(robot1);
//   // Set robot 2 start and goal constraints
//   std::unique_ptr<CSpaceConstraint> robot1StartConstraint =
//         std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot1, CfgType(start1P, robot1)));

//   std::unique_ptr<CSpaceConstraint> robot1GoalConstraint  =
//         std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot1, CfgType(goal1P, robot1)));

//   task1->SetStartConstraint(robot1StartConstraint);
//   task1->AddGoalConstraint(robot2GoalConstraint);

//   // Setup robot 2 task
//   MPTask* task2(robot2);
//   // Set robot 2 start and goal constraints
//   std::unique_ptr<CSpaceConstraint> robot2StartConstraint =
//         std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot2, CfgType(start2P, robot2)));

//   std::unique_ptr<CSpaceConstraint> robot2GoalConstraint  =
//         std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot2, CfgType(goal2P, robot2)));

//   task2->SetStartConstraint(robot2StartConstraint);
//   task2->AddGoalConstraint(robot2GoalConstraint);

//   // Construct group task from individual robot tasks
//   GroupTask* groupTask(robotGroup);
//   groupTask->AddTask(task1);
//   groupTask->AddTask(task2);

//   this->m_groupRoadmap = groupRoadmap;
//   this->m_groupTask    = groupTask;

//   // return std::make_pair(groupRoadmap, groupTask);
// }

#endif