#ifndef PPL_MIN_DIST_EVLUATOR_TEST_H_
#define PPL_MIN_DIST_EVLUATOR_TEST_H_

#include "MPLibrary/MPSolution.h"
#include "MPLibrary/MapEvaluators/MinimumDistanceEvaluator.h"
#include "Testing/MPLibrary/MapEvaluators/MapEvaluatorMethodTest.h"
#include "Utilities/MPUtils.h"

template <typename MPTraits>
class MinimumDistanceEvaluatorTest : virtual public MinimumDistanceEvaluator<MPTraits>,
                                     public MapEvaluatorMethodTest<MPTraits> {
 public:
  ///@name Local Types
  ///@{

  typedef typename MPTraits::RoadmapType RoadmapType;
  typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::WeightType WeightType;

  typedef TestBaseObject::TestResult TestResult;

  ///@}
  ///@name Construction
  ///@{

  MinimumDistanceEvaluatorTest();

  MinimumDistanceEvaluatorTest(XMLNode& _node);

  ~MinimumDistanceEvaluatorTest();

  ///@}

 private:
  ///@name Interface Test Function
  ///@{

  virtual TestResult MainFunctionTest() override;

  ///@}

  ///@name Helper Functions
  ///@{

  /// Construct a roadmap to test that edges and nodes
  void SingleSetup();

  ///@}

  ///@name Helper Members
  ///@{

  /// Single helper members
  RoadmapType* m_singleRoadmap;
  MPTask* m_singleTask;

  ///@}
};
/*--------------------------- Construction ---------------------------*/
template <typename MPTraits>
MinimumDistanceEvaluatorTest<MPTraits>::
    MinimumDistanceEvaluatorTest() : MinimumDistanceEvaluator<MPTraits>() {}

template <typename MPTraits>
MinimumDistanceEvaluatorTest<MPTraits>::
    MinimumDistanceEvaluatorTest(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node),
                                                   MinimumDistanceEvaluator<MPTraits>(_node) {}

template <typename MPTraits>
MinimumDistanceEvaluatorTest<MPTraits>::
    ~MinimumDistanceEvaluatorTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename MinimumDistanceEvaluatorTest<MPTraits>::TestResult
MinimumDistanceEvaluatorTest<MPTraits>::
    MainFunctionTest() {
  bool passed = true;
  std::string message;
  
  // // Test Individual Robot Functionality
  SingleSetup();

  bool minDistAchieved = this->IndividualRobotMainFunction(m_singleRoadmap,
                                                           m_singleTask);

  if (minDistAchieved) {
    passed = true;
    message += "\n\tSatisfied the mininumum distance requirement.\n";
  } else {
    passed = false;
    message += "\n\tDid NOT satisfy the mininumum distance requirement.\n";
  }

  // Clean up
  auto task = this->GetMPLibrary()->GetTask();
  task->ClearGoalConstraints();

  if (passed) {
    message = "MainFunctionTest::PASSED!\n";
  } else {
    message = "MainFunctionTest::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

/*--------------------------Helper Functions -------------------------*/

template <typename MPTraits>
void MinimumDistanceEvaluatorTest<MPTraits>::
    SingleSetup() {
  // Get Robot
  auto robot = this->GetMPProblem()->GetRobots()[1].get();

  /// Roadmap Construction
  RoadmapType* roadmap = new RoadmapType(robot);

  // Construct vertices and add to roadmap

  // Start & Goal: Valid robot configurations.
  auto p1 = CfgType(robot);
  std::istringstream p1Stream("0 0 0 0 0 0 0");
  p1.Read(p1Stream);
  auto start = roadmap->AddVertex(p1);

  auto p2 = CfgType(robot);
  std::istringstream p2Stream("0 20 -10 0 .3 .7 .8");
  p2.Read(p2Stream);
  auto goal = roadmap->AddVertex(p2);

  // Other vertices in the roadmap
  auto v1P = CfgType(robot);
  std::istringstream v1Stream("0 20 0 0 0 0 0");
  v1P.Read(v1Stream);
  auto v1 = roadmap->AddVertex(v1P);

  auto v2P = CfgType(robot);
  std::istringstream v2Stream("0 20 10 0 0 0 0");
  v2P.Read(v2Stream);
  auto v2 = roadmap->AddVertex(v2P);

  auto v3P = CfgType(robot);
  std::istringstream v3Stream("0 -20 10 0 0 0 0");
  v3P.Read(v3Stream);
  auto v3 = roadmap->AddVertex(v3P);

  auto v4P = CfgType(robot);
  std::istringstream v4Stream("0 -20 0 0 0 0 0");
  v4P.Read(v4Stream);
  auto v4 = roadmap->AddVertex(v4P);

  auto v5P = CfgType(robot);
  std::istringstream v5Stream("0 -20 -10 0 0 0 0");
  v5P.Read(v5Stream);
  auto v5 = roadmap->AddVertex(v5P);

  // Add edges to roadmap with respective weights
  // The shortest path will be from start -> v1 -> goal

  roadmap->AddEdge(start, v1, WeightType("a", 2.0));
  roadmap->AddEdge(v1, goal, WeightType("b", 1.0));

  roadmap->AddEdge(start, v4, WeightType("c", 2.0));
  roadmap->AddEdge(v4, v5, WeightType("d", 1.0));
  roadmap->AddEdge(v5, goal, WeightType("e", 4.0));

  roadmap->AddEdge(v1, v2, WeightType("f", 1.0));
  roadmap->AddEdge(v2, v3, WeightType("g", 4.0));
  roadmap->AddEdge(v3, v4, WeightType("h", 1.0));

  /// Task Construction
  auto task = this->GetMPLibrary()->GetTask();
  task->SetRobot(robot);

  // Set robot start and goal constraints
  auto startConstraint = std::unique_ptr<CSpaceConstraint>(
      new CSpaceConstraint(robot, p1));

  auto goalConstraint = std::unique_ptr<CSpaceConstraint>(
      new CSpaceConstraint(robot, p2));

  task->SetStartConstraint(std::move(startConstraint));
  task->AddGoalConstraint(std::move(goalConstraint));

  m_singleRoadmap = roadmap;
  m_singleTask = task;
}

#endif
