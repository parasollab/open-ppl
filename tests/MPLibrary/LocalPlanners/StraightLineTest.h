#ifndef PPL_STRAIGHT_LINE_TEST_H_
#define PPL_STRAIGHT_LINE_TEST_H_

#include "LocalPlannerMethodTest.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"

template <typename MPTraits>
class StraightLineTest : virtual public StraightLine<MPTraits>,
                         public LocalPlannerMethodTest<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult      TestResult;

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    StraightLineTest();

    StraightLineTest(XMLNode& _node);

    ~StraightLineTest();

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    virtual TestResult IndividualRobotIsConnectedTest() override;

    virtual TestResult IndividualRobotBlindPathTest() override;

    virtual TestResult RobotGroupIsConnectedTest() override;

    virtual TestResult RobotGroupBlindPathTest() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
StraightLineTest<MPTraits>::
StraightLineTest() : StraightLine<MPTraits>() {}

template <typename MPTraits>
StraightLineTest<MPTraits>::
StraightLineTest(XMLNode& _node) : LocalPlannerMethod<MPTraits>(_node),
                                   StraightLine<MPTraits>(_node) {}

template <typename MPTraits>
StraightLineTest<MPTraits>::
~StraightLineTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename StraightLineTest<MPTraits>::TestResult
StraightLineTest<MPTraits>::
IndividualRobotIsConnectedTest() {
  bool passed = true;
  std::string message = "";

  this->SetLibraryRobot();

  // Test two configurations that are connectable
  const CfgType c1 = this->GetIndividualCfg();

  CfgType c = this->GetIndividualCfg();
  c[0] += 10.0;
  const CfgType c2 = CfgType(c);

  CfgType col;
  LPOutput<MPTraits> lpOutput;

  bool result = this->IndividualRobotIsConnected(c1, c2, col, &lpOutput);

  if (!result) {
    passed = false;
    message = message + "\n\tLocal planner was not able to find a path between "
              "two connectable configurations\n";
  }

  // Test two configurations that are not connectable because one is out of 
  // bounds
  c[0] = 0.0;
  c[2] = 100.0;
  const CfgType c3 = CfgType(c);

  result = this->IndividualRobotIsConnected(c1, c3, col, &lpOutput);

  if (result) {
    passed = false;
    message = message + "\n\tLocal planner incorrectly found a path between "
              "two unconnectable configurations\n";
  }

  if (passed) {
    message = "IndividualRobotIsConnected::PASSED!\n";
  } else {
    message = "IndividualRobotIsConnected::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename StraightLineTest<MPTraits>::TestResult
StraightLineTest<MPTraits>::
IndividualRobotBlindPathTest() {
  bool passed = true;
  std::string message = "";

  // Check that a non-empty path is returned
  auto path = this->IndividualRobotBlindPath();
  bool result = (path.size() > 0);

  // Check that the path is a straight line along both segments
  if (result) {
    auto cfg = path[0];
    auto newCfg = path[1];
    std::size_t idx = 1;

    // Check that the first path segment contains configurations that are
    // constantly increasing in the x direction (and no other direction)
    while (passed and idx+1 < path.size() and fabs(newCfg[0] - 10.0) > 1e-7) {
      if (cfg[0] >= newCfg[0] or fabs(newCfg[1]) > 1e-7 
          or fabs(newCfg[2]) > 1e-7) {
        passed = false;
        break;
      }

      idx++;
      cfg = newCfg;
      newCfg = path[idx];
    }

    // Skip the endpoint of the first line segment
    idx++;
    cfg = newCfg;
    newCfg = path[idx];

    // Check that the second path segment contains configurations that are 
    // constantly increasing in the y direction (and no other direction)
    while (passed and idx+1 < path.size() and fabs(newCfg[1] - 10.0) > 1e-7) {
      if (cfg[1] >= newCfg[1] or fabs(newCfg[0] - 10.0) > 1e-7 
          or fabs(newCfg[2]) > 1e-7) {
        passed = false;
        break;
      }

      idx++;
      cfg = newCfg;
      newCfg = path[idx];
    }
  }

  if (!passed) 
    message = message + "\n\tLocal planner failed to find the correct blind path.\n";
  

  if (passed) {
    message = "IndividualRobotBlindPath::PASSED!\n";
  } else {
    message = "IndividualRobotBlindPath::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename StraightLineTest<MPTraits>::TestResult
StraightLineTest<MPTraits>::
RobotGroupIsConnectedTest() {
  bool passed = true;
  std::string message = "";

  this->SetLibraryGroup();

  //Test a scenario in which the two robots are in collision: this should result in "fail" to be considered as pass
  //two configurations (start and goal) for each robot are defined that are connectable to their goal
  //robot 1 starts at (-1,0,0 ) and move 2 steps in x direction
  //robot 2 starts at (0,-1,0) and move 2 steps in y direction

  GroupCfgType start = this->GetGroupCfg();

  auto& startCfg0 = start.GetRobotCfg(0);

  //set the initial position of robot 1 to (-1,0,0) and update the start groupconfig

  startCfg0[0] = -1;
  startCfg0[1] = 0;
  startCfg0[2] = 0;

  start.SetRobotCfg(0,move(startCfg0));

   //set the initial position of robot 2 to (0,-1,0) and update the start groupconfig

  auto& startCfg1 = start.GetRobotCfg(1);
  startCfg1[0] = 0;
  startCfg1[1] = -1;
  startCfg1[2] = 0;

  start.SetRobotCfg(1,move(startCfg1));

 //how to put startconfig 1 and 2 together to make a groupconfig for start?
  
  //robot 1 moves for 2 steps in x direction
  

  auto end = start;
  auto endCfg0 = startCfg0;
  endCfg0[0] += 2;

  end.SetRobotCfg(0,move(endCfg0));

  //robot 2 moves for 2 steps in y direction
  auto endCfg1 = startCfg1;
  endCfg1[1] +=2;

  end.SetRobotCfg(1,move(endCfg1));


  GroupCfgType col;
  GroupLPOutput<MPTraits> GroupLPOutput;

  bool result = this->RobotGroupIsConnected(start, end, col, &GroupLPOutput);

  if (result) {
  passed = false;
  message = message + "\n\tTwo robots are in collision. Local planner incorrectly found a path between "
              "two connectable configurations \n";
  }

  if (passed) {
    message = "RobotGroupIsConnected::PASSED!\n";
  } else {
    message = "RobotGroupIsConnected::FAILED :(\n" + message;
  }

  
  //Test a scenario in which the two robots are NOT in collision: this should result in "pass" to be considered as pass
  //two configurations (start and goal) for each robot are defined that are connectable to their goal
  //robot 1 starts at (-1,0,0 ) and move 2 steps in x direction
  //robot 2 starts at (2,-1,0) and move 2 steps in x direction

 GroupCfgType newstart = this->GetGroupCfg();

  auto& newstartCfg0 = newstart.GetRobotCfg(0);

  //set the initial position of robot 1 to (-1,0,0) and update the start groupconfig

  newstartCfg0[0] = -1;
  newstartCfg0[1] = 0;
  newstartCfg0[2] = 0;

  newstart.SetRobotCfg(0,move(newstartCfg0));

   //set the initial position of robot 2 to (2,-1,0) and update the start groupconfig

  auto& newstartCfg1 = newstart.GetRobotCfg(1);
  newstartCfg1[0] = 2;
  newstartCfg1[1] = -1;
  newstartCfg1[2] = 0;

  newstart.SetRobotCfg(1,move(newstartCfg1));

 //how to put startconfig 1 and 2 together to make a groupconfig for start?
  
  //robot 1 moves for 2 steps in x direction
  

  auto newend = newstart;

  auto newendCfg0 = newstartCfg0;
  newendCfg0[0] += 2;

  //robot 2 moves for 2 steps in x direction
  auto newendCfg1 = newstartCfg1;
  newendCfg1[0] += 2;


  result = this->RobotGroupIsConnected(newstart, newend, col, &GroupLPOutput);

  if (!result) {
  passed = false;
  message = message + "\n\tLocal planner was not able to find a path between "
              "two connectable configurations for two robots\n";
  }

  if (passed) {
    message = "RobotGroupIsConnected::PASSED!\n";
  } else {
    message = "RobotGroupIsConnected::FAILED :(\n" + message;
  }


  return std::make_pair(passed, message);
}

template<typename MPTraits>
typename StraightLineTest<MPTraits>::TestResult
StraightLineTest<MPTraits>::
RobotGroupBlindPathTest() {
  bool passed = true;
  std::string message = "";

  // Check that a non-empty path is returned
  auto path = this->RobotGroupBlindPath();
  bool result = (path.size() > 0);

  // Check that the path is a straight line along both segments
  if (result) {
    auto GroupCfg = path[0];
    auto newGroupCfg = path[1];
    std::size_t idx = 1;

    // Check that the first path segment contains configurations that are
    // constantly increasing in the x direction (and no other direction)
    while (passed and idx+1 < path.size() and fabs(newGroupCfg[0] - 10.0) > 1e-7) {
      if (GroupCfg[0] >= newGroupCfg[0] or fabs(newGroupCfg[1]) > 1e-7 
          or fabs(newGroupCfg[2]) > 1e-7) {
        passed = false;
        break;
      }

      idx++;
      GroupCfg = newGroupCfg;
      newGroupCfg = path[idx];
    }

    // Skip the endpoint of the first line segment
    idx++;
    GroupCfg = newGroupCfg;
    newGroupCfg = path[idx];

    // Check that the second path segment contains configurations that are 
    // constantly increasing in the y direction (and no other direction)
    while (passed and idx+1 < path.size() and fabs(newGroupCfg[1] - 10.0) > 1e-7) {
      if (GroupCfg[1] >= newGroupCfg[1] or fabs(newGroupCfg[0] - 10.0) > 1e-7 
          or fabs(newGroupCfg[2]) > 1e-7) {
        passed = false;
        break;
      }

      idx++;
      GroupCfg = newGroupCfg;
      newGroupCfg = path[idx];
    }
  }

if (!passed) 
    message = message + "\n\tLocal planner failed to find the correct blind path.\n";
  

  if (passed) {
    message = "RobotGroupIsConnected::PASSED!\n";
  } else {
    message = "RobotGroupIsConnected::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

/*--------------------------------------------------------------------*/
#endif
