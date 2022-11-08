#ifndef PPL_STRAIGHT_LINE_TEST_H_
#define PPL_STRAIGHT_LINE_TEST_H_

#include "Testing/MPLibrary/LocalPlanners/LocalPlannerMethodTest.h"
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

  if (!passed) {
    message = message + "\n\tLocal planner failed to find the correct blind path.\n";
  }

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

  // TODO Need support for GroupCfgs

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

  // TODO Need support for GroupCfgs

  if (passed) {
    message = "RobotGroupIsConnected::PASSED!\n";
  } else {
    message = "RobotGroupIsConnected::FAILED :(\n" + message;
  }

  return std::make_pair(passed, message);
}

/*--------------------------------------------------------------------*/
#endif
