#ifndef PPL_VALIDITY_CHECKER_METHOD_TEST_H_
#define PPL_VALIDITY_CHECKER_METHOD_TEST_H_

#include "MPLibrary/ValidityCheckers/ValidityCheckerMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class ValidityCheckerMethodTest : virtual public ValidityCheckerMethod<MPTraits>,
                                  public TestBaseObject {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult      TestResult;

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    ValidityCheckerMethodTest();

    ~ValidityCheckerMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    virtual TestResult IndividualCfgValidityTest() = 0;

    virtual TestResult GroupCfgValidityTest() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    virtual std::vector<std::pair<bool,CfgType>> IndividualCfgValidity();

    virtual std::vector<std::pair<bool,CfgType>> GroupCfgValidityBounding();

    virtual std::vector<std::pair<bool,GroupCfgType>> GroupCfgValidity();

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
ValidityCheckerMethodTest<MPTraits>::
ValidityCheckerMethodTest() : ValidityCheckerMethod<MPTraits>() {}

template <typename MPTraits>
ValidityCheckerMethodTest<MPTraits>::
~ValidityCheckerMethodTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename ValidityCheckerMethodTest<MPTraits>::TestResult
ValidityCheckerMethodTest<MPTraits>::
RunTest() {
  bool passed = true;
  std::string message = "";

  auto result = IndividualCfgValidityTest();
  passed = passed and result.first;
  message = message += result.second;

  result = GroupCfgValidityTest();
  passed = passed and result.first;
  message = message += result.second;

  return std::make_pair(passed,message);
}

/*----------------------- Default Function Calls ---------------------*/

template <typename MPTraits>
std::vector<std::pair<bool,typename MPTraits::CfgType>>
ValidityCheckerMethodTest<MPTraits>::
IndividualCfgValidity() {

  // Set the library for a single robot.
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  auto task = this->GetMPProblem()->GetTasks(robot)[0];
  this->GetMPLibrary()->SetTask(task.get());
  this->GetMPLibrary()->SetGroupTask(nullptr);

  std::vector<std::pair<bool,CfgType>> output;

  // Place a robot in the center of the envionment (assume it's open).
  CfgType cfg(robot);
  bool valid = this->IsValid(cfg,"Test");
  output.push_back(std::make_pair(valid,cfg));

  // If the robot has positional DOF, place it outside the environment
  // boundary.
  if(robot->GetMultiBody()->PosDOF() > 0) {
    auto x = this->GetEnvironment()->GetBoundary()->GetRange(0).min;
    cfg[0] = x - 1;
    valid = this->IsValid(cfg,"Test");
    output.push_back(std::make_pair(valid,cfg));
  }

  return output;
}

template <typename MPTraits>
std::vector<std::pair<bool,typename MPTraits::CfgType>> // TODO: need to update this to groupcfgtype (seebelowline 163) and above in declaration. same for inside test.
ValidityCheckerMethodTest<MPTraits>::
GroupCfgValidityBounding() {

  // Set the library for a single robot.
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  auto task = this->GetMPProblem()->GetTasks(robot)[0];
  this->GetMPLibrary()->SetTask(task.get());
  this->GetMPLibrary()->SetGroupTask(nullptr);

  std::vector<std::pair<bool,CfgType>> output;
/*
  // Place first robot at position [15, -5, 0, 0, 0, 0] (assume it's open).
  CfgType cfg1(robot);

  cfg1[0] = 15;
  cfg1[1] = -5;
  cfg1[2] = 0;
  cfg1[3] = 0;
  cfg1[4] = 0;
  cfg1[5] = 0;

  bool valid = this->IsValid(cfg1,"Test"); //should be true
  output.push_back(std::make_pair(valid,cfg1));

  //Place second robot at position [15, -3,5, 0, 0, 0, 0].
  CfgType cfg2(robot);

  cfg2[0] = 15;
  cfg2[1] = -3.5;
  cfg2[2] = 0;
  cfg2[3] = 0;
  cfg2[4] = 0;
  cfg2[5] = 0;

  bool valid = this->IsValid(cfg2,"Test"); //should be false
  output.push_back(std::make_pair(valid,cfg2));
  */

  return output;
}

template <typename MPTraits>
std::vector<std::pair<bool,typename MPTraits::GroupCfgType>>
ValidityCheckerMethodTest<MPTraits>::
GroupCfgValidity() {

  // Set the library for a robot group.
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  auto task = this->GetMPProblem()->GetTasks(robot)[0];
  this->GetMPLibrary()->SetTask(task.get());
  this->GetMPLibrary()->SetGroupTask(nullptr);

  // TODO::Implement this test. Make sure the negative example has two
  // robots colliding with each other and the positive has no inter-
  // robot collision.
  return {};
}

/*--------------------------------------------------------------------*/

#endif
