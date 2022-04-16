#ifndef PPL_CONSTRAINT_TEST_H_
#define PPL_CONSTRAINT_TEST_H_

#include "MPProblem/Constraints/Constraint.h"
#include "Testing/TestBaseObject.h"

class ConstraintTest : virtual public Constraint, 
                          public TestBaseObject<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    ConstraintTest();

    ~ConstraintTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    virtual TestResult TestFactory() = 0;

    virtual TestResult TestClone() = 0;

    virtual TestResult TestSetRobot() = 0;

    virtual TestResult TestGetBoundary() = 0;

    virtual TestResult TestSatisfied() = 0;

};

/*--------------------------- Construction ---------------------------*/

ConstraintTest::
ConstraintTest() : Constraint() {}

ConstraintTest::
~ConstraintTest() {}

/*----------------------------- Interface ----------------------------*/

typename ConstraintTest::TestResult
ConstraintTest::
RunTest() {

  bool passed = true;
  std::string message = "";

  auto result = TestFactory();
  passed = passed and result.first;
  message = message + result.second;

  result = TestClone();
  passed = passed and result.first;
  message = message + result.second;

  result = TestSetRobot();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGetBoundary();
  passed = passed and result.first;
  message = message + result.second;

  result = TestSatisfied();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

#endif
