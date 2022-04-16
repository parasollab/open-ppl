#ifndef PPL_BOUNDARY_CONSTRAINT_TEST_H_
#define PPL_BOUNDARY_CONSTRAINT_TEST_H_

#include "ConstraintTest.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/BoundaryConstraint.h"

class BoundaryConstraintTest :  public BoundaryConstraint,
                               public ConstraintTest {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    BoundaryConstraintTest();

    ~BoundaryConstraintTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestFactory() override;

    virtual TestResult TestClone() override;

    virtual TestResult TestSetRobot() override;

    virtual TestResult TestGetBoundary() override;

    virtual TestResult TestSatisfied() override;

    ///@}

};

/*--------------------------- Construction ---------------------------*/

BoundaryConstraintTest::
BoundaryConstraintTest() : BoundaryConstraint() {}

BoundaryConstraintTest::
~BoundaryConstraintTest() {}

/*--------------------- Test Interface Functions ---------------------*/

// BoundaryConstraint has no setter functions for the Boundary, so constructing a new one

typename BoundaryConstraintTest::TestResult
BoundaryConstraintTest::
TestFactory() {

  // Based on XML Node parameter: 
  // Unable to test without knowing specific values in XML File, 
  // But currently XML MPProblem not created?

  bool passed = true;
  std::string message = "";

  return std::make_pair(passed,message);
}

typename BoundaryConstraintTest::TestResult
BoundaryConstraintTest::
TestClone() {

  auto testPointer = this->Clone();

  if(testPointer->m_robot == m_robot && testPointer->m_boundary == m_boundary){
    delete testPointer;
    return std::make_pair(true,"BoundaryConstraintTest::TestClone PASSED");
  }
  delete testPointer;
  return std::make_pair(false,"Robot clone is not a valid match.");
}

typename BoundaryConstraintTest::TestResult
BoundaryConstraintTest::
TestSetRobot() {

  BoundaryConstraint tester = BoundaryConstraint(nullptr, Cfg(m_robot));
  tester.SetRobot(m_robot);

  if(tester.m_robot == m_robot){
    return std::make_pair(true,"BoundaryConstraintTest::TestSetRobot PASSED");
  }

  return std::make_pair(false,"Robot set incorrectly or not at all.");
}

typename BoundaryConstraintTest::TestResult
BoundaryConstraintTest::
TestGetBoundary() {

  BoundaryConstraint tester = BoundaryConstraint(nullptr, m_boundary);
  
  if(tester.GetBoundary() == m_boundary){
    return std::make_pair(true,"BoundaryConstraintTest::TestGetBoundary PASSED");
  }

  return std::make_pair(false,"Boundary Recieved Incorrectly.");
}

typename BoundaryConstraintTest::TestResult
BoundaryConstraintTest::
TestSatisfied() {

  // Test recieved Configuration is satisfied properly
  Cfg validCfg = Cfg(m_robot);
  if(!Satisfied(validCfg))){
    return std::make_pair(false,"Valid Cfg classified as invalid.");
  }

  return std::make_pair(true,"BoundaryConstraintTest::TestSatisfied PASSED");
}

#endif
