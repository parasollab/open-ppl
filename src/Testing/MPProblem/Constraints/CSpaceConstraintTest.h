#ifndef PPL_CSPACE_CONSTRAINT_TEST_H_
#define PPL_CSPACE_CONSTRAINT_TEST_H_

#include "ConstraintTest.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CSpaceConstraint.h"

class CSpaceConstraintTest :  public CSpaceConstraint,
                               public ConstraintTest {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    CSpaceConstraintTest();

    ~CSpaceConstraintTest();

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

CSpaceConstraintTest::
CSpaceConstraintTest() : CSpaceConstraint() {}

CSpaceConstraintTest::
CSpaceConstraintTest(MPProblem* _problem) : ConstraintTest(),
                                                      CSpaceConstraint(){
  m_MPProblem = _problem;
}

CSpaceConstraintTest::
~CSpaceConstraintTest() {}



/*--------------------- Test Interface Functions ---------------------*/

typename CSpaceConstraintTest::TestResult
CSpaceConstraintTest::
TestFactory() {

  // Based on XML Node parameter: 
  // Unable to test without knowing specific values in XML File, 
  // But currently XML MPProblem not created?

  bool passed = true;
  std::string message = "";

  return std::make_pair(passed,message);
}

typename CSpaceConstraintTest::TestResult
CSpaceConstraintTest::
TestClone() {

  auto testPointer = Clone();

  if(testPointer->m_robot == m_robot && testPointer->m_boundary == m_boundary){
    delete testPointer;
    return std::make_pair(true,"CSpaceConstraintTest::TestClone PASSED");
  }
  delete testPointer;
  return std::make_pair(false,"Robot clone is not a valid match.");
}

typename CSpaceConstraintTest::TestResult
CSpaceConstraintTest::
TestSetRobot() {

  CSpaceConstraintTest tester = CSpaceConstraintTest(nullptr, Cfg(m_robot));
  tester.SetRobot(m_robot);

  if(tester.m_robot == m_robot){
    return std::make_pair(true,"CSpaceConstraintTest::TestSetRobot PASSED");
  }

  return std::make_pair(false,"Robot set incorrectly or not at all.");

}

typename CSpaceConstraintTest::TestResult
CSpaceConstraintTest::
TestGetBoundary() {

  CSpaceConstraintTest tester = CSpaceConstraintTest(nullptr, m_boundary);
  
  if(tester.GetBoundary() == m_boundary){
    return std::make_pair(true,"CSpaceConstraintTest::TestGetBoundary PASSED");
  }

  return std::make_pair(false,"Boundary Recieved Incorrectly.");
}

typename CSpaceConstraintTest::TestResult
CSpaceConstraintTest::
TestSatisfied() {

  // Test recieved Configuration is satisfied properly
  Cfg validCfg = Cfg(m_robot);
  if(!Satisfied(validCfg))){
    return std::make_pair(false,"Valid Cfg classified as invalid.");
  }

  return std::make_pair(true,"CSpaceConstraintTest::TestSatisfied PASSED");
}

#endif
