#ifndef PPL_NSphericalShell_TEST_H
#define PPL_NSphericalShell_TEST_H

#include "Geometry/Shapes/NSphericalShell.h"
#include "Testing/TestBaseObject.h"

#include <cmath>
#include <string>
#include <utility>
#include <cstddef>
#include <iostream>
#include <vector>
#include <functional>

class NSphericalShellTest : public NSphericalShell,
				public TestBaseObject {
  public: 
    ///@name LocalTypes
    ///@{

    typedef std::pair<bool,std::string> TestResult;

    ///@}
    ///@name Construction
    ///@{

    explicit NSphericalShellTest();

    virtual ~NSphericalShellTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest();

    ///@}


  private:

    ///@name Internal State
    ///@{

    std::vector<double> m_center;        ///< The center point of the sphere.
    std::vector<Range<double>> m_range;  ///< The range in each dimension.

    ///@}

    friend std::istream& operator>>(std::istream& _is, NSphericalShell& _sphere);

    TestResult TestContains();

    TestResult TestClearance();

    TestResult TestClearancePoint();

    TestResult TestSample();

    /// Helper for constructing an instance of the NSphericalShell
    /// @param dimension the dimension of the NSphericalShell
    /// @param radius the raidus of the NSphericalShell
    /// @return An instance of NSphericalShell 
    NSphericalShell* Construct(int dimension, int innerRadius, int outerRadius);

    /// Helper for printing out the test result messages
    void RunTestMessage(TestResult (*inputTestFunction)());

};


/*--------------------------- Construction ---------------------------*/


NSphericalShellTest::
NSphericalShellTest() : NSphericalShell(1) {
}


NSphericalShellTest::
~NSphericalShellTest() noexcept = default;


/*--------------------------- Tests ---------------------------*/

typename NSphericalShellTest::
TestResult
NSphericalShellTest::RunTest() {
  std::cout << "TESTS FOR NSphericalShell" << std::endl;

  // RunTestMessage(&NSphericalShellTest::TestContains);
  // RunTestMessage(&NSphericalShellTest::testClerance);
  // RunTestMessage(&NSphericalShellTest::testClerancePoint);
  // RunTestMessage(&NSphericalShellTest::testSample);
  TestResult containsTestResult = TestContains();
  std::cout << "PASSED: "<< containsTestResult.first << std::endl << containsTestResult.second;

  TestResult clearanceTestResult = TestClearance();
  std::cout << "PASSED: "<< clearanceTestResult.first << std::endl << clearanceTestResult.second;

  TestResult clearancePointTestResult = TestClearancePoint();
  std::cout << "PASSED: "<< clearancePointTestResult.first << std::endl << clearancePointTestResult.second;

  TestResult sampleTestResult = TestSample();
  std::cout << "PASSED: "<< sampleTestResult.first << std::endl << sampleTestResult.second;

  return TestResult(true, "this is for testing infrastructure");
}


typename NSphericalShellTest::
TestResult
NSphericalShellTest::TestContains() {

  int dimension = 3;
  int innerRadius = 1;
  int outerRadius = 2;
  NSphericalShell* testSphericalShell = Construct(dimension, outerRadius, innerRadius);

  bool passed = true;
  std::string message = "";

  // this point should be contained
  std::vector<double> validP;
  validP.push_back((innerRadius + outerRadius)/2);
  for (int i = 1; i < dimension; i++) {
    validP.push_back(1);
  }

  // this point should not be contained
  std::vector<double> invalidP;
  for (int i = 0; i < dimension; i++) {
    invalidP.push_back(0);
  }

  if (!testSphericalShell->Contains(validP)) {
    passed = false;
    message = message + "\n\tNSphericalShell does not contain valid point.\n";
  } 

  if (testSphericalShell->Contains(invalidP)) {
    passed = false;
    message = message + "\n\tNSphericalShell contains invalid point.\n";
  }

  if (passed) {
    message = "NSphericalShell Contains: PASSED!\n";
  } else {
    message = "NSphericalShell Contains: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 


typename NSphericalShellTest::
TestResult
NSphericalShellTest::TestClearance() {

  int dimension = 3;
  int innerRadius = 1;
  int outerRadius = 2;
  NSphericalShell* testSphericalShell = Construct(dimension, outerRadius, innerRadius);

  bool passed = true;
  std::string message = "";

  // the origin point, whose clearance
  // should be equal to innerRadius
  std::vector<double> testP;
  for (int i = 0; i < dimension; i++) {
    testP.push_back(0);
  }

  double expectedClearance = -innerRadius;
  double actualClearance = testSphericalShell -> Clearance(testP);

  if (expectedClearance != actualClearance) {
    passed = false;
    message = message + "\n\tReturned incorrect clearance.\n";
  }

  if (passed) {
    message = "NSphericalShell Clearance: PASSED!\n";
  } else {
    message = "NSphericalShell Clearance: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 

typename NSphericalShellTest::
TestResult
NSphericalShellTest::TestClearancePoint() {

  int dimension = 3;
  int innerRadius = 1;
  int outerRadius = 2;
  NSphericalShell* testSphericalShell = Construct(dimension, outerRadius, innerRadius);

  bool passed = true;
  std::string message = "";

  // a point on the surface of the spherical shell
  // whose clearance point should be itself
  std::vector<double> testP;
  testP.push_back(innerRadius);
  for (int i = 1; i <= dimension; i++) {
    testP.push_back(0);
  }

  std::vector<double> expectedClearancePoint;
  expectedClearancePoint.push_back(innerRadius);
  for (int i = 1; i < dimension; i++) {
    expectedClearancePoint.push_back(0);
  }

  vector<double> actualClearancePoint = testSphericalShell -> ClearancePoint(testP);

  if (expectedClearancePoint != actualClearancePoint) {
    passed = false;
    message = "\n\tReturned clearance point is incorrect.\n";
  }

  if (passed) {
    message = "NSphericalShell Clearance point: PASSED!\n";
  } else {
    message = "NSphericalShell Clearance point: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 


typename NSphericalShellTest::
TestResult
NSphericalShellTest::TestSample() {

  int dimension = 3;
  int innerRadius = 1;
  int outerRadius = 2;
  NSphericalShell* testSphericalShell = Construct(dimension, outerRadius, innerRadius);

  bool passed = true;
  std::string message = "";

  // sample 100 random points and test if they lie within the spherical shell
  for (int i = 0; i < 100; i++) {
    vector<double> randomSamplePoint = testSphericalShell -> Sample();
    if (!testSphericalShell -> Contains(randomSamplePoint)) {
      passed = false;
      message = message + "\n\tSampled point not in the spherical shell\n";
      break;
    }
  }

  if (passed) {
    message = "NSphericalShell Sample: PASSED!\n";
  } else {
    message = "NSphericalShell Sample: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 


/*---------------------------- Helpers --------------------------*/

NSphericalShell* 
NSphericalShellTest::Construct(int dimension, int outerRadius, int innererRadius) {

  // define a valid n dimensional sphere centered at origin with a given radius
  NSphericalShell* testSphere = new NSphericalShell(dimension, outerRadius, innererRadius);

  return testSphere;

}

void 
NSphericalShellTest::RunTestMessage(TestResult (*inputTestFunction)()) {
  TestResult testResult = inputTestFunction();
  std::cout << "PASSED: "<< testResult.first << std::endl << testResult.second;
}



#endif
