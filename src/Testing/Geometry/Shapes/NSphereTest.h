#ifndef PPL_NSphere_TEST_H
#define PPL_NSphere_TEST_H

#include "Geometry/Shapes/NSphere.h"
#include "Testing/TestBaseObject.h"

#include <cmath>
#include <string>
#include <utility>
#include <cstddef>
#include <iostream>
#include <vector>
#include <functional>

class NSphereTest : public NSphere,
				public TestBaseObject {
  public: 
    ///@name LocalTypes
    ///@{

    typedef std::pair<bool,std::string> TestResult;

    ///@}
    ///@name Construction
    ///@{

    explicit NSphereTest();

    virtual ~NSphereTest();

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

    friend std::istream& operator>>(std::istream& _is, NSphere& _sphere);

    TestResult TestContains();

    TestResult TestClearance();

    TestResult TestClearancePoint();

    TestResult TestSample();

    /// Helper for constructing an instance of the NSphere
    /// @param dimension the dimension of the NSphere
    /// @param radius the raidus of the NSphere
    /// @return An instance of NSphere 
    NSphere* Construct(int dimension, int radius);

    /// Helper for printing out the test result messages
    void RunTestMessage(TestResult (*inputTestFunction)());

};


/*--------------------------- Construction ---------------------------*/


NSphereTest::
NSphereTest() : NSphere(1) {
}


NSphereTest::
~NSphereTest() noexcept = default;


/*--------------------------- Tests ---------------------------*/

typename NSphereTest::
TestResult
NSphereTest::RunTest() {
  std::cout << "TESTS FOR NSphere" << std::endl;

  // RunTestMessage(&NSphereTest::TestContains);
  // RunTestMessage(&NSphereTest::testClerance);
  // RunTestMessage(&NSphereTest::testClerancePoint);
  // RunTestMessage(&NSphereTest::testSample);
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


typename NSphereTest::
TestResult
NSphereTest::TestContains() {

  int dimension = 3;
  int radius = 1;
  NSphere* testSphere = Construct(dimension, radius);

  bool passed = true;
  std::string message = "";

  // this point should be contained
  std::vector<double> validP;
  for (int i = 0; i < dimension; i++) {
    validP.push_back(radius/2);
  }

  // this point should not be contained
  std::vector<double> invalidP;
  for (int i = 0; i < dimension; i++) {
    invalidP.push_back(2.0 * radius);
  }

  if (!testSphere->Contains(validP)) {
    passed = false;
    message = message + "\n\tNSphere does not contain valid point.\n";
  } 

  if (testSphere->Contains(invalidP)) {
    passed = false;
    message = message + "\n\tNSphere contains invalid point.\n";
  }

  if (passed) {
    message = "NSphere Contains: PASSED!\n";
  } else {
    message = "NSphere Contains: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 


typename NSphereTest::
TestResult
NSphereTest::TestClearance() {

  int dimension = 3;
  int radius = 1;
  NSphere* testSphere = Construct(dimension, radius);

  bool passed = true;
  std::string message = "";

  // the origin point, whose clearance
  // should be equal to the radius
  std::vector<double> testP;
  for (int i = 0; i < dimension; i++) {
    testP.push_back(0);
  }

  double expectedClearance = radius;
  double actualClearance = testSphere -> Clearance(testP);

  if (expectedClearance != actualClearance) {
    passed = false;
    message = message + "\n\tReturned incorrect clearance.\n";
  }

  if (passed) {
    message = "NSphere Clearance: PASSED!\n";
  } else {
    message = "NSphere Clearance: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 

typename NSphereTest::
TestResult
NSphereTest::TestClearancePoint() {

  int dimension = 3;
  int radius = 1;
  NSphere* testsphere = Construct(dimension, radius);

  bool passed = true;
  std::string message = "";

  // note: the clearance point of the origin point would be
  // (-nan, -nan, ...)

  // a point on the surface of the sphere
  // whose clearance point should be itself
  std::vector<double> testP;
  testP.push_back(radius);
  for (int i = 1; i < dimension; i++) {
    testP.push_back(0);
  }

  std::vector<double> expectedClearancePoint;
  expectedClearancePoint.push_back(radius);
  for (int i = 1; i < dimension; i++) {
    expectedClearancePoint.push_back(0);
  }

  vector<double> actualClearancePoint = testsphere -> ClearancePoint(testP);

  if (expectedClearancePoint != actualClearancePoint) {
    passed = false;
    message = "\n\tReturned clearance point is incorrect.\n";
  }

  if (passed) {
    message = "NSphere Clearance point: PASSED!\n";
  } else {
    message = "NSphere Clearance point: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 


typename NSphereTest::
TestResult
NSphereTest::TestSample() {

  int dimension = 3;
  int radius = 1;
  NSphere* testsphere = Construct(dimension, radius);

  bool passed = true;
  std::string message = "";

  // sample 100 random points and test if they lie within the sphere
  for (int i = 0; i < 100; i++) {
    vector<double> randomSamplePoint = testsphere -> Sample();
    if (!testsphere -> Contains(randomSamplePoint)) {
      passed = false;
      message = message + "\n\tSampled point not in the sphere\n";
    }
  }

  if (passed) {
    message = "NSphere Sample: PASSED!\n";
  } else {
    message = "NSphere Sample: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 


/*---------------------------- Helpers --------------------------*/

NSphere* 
NSphereTest::Construct(int dimension, int radius) {

  // define a valid n dimensional sphere centered at origin with a given radius
  NSphere* testSphere = new NSphere(dimension, radius);

  return testSphere;

}

void 
NSphereTest::RunTestMessage(TestResult (*inputTestFunction)()) {
  TestResult testResult = inputTestFunction();
  std::cout << "PASSED: "<< testResult.first << std::endl << testResult.second;
}



#endif
