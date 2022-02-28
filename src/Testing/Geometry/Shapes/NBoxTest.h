#ifndef PPL_NBOX_TEST_H
#define PPL_NBOX_TEST_H

#include "Geometry/Shapes/NBox.h"
#include "Testing/TestBaseObject.h"

#include <cmath>
#include <string>
#include <utility>
#include <cstddef>
#include <iostream>
#include <vector>

class NBoxTest : public NBox,
				public TestBaseObject {
  public: 
    ///@name LocalTypes
    ///@{

    typedef std::pair<bool,std::string> TestResult;

    ///@}
    ///@name Construction
    ///@{

    explicit NBoxTest();

    virtual ~NBoxTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest();

    ///@}


  private:

    ///@name Internal State
    ///@{

    std::vector<double> m_center;        ///< The center point of the box.
    std::vector<Range<double>> m_range;  ///< The range in each dimension.

    ///@}

    friend std::istream& operator>>(std::istream& _is, NBox& _box);

    TestResult testContains();

    TestResult testClearance();

    TestResult testClearancePoint();

    TestResult testSample();

    /// Helper for constructing an instance of the NBox
    /// @return An instance of NBox 
    NBox* construct();

    /// Helper for printing out the test result messages
    void RunTestMessage(TestResult());

    int m_dimension;  /// the dimension for the test box

};


/*--------------------------- Construction ---------------------------*/


NBoxTest::
NBoxTest() : NBox(1) {
}


NBoxTest::
~NBoxTest() noexcept = default;


/*--------------------------- Tests ---------------------------*/

typename NBoxTest::
TestResult
NBoxTest::RunTest() {
  std::cout << "TESTS FOR NBOX" << std::endl;

  // RunTestMessage(testContains);
  // RunTestMessage(testClerance);
  // RunTestMessage(testClerancePoint);
  // RunTestMessage(testSample);
  TestResult containsTestResult = testContains();
  std::cout << "PASSED: "<< containsTestResult.first << std::endl << containsTestResult.second;

  TestResult clearanceTestResult = testClearance();
  std::cout << "PASSED: "<< clearanceTestResult.first << std::endl << clearanceTestResult.second;

  TestResult clearancePointTestResult = testClearancePoint();
  std::cout << "PASSED: "<< clearancePointTestResult.first << std::endl << clearancePointTestResult.second;

  TestResult sampleTestResult = testSample();
  std::cout << "PASSED: "<< sampleTestResult.first << std::endl << sampleTestResult.second;

  return TestResult(true, "this is for testing infrastructure");
}


typename NBoxTest::
TestResult
NBoxTest::testContains() {

  NBox* testBox = construct();

  bool passed = true;
  std::string message = "";

  // this point should be contained
  std::vector<double> validP;
  for (int i = 0; i < m_dimension; i++) {
    validP.push_back((i + 1) / 2);
  }

  // this point should not be contained
  std::vector<double> invalidP;
  for (int i = 0; i < m_dimension; i++) {
    invalidP.push_back(2.0 * (i + 1));
  }

  if (!testBox->Contains(validP)) {
    passed = false;
    message = message + "\n\tNBox does not contain valid point.\n";
  } 

  if (testBox->Contains(invalidP)) {
    passed = false;
    message = message + "\n\tNBox contains invalid point.\n";
  }

  if (passed) {
    message = "NBox Contains: PASSED!\n";
  } else {
    message = "NBox Contains: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 


typename NBoxTest::
TestResult
NBoxTest::testClearance() {

  NBox* testBox = construct();

  bool passed = true;
  std::string message = "";

  // the origin point, whose clearance
  // should be equal to 1
  std::vector<double> testP;
  for (int i = 0; i < m_dimension; i++) {
    testP.push_back(0);
  }

  double expectedClearance = 1;
  double actualClearance = testBox -> Clearance(testP);

  if (expectedClearance != actualClearance) {
    passed = false;
    message = message + "\n\tReturned incorrect clearance.\n";
  }

  if (passed) {
    message = "NBox Clearance: PASSED!\n";
  } else {
    message = "NBox Clearance: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 

typename NBoxTest::
TestResult
NBoxTest::testClearancePoint() {

  NBox* testBox = construct();

  bool passed = true;
  std::string message = "";

  // the origin point, whose clearance
  // point should be equal to (-1, 0, 0, ...)
  std::vector<double> testP;
  for (int i = 1; i <= m_dimension; i++) {
    testP.push_back(0);
  }

  std::vector<double> expectedClearancePoint;
  expectedClearancePoint.push_back(1);
  for (int i = 1; i < m_dimension; i++) {
    expectedClearancePoint.push_back(0);
  }

  vector<double> actualClearancePoint = testBox -> ClearancePoint(testP);

  if (expectedClearancePoint != actualClearancePoint) {
    passed = false;
    message = "\n\tReturned clearance point is incorrect.\n";
  }

  if (passed) {
    message = "NBox Clearance point: PASSED!\n";
  } else {
    message = "NBox Clearance point: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 


typename NBoxTest::
TestResult
NBoxTest::testSample() {

  NBox* testBox = construct();

  bool passed = true;
  std::string message = "";

  // sample 100 random points and test if they lie within the box
  for (int i = 0; i < 100; i++) {
    vector<double> randomSamplePoint = testBox -> Sample();
    if (!testBox -> Contains(randomSamplePoint)) {
      passed = false;
      message = message + "\n\tSampled point not in the box\n";
    }
  }

  if (passed) {
    message = "NBox Sample: PASSED!\n";
  } else {
    message = "NBox Sample: Failed :(\n" + message;
  }

  return std::make_pair(passed, message);

} 


/*---------------------------- Helpers --------------------------*/

NBox* NBoxTest::construct() {
  // define a valid n dimensional box centered at origin with range (i - 1, i + 1)
  m_dimension = 3;
  NBox* testBox = new NBox(m_dimension);
  
  for (int i = 0; i < m_dimension; i++) {
    testBox->SetRange(i, -i - 1, i + 1);
  }

  return testBox;

}

void NBoxTest::RunTestMessage(TestResult inputTestFunction()) {
  TestResult testResult = inputTestFunction();
  std::cout << "PASSED: "<< testResult.first << std::endl << testResult.second;
}



#endif
