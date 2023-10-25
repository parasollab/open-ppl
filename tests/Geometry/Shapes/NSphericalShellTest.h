#ifndef PPL_NSphericalShell_TEST_H
#define PPL_NSphericalShell_TEST_H

#include "Geometry/Shapes/NSphericalShell.h"    //src

#include <cmath>
#include <string>
#include <utility>
#include <cstddef>
#include <iostream>
#include <vector>
#include <functional>

class NSphericalShellTest : public NSphericalShell {
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


  public:

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


NSphericalShellTest::NSphericalShellTest() : NSphericalShell(1) {
}
NSphericalShellTest::
~NSphericalShellTest() noexcept = default;


/*--------------------------- Tests ---------------------------*/

typename NSphericalShellTest::TestResult NSphericalShellTest::TestContains() {

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
    std::cout << "\n\tNSphericalShell does not contain valid point." << std::endl;
  } 

  if (testSphericalShell->Contains(invalidP)) {
    passed = false;
    std::cout << "\n\tNSphericalShell contains invalid point." << std::endl;
  }

  message = "\tFINISHED TestContains";
  return std::make_pair(passed, message);

} 


typename NSphericalShellTest::TestResult NSphericalShellTest::TestClearance() {

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
    std::cout << "\n\tReturned incorrect clearance." << std::endl;
  }

  message = "\tFINISHED TestClearance";
  return std::make_pair(passed, message);

} 

typename NSphericalShellTest::TestResult NSphericalShellTest::TestClearancePoint() {

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

  std::vector<double> actualClearancePoint = testSphericalShell -> ClearancePoint(testP);

  if (expectedClearancePoint != actualClearancePoint) {
    passed = false;
    std::cout << "\n\tReturned clearance point is incorrect." << std::endl;
  }

  message = "\tFINISHED TestClearancePoint";
  return std::make_pair(passed, message);

} 


typename NSphericalShellTest::TestResult NSphericalShellTest::TestSample() {

  int dimension = 3;
  int innerRadius = 1;
  int outerRadius = 2;
  NSphericalShell* testSphericalShell = Construct(dimension, outerRadius, innerRadius);

  bool passed = true;
  std::string message = "";

  // sample 100 random points and test if they lie within the spherical shell
  for (int i = 0; i < 100; i++) {
    std::vector<double> randomSamplePoint = testSphericalShell -> Sample();
    if (!testSphericalShell -> Contains(randomSamplePoint)) {
      passed = false;
      std::cout << "\n\tSampled point not in the spherical shell." << std::endl;
      break;
    }
  }

  message = "\tFINISHED TestSample";
  return std::make_pair(passed, message);

} 


/*---------------------------- Helpers --------------------------*/

NSphericalShell* NSphericalShellTest::Construct(int dimension, int outerRadius, int innererRadius) {

  // define a valid n dimensional sphere centered at origin with a given radius
  NSphericalShell* testSphere = new NSphericalShell(dimension, outerRadius, innererRadius);

  return testSphere;

}

void NSphericalShellTest::RunTestMessage(TestResult (*inputTestFunction)()) {
  TestResult testResult = inputTestFunction();
  std::cout << "PASSED: "<< testResult.first << std::endl << testResult.second;
}

#endif
