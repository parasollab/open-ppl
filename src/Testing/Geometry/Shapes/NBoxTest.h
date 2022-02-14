#ifndef PPL_NBOX_H
#define PPL_NBOX_H

#include "Geometry/Shapes/NBox.h"
#include "Testing/TestBaseObject.h"
#include "Geometry/Boundaries/Range.h"

/*-----------------------------------------------------------*/
///
/// This is a base class to standardize the testing interface.
/// All child classes should build their tests from the 
/// RunTests function. 
///
/*-----------------------------------------------------------*/

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

    explicit NBoxTest(const size_t _n);

    explicit NBoxTest(const std::vector<double>& _center);

    virtual ~NBoxTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() = 0;

    ///@}


  private:

    ///@name Internal State
    ///@{

    std::vector<double> m_center;        ///< The center point of the box.
    std::vector<Range<double>> m_range;  ///< The range in each dimension.

    ///@}

    friend std::istream& operator>>(std::istream& _is, NBox& _box);




    // helpers for test
    int dimension;

    NBox testBox;

};


/*--------------------------- Construction ---------------------------*/

NBoxTest::
NBoxTest(const size_t _n): NBox(_n) {
}

NBoxTest::
NBoxTest(XMLNode& _node, const std::vector<double>& _center): NBox(_node, _center) {
}

NBoxTest::
~NBoxTest() noexcept = default;


/*--------------------------- Tests ---------------------------*/



// valid case

// edge cases



void NBoxTest::testContains() {

  construct();

  // this point should be contained
  std::vector<double> validP;
  for (int i = 1; i <= dimension; i++) {
    validP.push_back(i - 0.1);
  }

  // this point should not be contained
  std::vector<double> invalidP;
  for (int i = 1; i <= dimension; i++) {
    validP.push_back(2.0 * i);
  }

  if (!testBox.Contains(validP) || testBox.Contains(invalidP)) {
    passed = false;
  } 

  passed = true;

} 


void NBoxTest::testClearance() {

  construct();

  // the origin point, whose clearance
  // should be equal to 1
  std::vector<double> testP;
  for (int i = 1; i <= dimension; i++) {
    testP.push_back(0);
  }

  double expectedClearance = 1;
  double actualClearance = testBox.Clearance(testP);

  if (expectedClearance != actualClearance) {
    passed = false;
  }

  passed = true;


} 

void NBoxTest::testClearancePoint() {

  construct();

  // the origin point, whose clearance
  // point should be equal to (-1, 0, 0, ...)
  std::vector<double> testP;
  for (int i = 1; i <= dimension; i++) {
    testP.push_back(0);
  }

  std::vector<double> expectedClearancePoint;
  for (int i = 1; i <= dimension; i++) {
    expectedClearancePoint.push_back(0);
  }

  double actualClearancePoint = testBox.Clearance(testP);

  if (expectedClearance != actualClearance) {
    passed = false;
  }

  passed = true;



} 

void NBoxTest::testSamples() {

} 


/*---------------------------- Helpers --------------------------*/

// construct an instance of the NBox()
// should be run in every test case 
void NBoxTest::construct {

    // define a valid n dimensional box centered at origin
    // with range equal to 2i where i is the dimension index
    testBox = new NBox(dimension);
    for (int i = 0; i < dimension; i++) {
      testBox.setRange(i, -i, i);
    }

}




#endif
