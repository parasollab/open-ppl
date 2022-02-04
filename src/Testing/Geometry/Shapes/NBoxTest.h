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

};


/*--------------------------- Construction ---------------------------*/

NBoxTest::
NBoxTest(const size_t _n): NBox() {
}

NBoxTest::
NBoxTest(const std::vector<double>& _center): NBox() {
}

// NBoxTest::
// NBoxTest() {
// }


NBoxTest::
~NBoxTest() noexcept = default;

#endif
