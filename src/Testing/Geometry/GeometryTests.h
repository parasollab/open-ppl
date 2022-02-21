#ifndef PPL_GEOMETRY_TESTS_H_
#define PPL_GEOMETRY_TESTS_H_

#include "Testing/TestBaseObject.h"

#include "Testing/Geometry/Shapes/NBoxTest.h"

class GeometryTests : public TestBaseObject {
  public:
    ///@name LocalTypes
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    GeometryTests();

    GeometryTests(const std::string& _xmlFile);

    virtual ~GeometryTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}
  
  private:

    NBoxTest* m_nBoxTest{nullptr};
};

/*--------------------------- Construction ---------------------------*/

MPProblemTests::
MPProblemTests() {

    m_nBoxTest = NBoxTest();
}

MPProblemTests::
MPProblemTests(const std::string& _xmlFile) {

    m_nBoxTest = NBoxTest(_xmlFile);

}

MPProblemTests::
~MPProblemTests() {}
/*----------------------------- Interface ----------------------------*/

typename MPProblemTests::TestResult
MPProblemTests::
RunTest() {
  return TestResult();
}

/*--------------------------------------------------------------------*/

#endif
