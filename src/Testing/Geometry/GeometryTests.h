#ifndef PPL_GEOMETRY_TESTS_H_
#define PPL_GEOMETRY_TESTS_H_

#include "Testing/TestBaseObject.h"
#include "Utilities/XMLNode.h"
#include "Testing/Geometry/Shapes/NBoxTest.h"
#include "Testing/Geometry/Shapes/NSphereTest.h"
#include "Testing/Geometry/Shapes/NSphericalShellTest.h"

class GeometryTests : public TestBaseObject {
  public:
    ///@name LocalTypes
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    GeometryTests();

    GeometryTests(XMLNode& _node);

    virtual ~GeometryTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}
  
  private:

    NBoxTest* m_nBoxTest{nullptr};
    NSphereTest* m_nSphereTest{nullptr};
    NSphericalShellTest* m_nSphericalShellTest{nullptr};
};

/*--------------------------- Construction ---------------------------*/

GeometryTests::
GeometryTests() {
}


GeometryTests::
~GeometryTests() {}
/*----------------------------- Interface ----------------------------*/

typename GeometryTests::TestResult
GeometryTests::
RunTest() {
  // m_nBoxTest = new NBoxTest();
  // return m_nBoxTest->RunTest();

  // m_nSphereTest = new NSphereTest();
  // return m_nSphereTest->RunTest();

  m_nSphericalShellTest = new NSphericalShellTest();
  return m_nSphericalShellTest->RunTest();

  delete m_nBoxTest;
  delete m_nSphereTest;
  delete m_nSphericalShellTest;
}

/*--------------------------------------------------------------------*/

#endif
