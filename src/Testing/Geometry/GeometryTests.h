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
    vector<TestBaseObject*>* m_tests;
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
  int total = 0, passed = 0, failed = 0;
  TestResult res;
  m_tests = new vector<TestBaseObject*>;

  m_tests->push_back(new NBoxTest());
  m_tests->push_back(new NSphereTest());
  m_tests->push_back(new NSphericalShellTest());

  for (auto it = m_tests->begin(); it != m_tests->end(); ++it ) {
    auto test = dynamic_cast<TestBaseObject*>(*it);
    res = test->RunTest();

    total++;

    if(res.first) {
      passed++;
    }
    else {
      failed++;
    }
  }

  bool success = (failed == 0);
  std::string message = "COMPLETED Geometry TESTS"
                        "\nTotal: "  + std::to_string(total)  +
                        "\nPassed: " + std::to_string(passed) +
                        "\nFailed: " + std::to_string(failed) +
                        "\n\n\n";


  for (auto it = m_tests->begin(); it != m_tests->end(); ++it ) {
    delete *it;
  }

  delete m_tests;

  return std::make_pair(success,message);
}
/*--------------------------------------------------------------------*/

#endif
