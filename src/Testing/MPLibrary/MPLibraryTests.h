#ifndef PPL_M_P_LIBRARY_TEST_H_
#define PPL_M_P_LIBRARY_TEST_H_

#include "MPLibrary/MPLibrary.h"
#include "Testing/TestBaseObject.h"

#include "Testing/MPLibrary/DistanceMetrics/DistanceMetricMethodTest.h"
#include "Testing/MPLibrary/ValidityCheckers/ValidityCheckerMethodTest.h"
//#include "Testing/MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethodTest.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"
#include "Testing/MPLibrary/LocalPlanners/LocalPlannerMethodTest.h"
#include "Testing/MPLibrary/Extenders/ExtenderMethodTest.h"
//#include "Testing/MPLibrary/PathModifiers/PathModifierMethodTest.h"
//#include "Testing/MPLibrary/Connectors/ConnectorMethodTest.h"
//#include "Testing/MPLibrary/Metrics/MetricMethodTest.h"
#include "Testing/MPLibrary/MapEvaluators/MapEvaluatorMethodTest.h"

template <typename MPTraits>
class MPLibraryTests : public MPLibraryType<MPTraits>, public TestBaseObject {
  public:
    ///@LocalTypes
    ///@{

    typedef MPLibraryType<MPTraits>    MPLibrary;
    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Method Set Types
    ///@{

    ///@}
    ///@name Construction
    ///@{

    MPLibraryTests();

    MPLibraryTests(const std::string& _xmlFile);

    virtual ~MPLibraryTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  private:
   
    ///@name Helper Functions
    ///@{

    template <typename MethodTypeList>
    void RunMethodSetTests(const MethodTypeList& _mtl,size_t& _passed, 
                           size_t& failed, size_t& _total);

    ///@} 
    ///@name Internal State
    ///@{

    bool verbose{true};

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
MPLibraryTests<MPTraits>::
MPLibraryTests() {}

template <typename MPTraits>
MPLibraryTests<MPTraits>::
MPLibraryTests(const std::string& _xmlFile) : MPLibraryType<MPTraits>(_xmlFile) {}

template <typename MPTraits>
MPLibraryTests<MPTraits>::
~MPLibraryTests() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename MPLibraryTests<MPTraits>::TestResult
MPLibraryTests<MPTraits>::
RunTest() {

  size_t passed = 0;
  size_t failed = 0;
  size_t total = 0;

  // Distance metric tests
  RunMethodSetTests(*this->m_distanceMetrics,passed,failed,total);

  // Validity checker tests
  RunMethodSetTests(*this->m_validityCheckers,passed,failed,total);

  // Neighborhood finder tests
  RunMethodSetTests(*this->m_neighborhoodFinders,passed,failed,total);

  // Sampler tests
  RunMethodSetTests(*this->m_samplers,passed,failed,total);

  // Local planner tests
  RunMethodSetTests(*this->m_localPlanners,passed,failed,total);

  // Extender tests
  RunMethodSetTests(*this->m_extenders,passed,failed,total);

  // Path modifier tests
  RunMethodSetTests(*this->m_pathModifiers,passed,failed,total);

  // Connector tests
  RunMethodSetTests(*this->m_connectors,passed,failed,total);

  // Metric tests
  RunMethodSetTests(*this->m_metrics,passed,failed,total);

  // Map evaluator tests
  RunMethodSetTests(*this->m_mapEvaluators,passed,failed,total);

  bool success = (failed == 0);
  std::string message = "COMPLETED TESTS" 
                        "\nTotal: "  + std::to_string(total)  +
                        "\nPassed: " + std::to_string(passed) +
                        "\nFailed: " + std::to_string(failed) +
                        "\n\n\n";

  return std::make_pair(success,message);
}

/*-------------------------- Helper Functions ------------------------*/

template <typename MPTraits>
template <typename MethodTypeList>
void
MPLibraryTests<MPTraits>::
RunMethodSetTests(const MethodTypeList& _mtl, size_t& _passed, 
                  size_t& _failed, size_t& _total) {

  for(auto iter = _mtl.begin(); iter != _mtl.end(); iter++) {

    std::cout << "Running test for " << iter->first << "...";

    auto test = dynamic_cast<TestBaseObject*>(iter->second.get());
    auto result = test->RunTest();

    _total++;

    if(result.first) {
      std::cout << "PASSED!" << std::endl;
      _passed++;
    }
    else {
      std::cout << "FAILED :(" << std::endl;
      _failed++;
    }

    if(verbose) {
      std::cout << result.second << std::endl;
    }    
  }
}
/*--------------------------------------------------------------------*/
#endif
