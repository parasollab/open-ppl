#ifndef PPL_NUM_NODES_METRIC_TEST_H_
#define PPL_NUM_NODES_METRIC_TEST_H_

#include "MetricMethodTest.h"
#include "MPLibrary/Metrics/NumNodesMetric.h"

class NumNodesMetricTest :  public NumNodesMetric,
                               public MetricMethodTest {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    NumNodesMetricTest();

    NumNodesMetricTest(MPProblem* _problem);

    ~NumNodesMetricTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestMetric() override;

    ///@}

};

/*--------------------------- Construction ---------------------------*/

NumNodesMetricTest::
NumNodesMetricTest() : NumNodesMetric() {}

NumNodesMetricTest::
NumNodesMetricTest(MPProblem* _problem) : MetricMethodTest(),
                                                             NumNodesMetric(){
  m_MPProblem = _problem;
}

NumNodesMetricTest::
~NumNodesMetricTest() {}



/*--------------------- Test Interface Functions ---------------------*/

typename NumNodesMetricTest::TestResult
NumNodesMetricTest::
TestMetric() {

  // Set up environment from parent
  double metric = Metric();
  double expected = this->GetGroupRoadmap() ? (this->GetGroupRoadmap()->Size()) : (this->GetRoadmap()->Size());

  // Correct value?
  if(metric == expected){
    return std::make_pair(true,"Testing NumNodesMetric::PASSED");
  }
  return std::make_pair(true,"Testing NumNodesMetric, Wrong number of nodes recieved.");
}

#endif
