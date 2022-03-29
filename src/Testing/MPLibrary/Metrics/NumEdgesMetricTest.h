#ifndef PPL_NUM_EDGES_METRIC_TEST_H_
#define PPL_NUM_EDGES_METRIC_TEST_H_

#include "MetricMethodTest.h"
#include "MPLibrary/Metrics/NumEdgesMetric.h"

class NumEdgesMetricTest :  public NumEdgesMetric,
                               public MetricMethodTest {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    NumEdgesMetricTest();

    NumEdgesMetricTest(MPProblem* _problem);

    ~NumEdgesMetricTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestMetric() override;

    ///@}

};

/*--------------------------- Construction ---------------------------*/

NumEdgesMetricTest::
NumEdgesMetricTest() : NumEdgesMetric() {}

NumEdgesMetricTest::
NumEdgesMetricTest(MPProblem* _problem) : MetricMethodTest(),
                                                             NumEdgesMetric(){
  m_MPProblem = _problem;
}

NumEdgesMetricTest::
~NumEdgesMetricTest() {}



/*--------------------- Test Interface Functions ---------------------*/

typename NumEdgesMetricTest::TestResult
NumEdgesMetricTest::
TestMetric() {

  // Set up environment from parent
  double metric = Metric();
  double expected = this->GetGroupRoadmap() ? (this->GetGroupRoadmap()->get_num_edges()) : (this->GetRoadmap()->get_num_edges());

  // Correct value?
  if(metric == expected){
    return std::make_pair(true,"Testing NumEdgesMetric::PASSED");
  }
  return std::make_pair(true,"Testing NumEdgesMetric, Wrong number of edges recieved.");
}

#endif
