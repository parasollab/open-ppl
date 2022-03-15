#ifndef PPL_NUM_EDGES_METRIC_TEST_H
#define PPL_NUM_EDGES_METRIC_TEST_H

#include "MPLibrary/Metrics/NumEdgesMetric.h"
#include "Testing/MPLibrary/Metrics/MetricMethodTest.h"

template <typename MPTraits>
class NumEdgesMetricTest : virtual public NumEdgesMetric<MPTraits>,
                           public MetricMethodTest<MPTraits> {

  public:
  
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    NumEdgesMetricTest();

    NumEdgesMetricTest(XMLNode& _node);

    ~NumEdgesMetricTest();

    ///@}

  private:

    ///@name Interface Test Function
    ///@{

    virtual TestResult MetricTest() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
NumEdgesMetricTest<MPTraits>::
NumEdgesMetricTest() : NumEdgesMetric<MPTraits>() {}

template <typename MPTraits>
NumEdgesMetricTest<MPTraits>::
NumEdgesMetricTest(XMLNode& _node) : MetricMethod<MPTraits>(_node), 
                                     NumEdgesMetric<MPTraits>(_node) {}

template <typename MPTraits>
NumEdgesMetricTest<MPTraits>::
~NumEdgesMetricTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename NumEdgesMetricTest<MPTraits>::TestResult
NumEdgesMetricTest<MPTraits>::
MetricTest() {
  
  bool passed = true;
  std::string message = "";

  
  if (passed)
    message = "MetricTest::PASSED!\n";
  else
    message = "MetricTest::FAILED :(\n" + message;
  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/

#endif
