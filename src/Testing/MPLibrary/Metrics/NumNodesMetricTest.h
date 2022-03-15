#ifndef PPL_NUM_NODES_METRIC_TEST_H
#define PPL_NUM_NODES_METRIC_TEST_H

#include "MPLibrary/Metrics/NumNodesMetric.h"
#include "Testing/MPLibrary/Metrics/MetricMethodTest.h"

template <typename MPTraits>
class NumNodesMetricTest : virtual public NumNodesMetric<MPTraits>,
                           public MetricMethodTest<MPTraits> {

  public:
  
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    NumNodesMetricTest();

    NumNodesMetricTest(XMLNode& _node);

    ~NumNodesMetricTest();

    ///@}

  private:

    ///@name Interface Test Function
    ///@{

    virtual TestResult MetricTest() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
NumNodesMetricTest<MPTraits>::
NumNodesMetricTest() : NumNodesMetric<MPTraits>() {}

template <typename MPTraits>
NumNodesMetricTest<MPTraits>::
NumNodesMetricTest(XMLNode& _node) : MetricMethod<MPTraits>(_node), 
                                     NumNodesMetric<MPTraits>(_node) {}

template <typename MPTraits>
NumNodesMetricTest<MPTraits>::
~NumNodesMetricTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename NumNodesMetricTest<MPTraits>::TestResult
NumNodesMetricTest<MPTraits>::
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
