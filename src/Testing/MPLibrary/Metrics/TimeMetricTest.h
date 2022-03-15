#ifndef PPL_TIME_METRIC_TEST_H
#define PPL_TIME_METRIC_TEST_H

#include "MPLibrary/Metrics/TimeMetric.h"
#include "Testing/MPLibrary/Metrics/MetricMethodTest.h"

template <typename MPTraits>
class TimeMetricTest : virtual public TimeMetric<MPTraits>,
                           public MetricMethodTest<MPTraits> {

  public:
  
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    TimeMetricTest();

    TimeMetricTest(XMLNode& _node);

    ~TimeMetricTest();

    ///@}

  private:

    ///@name Interface Test Function
    ///@{

    virtual TestResult MetricTest() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
TimeMetricTest<MPTraits>::
TimeMetricTest() : TimeMetric<MPTraits>() {}

template <typename MPTraits>
TimeMetricTest<MPTraits>::
TimeMetricTest(XMLNode& _node) : MetricMethod<MPTraits>(_node), 
                                 TimeMetric<MPTraits>(_node) {}

template <typename MPTraits>
TimeMetricTest<MPTraits>::
~TimeMetricTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename TimeMetricTest<MPTraits>::TestResult
TimeMetricTest<MPTraits>::
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
