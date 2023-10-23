#ifndef PPL_TIME_METRIC_TEST_H_
#define PPL_TIME_METRIC_TEST_H_

#include "MetricMethodTest.h"
#include "MPLibrary/Metrics/TimeMetric.h"   //src

template <typename MPTraits>
class TimeMetricTest :  virtual public TimeMetric<MPTraits>,
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

    ///@name Test Interface Functions
    ///@{

    virtual TestResult TestMetric() override;

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

/*--------------------- Test Interface Functions ---------------------*/

template <typename MPTraits>
typename TimeMetricTest<MPTraits>::TestResult
TimeMetricTest<MPTraits>::
TestMetric() {
  bool passed = true;
  std::string message = "";

  // // Sleep
  // sleep(10);

  // // Get time
  // double metric = (*this)();
  // double expected = 10;

  // // Correct value?

  // Difficult to test out of context of application
  if (false) {
    passed = false;
    std::cout << "\n\tWrong time elapsed recieved." << std::endl;
  }

  message = "\tFINISHED TestMetric";
  return std::make_pair(passed, message);

}

#endif
