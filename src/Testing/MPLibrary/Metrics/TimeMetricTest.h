#ifndef PPL_TIME_METRIC_TEST_H_
#define PPL_TIME_METRIC_TEST_H_

#include "MetricMethodTest.h"
#include "MPLibrary/Metrics/TimeMetric.h"

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

    TimeMetricTest(MPProblem* _problem); // TODO delete this (see below)

    // TODO you also need a constructor that takes in an XMLNode (see Testing/Samplers/UniformRandomSamplerTest for example)

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
TimeMetricTest() : MetricMethodTest<MPTraits>(), TimeMetric<MPTraits>() {}

template <typename MPTraits>
TimeMetricTest<MPTraits>::
TimeMetricTest(MPProblem* _problem) : MetricMethodTest<MPTraits>(),
                                      TimeMetric<MPTraits>(){
  m_MPProblem = _problem; // TODO you can delete this. You can get the MPProblem with this->GetMPProblem().
}

template <typename MPTraits>
TimeMetricTest<MPTraits>::
~TimeMetricTest() {}

/*--------------------- Test Interface Functions ---------------------*/

template <typename MPTraits>
typename TimeMetricTest<MPTraits>::TestResult
TimeMetricTest<MPTraits>::
TestMetric() {

  // Set up environment from parent
  double metric = Metric();
  auto stats = this->GetStatClass();

  // TODO what is s_clockName? (See TimeMetric.h), but you might not need it
  // You might not want to call Metric() because you don't actually need a
  // roadmap. Maybe just sleep for some time (https://www.cplusplus.com/reference/thread/this_thread/sleep_for/)
  // and then check that the time metric call returns at least that long

  // Report the elapsed time.
  stats->StopClock(s_clockName);
  stats->StartClock(s_clockName);

  double expected = (double)stats->GetSeconds(s_clockName);

  // Correct value?
  if(metric == expected){
    return std::make_pair(true,"Testing TimeMetric::PASSED");
  }
  return std::make_pair(false,"Testing TimeMetric, Wrong time elapsed recieved.");
}

#endif
